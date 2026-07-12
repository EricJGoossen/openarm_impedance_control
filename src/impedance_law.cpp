#include "openarm_impedance_control/impedance_law.hpp"

#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio/algorithm/frames.hpp"
#include "pinocchio/algorithm/model.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace openarm_impedance_controller {

ImpedanceLaw::ImpedanceLaw(
    const std::string& urdf_string,
    const std::vector<std::string>& joint_names,
    const std::string& ee_frame_name,
    std::vector<double> torque_limits,
    std::vector<double> cartesian_wrench_limits,
    bool do_gravity_compensation)
  : torque_limits_(std::move(torque_limits)),
    cartesian_wrench_limits_(std::move(cartesian_wrench_limits)),
    do_gravity_compensation_(do_gravity_compensation)
{
  try {
    // Build the full model from URDF 
    pinocchio::Model full_model;
    pinocchio::urdf::buildModelFromXML(urdf_string, full_model);

    // Sanity check: every requested joint must exist in the URDF.
    for (const auto& name : joint_names) {
      if (!full_model.existJointName(name)) {
        throw std::runtime_error("Joint '" + name + "' not found in URDF");
      }
    }

    // Lock every movable joint that this controller doesn't command.
    std::vector<pinocchio::JointIndex> joints_to_lock;
    for (pinocchio::JointIndex jid = 1;
         jid < static_cast<pinocchio::JointIndex>(full_model.njoints); ++jid) {
      const std::string& name = full_model.names[jid];
      if (std::find(joint_names.begin(), joint_names.end(), name) == joint_names.end()) {
        joints_to_lock.push_back(jid);
      }
    }

    const Eigen::VectorXd q_ref = pinocchio::neutral(full_model);
    pinocchio::buildReducedModel(full_model, joints_to_lock, q_ref, pinocchio_model_);
    pinocchio_data_  = pinocchio::Data(pinocchio_model_);
    validation_data_ = pinocchio::Data(pinocchio_model_);

    ee_frame_id_ = pinocchio_model_.getFrameId(ee_frame_name);
    if (ee_frame_id_ == static_cast<pinocchio::FrameIndex>(pinocchio_model_.nframes)) {
      throw std::runtime_error("EE frame '" + ee_frame_name + "' not found in reduced model");
    }

    // The reduced model's joint order must match the param order.
    for (pinocchio::JointIndex jid = 1;
         jid < static_cast<pinocchio::JointIndex>(pinocchio_model_.njoints); ++jid) {
      const size_t i = static_cast<size_t>(jid) - 1;
      if (i >= joint_names.size() || pinocchio_model_.names[jid] != joint_names[i]) {
        throw std::runtime_error(
            "Reduced-model joint order does not match the `joints` parameter order "
            "(index " + std::to_string(i) + ": model has '" +
            pinocchio_model_.names[jid] + "'). Reorder `joints` to match the URDF "
            "tree, or add an index map before commanding torque.");
      }
    }

    // q packing and the torque-clamp bitmask assume one configuration
    // coordinate per DOF, true for a revolute chain; assert rather than
    // misindex silently if a free/planar joint ever appears.
    if (pinocchio_model_.nq != pinocchio_model_.nv) {
      throw std::runtime_error("This controller assumes one coordinate per DOF (all revolute joints)");
    }
    if (pinocchio_model_.nv > 32) {
      throw std::runtime_error("More than 32 DOF: the torque-clamp bitmask would overflow");
    }

    // Joint position limits come straight from the URDF.
    q_lower_ = pinocchio_model_.lowerPositionLimit;
    q_upper_ = pinocchio_model_.upperPositionLimit;
    dq_limit_ = pinocchio_model_.velocityLimit;

    if (torque_limits_.size() != static_cast<size_t>(pinocchio_model_.nv)) {
      throw std::runtime_error("joint_torque_limits must have " +
                               std::to_string(pinocchio_model_.nv) + " elements");
    }
    if (cartesian_wrench_limits_.size() != 6) {
      throw std::runtime_error("cartesian_wrench_limits must have exactly 6 elements "
                               "[fx fy fz mx my mz]");
    }
    for (double t : torque_limits_) {
      if (!std::isfinite(t) || t <= 0.0) {
        throw std::invalid_argument("joint_torque_limits must be finite and positive");
      }
    }
    for (double t : cartesian_wrench_limits_) {
      if (!std::isfinite(t) || t <= 0.0) {
        throw std::invalid_argument("cartesian_wrench_limits must be finite and positive");
      }
    }

    const int nv = pinocchio_model_.nv;
    k_null_ = Eigen::VectorXd::Zero(nv);
    d_null_ = Eigen::VectorXd::Zero(nv);

    J_.resize(6, nv);
    J_pinv_.resize(nv, 6);
    N_.resize(nv, nv);
    tau_.resize(nv);
    q_err_.resize(nv);
    dq_err_.resize(nv);
  } catch (const std::exception& e) {
    throw std::runtime_error(std::string("Impedance controller init failed: ") + e.what());
  }
}

// Gain setters

ImpedanceLaw::Vector6d ImpedanceLaw::checkSix(
    const std::vector<double>& v, const char* label) {
  if (v.size() != 6) {
    throw std::invalid_argument(std::string(label) + " must have exactly 6 elements");
  }
  for (double x : v) {
    if (!std::isfinite(x) || x < 0.0) {
      throw std::invalid_argument(std::string(label) + " must be finite and non-negative");
    }
  }
  return Eigen::Map<const Vector6d>(v.data());
}

Eigen::VectorXd ImpedanceLaw::checkNv(
    const std::vector<double>& v, const char* label) const {
  if (v.size() != static_cast<size_t>(pinocchio_model_.nv)) {
    throw std::invalid_argument(std::string(label) + " must have exactly " +
                                std::to_string(pinocchio_model_.nv) + " elements");
  }
  for (double x : v) {
    if (!std::isfinite(x) || x < 0.0) {
      throw std::invalid_argument(std::string(label) + " must be finite and non-negative");
    }
  }
  return Eigen::Map<const Eigen::VectorXd>(v.data(), v.size());
}

void ImpedanceLaw::setCartesianStiffness(const std::vector<double>& k_cart) {
  k_cart_ = checkSix(k_cart, "k_cartesian");
}

void ImpedanceLaw::setCartesianDamping(const std::vector<double>& d_cart) {
  d_cart_ = checkSix(d_cart, "d_cartesian");
}

void ImpedanceLaw::setStiffnessScale(const std::vector<double>& kappa_k) {
  Vector6d k = checkSix(kappa_k, "k_cartesian_scale");
  kappa_k_ = k.cwiseMin(1.0).cwiseMax(0.0);
}

void ImpedanceLaw::setDampingScale(const std::vector<double>& kappa_d) {
  Vector6d d = checkSix(kappa_d, "d_cartesian_scale");
  kappa_d_ = d.cwiseMin(1.0).cwiseMax(0.0);
}

void ImpedanceLaw::setNullspaceStiffness(const std::vector<double>& k_null) {
  k_null_ = checkNv(k_null, "k_nullspace");
}

void ImpedanceLaw::setNullspaceDamping(const std::vector<double>& d_null) {
  d_null_ = checkNv(d_null, "d_nullspace");
}

// FK for goal validation (non-RT)

Eigen::Vector3d ImpedanceLaw::tcpPosition(const Eigen::VectorXd& q) const {
  if (q.size() != pinocchio_model_.nq) {
    throw std::invalid_argument("tcpPosition: q has the wrong size");
  }
  pinocchio::forwardKinematics(pinocchio_model_, validation_data_, q);
  pinocchio::updateFramePlacements(pinocchio_model_, validation_data_);
  return validation_data_.oMf[ee_frame_id_].translation();
}

// Control law

Eigen::VectorXd ImpedanceLaw::computeControl(
    const Eigen::VectorXd& q,
    const Eigen::VectorXd& dq,
    const Eigen::VectorXd& q_ref,
    const Eigen::VectorXd& dq_ref)
{
  const int nv = pinocchio_model_.nv;

  // Jacobian at the TCP, world-aligned axes.
  pinocchio::computeJointJacobians(pinocchio_model_, pinocchio_data_, q);
  pinocchio::updateFramePlacements(pinocchio_model_, pinocchio_data_);

  J_.setZero();
  pinocchio::getFrameJacobian(
      pinocchio_model_, pinocchio_data_, ee_frame_id_,
      pinocchio::LOCAL_WORLD_ALIGNED, J_);

  // Build the Cartesian gain matrices directly. 
  Matrix6d K_cart = (kappa_k_.cwiseProduct(k_cart_)).asDiagonal();
  Matrix6d D_cart = (kappa_d_.cwiseProduct(d_cart_)).asDiagonal();

  // If the compliance axes are tool-fixed, rotate the diagonals into the world
  // frame the Jacobian is expressed in: K_w = S K S^T, S = blkdiag(R, R).
  if (compliance_frame_ == ComplianceFrame::kTcp) {
    const Eigen::Matrix3d R = pinocchio_data_.oMf[ee_frame_id_].rotation();
    Matrix6d S = Matrix6d::Zero();
    S.topLeftCorner<3, 3>()     = R;
    S.bottomRightCorner<3, 3>() = R;
    K_cart = S * K_cart * S.transpose();
    D_cart = S * D_cart * S.transpose();
  }

  q_err_  = q  - q_ref;
  dq_err_ = dq - dq_ref;

  // Task term
  Vector6d F = -(K_cart * (J_ * q_err_)) - (D_cart * (J_ * dq_err_));
  wrench_clamp_mask_ = clamp(F, cartesian_wrench_limits_);

  tau_.noalias() = J_.transpose() * F;

  // Null-space term. 
  const Matrix6d JJt = J_ * J_.transpose() +
                       kLambda2 * Matrix6d::Identity();
  J_pinv_.noalias() = J_.transpose() * JJt.inverse();
  N_ = Eigen::MatrixXd::Identity(nv, nv);
  N_.noalias() -= J_pinv_ * J_;

  const Eigen::VectorXd tau_null =
      -(k_null_.asDiagonal() * q_err_) - (d_null_.asDiagonal() * dq_err_);
  tau_.noalias() += N_ * tau_null;

  // Gravity compensation. 
  if (do_gravity_compensation_) {
    const Eigen::VectorXd zero = Eigen::VectorXd::Zero(nv);
    tau_ += pinocchio::rnea(pinocchio_model_, pinocchio_data_, q, zero, zero);
  }

  torque_clamp_mask_ = clamp(tau_, torque_limits_);
  return tau_;
}

std::uint32_t ImpedanceLaw::clamp(Eigen::Ref<Eigen::VectorXd> v,
                                  const std::vector<double>& limits) {
  std::uint32_t mask = 0;
  for (size_t i = 0; i < limits.size(); ++i) {
    const auto idx = static_cast<Eigen::Index>(i);
    if (v(idx) < -limits[i]) {
      v(idx) = -limits[i];
      mask |= (1u << i);
    } else if (v(idx) > limits[i]) {
      v(idx) = limits[i];
      mask |= (1u << i);
    }
  }
  return mask;
}

}  // namespace openarm_impedance_controller