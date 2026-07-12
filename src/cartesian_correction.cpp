#include "openarm_impedance_control/cartesian_correction.hpp"

#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio/algorithm/frames.hpp"
#include "pinocchio/algorithm/model.hpp"

#include <stdexcept>
#include <iostream>
#include <cmath>

namespace openarm_impedance_control {

CartesianCorrection::CartesianCorrection(
    const std::string& urdf_string,
    const std::vector<std::string>& joint_names,
    const std::string& ee_frame_name,
    std::vector<double> torque_limits,
    const std::vector<double> k_joint_gains,
    const std::vector<double> d_joint_gains,
    bool do_gravity_compensation)
  : torque_limits_(std::move(torque_limits)),
    do_gravity_compensation_(do_gravity_compensation),
    K_joint_(Eigen::Map<const Eigen::VectorXd>(k_joint_gains.data(), k_joint_gains.size()).asDiagonal()),
    D_joint_(Eigen::Map<const Eigen::VectorXd>(d_joint_gains.data(), d_joint_gains.size()).asDiagonal()),
    relax_K_(Eigen::Matrix<double, 6, 6>::Zero()),
    relax_D_(Eigen::Matrix<double, 6, 6>::Zero())
{
  try {
    // Build the full model from URDF (contains every joint: both arms, grippers)
    pinocchio::Model full_model;
    pinocchio::urdf::buildModelFromXML(urdf_string, full_model);

    // Lock every movable joint that this controller doesn't command.
    std::vector<pinocchio::JointIndex> joints_to_lock;
    for (pinocchio::JointIndex jid = 1;
         jid < static_cast<pinocchio::JointIndex>(full_model.njoints); ++jid) {
      const std::string& name = full_model.names[jid];
      if (std::find(joint_names.begin(), joint_names.end(), name) == joint_names.end()) {
        joints_to_lock.push_back(jid);
      }
    }

    // Sanity check: every requested joint must exist in the URDF.
    for (const auto& name : joint_names) {
      if (!full_model.existJointName(name)) {
        throw std::runtime_error("Joint '" + name + "' not found in URDF");
      }
    }

    // Reference configuration for the locked joints (neutral pose).
    const Eigen::VectorXd q_ref = pinocchio::neutral(full_model);

    // Reduce: locked joints are frozen at q_ref, leaving only joint_names.
    pinocchio::buildReducedModel(full_model, joints_to_lock, q_ref, pinocchio_model_);
    pinocchio_data_ = pinocchio::Data(pinocchio_model_);

    ee_frame_id_ = pinocchio_model_.getFrameId(ee_frame_name);
    if (ee_frame_id_ == static_cast<pinocchio::FrameIndex>(pinocchio_model_.nframes)) {
      throw std::runtime_error("EE frame '" + ee_frame_name + "' not found in reduced model");
    }
  } catch (const std::exception& e) {
    throw std::runtime_error(std::string("Impedance controller init failed: ") + e.what());
  }
}

void CartesianCorrection::applyRelaxGains(const std::vector<double>& gains,
                                           const char* label,
                                           Eigen::Matrix<double, 6, 6>& relax) {
  if (gains.size() != 6) {
    throw std::invalid_argument(std::string(label) + " must have exactly 6 elements");
  }
  Eigen::Matrix<double, 6, 1> g = Eigen::Map<const Eigen::Matrix<double, 6, 1>>(gains.data());
  clamp(g, 0.0, 1.0, label);
  relax.setIdentity();
  relax.diagonal() -= g;
}

void CartesianCorrection::setStiffness(const std::vector<double>& k_gains) {
  applyRelaxGains(k_gains, "stiffness gains", relax_K_);
}

void CartesianCorrection::setDamping(const std::vector<double>& d_gains) {
  applyRelaxGains(d_gains, "damping gains", relax_D_);
}

Eigen::VectorXd CartesianCorrection::computeControl(
    const Eigen::VectorXd& q,
    const Eigen::VectorXd& dq,
    const Eigen::VectorXd& q_ref,
    const Eigen::VectorXd& dq_ref)
{
  // Compute the Jacobian at the current joint configuration
  pinocchio::computeJointJacobians(pinocchio_model_, pinocchio_data_, q);

  // 6 x nv Jacobian in world-aligned frame (linear rows first, then angular)
  Eigen::Matrix<double, 6, Eigen::Dynamic> J(6, pinocchio_model_.nv);
  J.setZero();
  pinocchio::getFrameJacobian(
      pinocchio_model_, pinocchio_data_, ee_frame_id_,
      pinocchio::LOCAL_WORLD_ALIGNED, J);


  Eigen::MatrixXd J_pinv = J.completeOrthogonalDecomposition().pseudoInverse();
  Eigen::MatrixXd K_bar = J_pinv.transpose() * K_joint_ * J_pinv;
  Eigen::MatrixXd D_bar = J_pinv.transpose() * D_joint_ * J_pinv;

  Eigen::VectorXd tau = Eigen::VectorXd::Zero(pinocchio_model_.nv);
  tau += J.transpose() * relax_K_ * K_bar * J * (q_ref - q);
  tau += J.transpose() * relax_D_ * D_bar * J * (dq_ref - dq);

  // Add gravity compensation torques if enabled
  if (do_gravity_compensation_) {
    const Eigen::VectorXd zero = Eigen::VectorXd::Zero(pinocchio_model_.nv);
    tau += pinocchio::rnea(pinocchio_model_, pinocchio_data_, q, zero, zero);
  }

  clamp(tau, torque_limits_, "joint torque limits");

  return tau;
}

void CartesianCorrection::clamp(Eigen::Ref<Eigen::VectorXd> v, double lower_limit, double upper_limit, const std::string& context) {
  for (int i = 0; i < v.size(); ++i) {
    if (v(i) < lower_limit) {
      std::cerr << "[CartesianCorrection] Clamping " << context << " [" << i << "] to lower limit " << lower_limit << "\n";
      v(i) = lower_limit;
    } else if (v(i) > upper_limit) {
      std::cerr << "[CartesianCorrection] Clamping " << context << " [" << i << "] to upper limit " << upper_limit << "\n";
      v(i) = upper_limit;
    }
  }
}

void CartesianCorrection::clamp(Eigen::Ref<Eigen::VectorXd> v, const std::vector<double>& limits, const std::string& context) {
  for (size_t i = 0; i < limits.size(); ++i) {
    if (v(i) < -limits[i]) {
      std::cerr << "[CartesianCorrection] Clamping " << context << " [" << i << "] to lower limit " << -limits[i] << "\n";
      v(i) = -limits[i];
    } else if (v(i) > limits[i]) {
      std::cerr << "[CartesianCorrection] Clamping " << context << " [" << i << "] to upper limit " << limits[i] << "\n";
      v(i) = limits[i];
    }
  }
}

}  // namespace openarm_impedance_control