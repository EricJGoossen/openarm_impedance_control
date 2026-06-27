#include "openarm_impedance_control/impedance.hpp"

#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio/algorithm/frames.hpp"
#include "pinocchio/algorithm/model.hpp"

#include <stdexcept>
#include <iostream>
#include <cmath>

namespace openarm_impedance_control {

Impedance::Impedance(
    const std::string& urdf_string,
    const std::vector<std::string>& joint_names,
    const std::string& ee_frame_name,
    const std::vector<double>& torque_limits,
    bool do_gravity_compensation,
    bool do_damping,
    bool do_stiffness)
  : torque_limits_(torque_limits),
    do_gravity_compensation_(do_gravity_compensation),
    do_damping_(do_damping),
    do_stiffness_(do_stiffness),
    K_(Eigen::Matrix<double, 6, 6>::Zero()),
    D_(Eigen::Matrix<double, 6, 6>::Zero())
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
    throw std::runtime_error(std::string("CartesianImpedance init failed: ") + e.what());
  }
}

void Impedance::setStiffness(const std::vector<double>& k_gains) {
  K_ = Eigen::Map<const Eigen::Matrix<double, 6, 1>>(k_gains.data()).asDiagonal();
}

void Impedance::setDamping(const std::vector<double>& d_gains) {
  D_ = Eigen::Map<const Eigen::Matrix<double, 6, 1>>(d_gains.data()).asDiagonal();
}

Eigen::VectorXd Impedance::computeControl(
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

  // Compute joint space K and D matrices
  const int nv = pinocchio_model_.nv;
  Eigen::MatrixXd Kq = J.transpose() * K_ * J
                    + kEpsilon * Eigen::MatrixXd::Identity(nv, nv);
  Eigen::MatrixXd Dq = J.transpose() * D_ * J;

  // Compute torques from impedance control law
  Eigen::VectorXd tau = Eigen::VectorXd::Zero(pinocchio_model_.nv);
  if (do_stiffness_) tau += Kq * (q_ref - q);
  if (do_damping_)   tau += Dq * (dq_ref - dq);

  // Add gravity compensation torques if enabled
  if (do_gravity_compensation_) {
    const Eigen::VectorXd zero = Eigen::VectorXd::Zero(pinocchio_model_.nv);
    tau += pinocchio::rnea(pinocchio_model_, pinocchio_data_, q, zero, zero);
  }

  clamp(tau, torque_limits_);

  return tau;
}

void Impedance::clamp(Eigen::Ref<Eigen::VectorXd> v, const std::vector<double>& limits) {
  for (size_t i = 0; i < limits.size(); ++i) {
    if (v(i) < -limits[i]) {
      std::cerr << "[Impedance] Clamping [" << i << "] to lower limit " << -limits[i] << "\n";
      v(i) = -limits[i];
    } else if (v(i) > limits[i]) {
      std::cerr << "[Impedance] Clamping [" << i << "] to upper limit " << limits[i] << "\n";
      v(i) = limits[i];
    }
  }
}

}  // namespace openarm_impedance