#include "openarm_impedance_control/robot_model.hpp"

#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio/algorithm/frames.hpp"
#include "pinocchio/algorithm/model.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"

#include <algorithm>
#include <stdexcept>

namespace openarm_impedance_controller {

RobotModel::RobotModel(const std::string& urdf_string,
                       const std::vector<std::string>& joint_names,
                       const std::string& ee_frame_name) {
  try {
    pinocchio::Model full_model;
    pinocchio::urdf::buildModelFromXML(urdf_string, full_model);

    for (const auto& name : joint_names) {
      if (!full_model.existJointName(name)) {
        throw std::runtime_error("Joint '" + name + "' not found in URDF");
      }
    }

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
    data_rt_    = pinocchio::Data(pinocchio_model_);
    data_nonrt_ = pinocchio::Data(pinocchio_model_);

    ee_frame_id_ = pinocchio_model_.getFrameId(ee_frame_name);
    if (ee_frame_id_ == static_cast<pinocchio::FrameIndex>(pinocchio_model_.nframes)) {
      throw std::runtime_error("EE frame '" + ee_frame_name + "' not found in reduced model");
    }

    for (pinocchio::JointIndex jid = 1;
        jid < static_cast<pinocchio::JointIndex>(pinocchio_model_.njoints); ++jid) {
      const size_t i = static_cast<size_t>(jid) - 1;
      if (i >= joint_names.size() || pinocchio_model_.names[jid] != joint_names[i]) {
        throw std::runtime_error(
            "Reduced-model joint order does not match the `joints` parameter order "
            "(index " + std::to_string(i) + ": model has '" +
            pinocchio_model_.names[jid] + "'). Reorder `joints` to match the URDF tree.");
      }
    }

    if (pinocchio_model_.nq != pinocchio_model_.nv) {
      throw std::runtime_error("This controller assumes one coordinate per DOF (all revolute joints)");
    }

    q_lower_  = pinocchio_model_.lowerPositionLimit;
    q_upper_  = pinocchio_model_.upperPositionLimit;
    dq_limit_ = pinocchio_model_.velocityLimit;

    J_rt_.resize(6, pinocchio_model_.nv);
  } catch (const std::exception& e) {
    throw std::runtime_error(std::string("RobotModel init failed: ") + e.what());
  }
}

Eigen::VectorXd RobotModel::gravityTorqueRT(const Eigen::VectorXd& q) {
  const Eigen::VectorXd zero = Eigen::VectorXd::Zero(pinocchio_model_.nv);
  return pinocchio::rnea(pinocchio_model_, data_rt_, q, zero, zero);
}

Eigen::VectorXd RobotModel::gravityTorqueNonRT(const Eigen::VectorXd& q) const {
  const Eigen::VectorXd zero = Eigen::VectorXd::Zero(pinocchio_model_.nv);
  return pinocchio::rnea(pinocchio_model_, data_nonrt_, q, zero, zero);
}

Eigen::Vector3d RobotModel::tcpPositionNonRT(const Eigen::VectorXd& q) const {
  pinocchio::forwardKinematics(pinocchio_model_, data_nonrt_, q);
  pinocchio::updateFramePlacements(pinocchio_model_, data_nonrt_);
  return data_nonrt_.oMf[ee_frame_id_].translation();
}

const Eigen::Matrix<double, 6, Eigen::Dynamic>& RobotModel::jacobianRT(const Eigen::VectorXd& q) {
  pinocchio::computeJointJacobians(pinocchio_model_, data_rt_, q);
  pinocchio::updateFramePlacements(pinocchio_model_, data_rt_);
  J_rt_.setZero();
  pinocchio::getFrameJacobian(pinocchio_model_, data_rt_, ee_frame_id_,
                             pinocchio::LOCAL_WORLD_ALIGNED, J_rt_);
  return J_rt_;
}

Eigen::Vector3d RobotModel::lastTcpPositionRT() const {
  return data_rt_.oMf[ee_frame_id_].translation();
}

}  // namespace openarm_impedance_controller