#pragma once

#include <string>
#include <vector>

#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Geometry"
#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/multibody/model.hpp"
#include "pinocchio/multibody/data.hpp"

namespace openarm_impedance_controller {

class RobotModel {
 public:
  RobotModel(const std::string& urdf_string,
            const std::vector<std::string>& joint_names,
            const std::string& ee_frame_name);

  int nv() const { return pinocchio_model_.nv; }
  int nq() const { return pinocchio_model_.nq; }

  const Eigen::VectorXd& jointLowerLimits() const { return q_lower_; }
  const Eigen::VectorXd& jointUpperLimits() const { return q_upper_; }
  const Eigen::VectorXd& jointVelocityLimits() const { return dq_limit_; }

  // Gravity torque at q 
  Eigen::VectorXd gravityTorqueRT(const Eigen::VectorXd& q);
  Eigen::VectorXd gravityTorqueNonRT(const Eigen::VectorXd& q) const;

  // FK for the TCP position (non-RT: goal validation).
  Eigen::Vector3d tcpPositionNonRT(const Eigen::VectorXd& q) const;

  // Jacobian (6 x nv, LOCAL_WORLD_ALIGNED) at q, RT thread only. Also updates
  // the RT frame placement, so lastTcpPositionRT() reflects this same q.
  const Eigen::Matrix<double, 6, Eigen::Dynamic>& jacobianRT(const Eigen::VectorXd& q);
  Eigen::Vector3d lastTcpPositionRT() const;

 private:
  pinocchio::Model        pinocchio_model_;
  pinocchio::Data         data_rt_;
  mutable pinocchio::Data data_nonrt_;
  pinocchio::FrameIndex   ee_frame_id_;

  Eigen::VectorXd q_lower_;
  Eigen::VectorXd q_upper_;
  Eigen::VectorXd dq_limit_;

  Eigen::Matrix<double, 6, Eigen::Dynamic> J_rt_;
};

}  // namespace openarm_impedance_controller