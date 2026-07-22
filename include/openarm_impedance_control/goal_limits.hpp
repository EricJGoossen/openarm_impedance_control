#pragma once

#include <string>
#include <vector>

#include "eigen3/Eigen/Core"
#include "rclcpp/rclcpp.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"

namespace openarm_impedance_controller {

class RobotModel;

class GoalLimits {
 public:
  // Throws std::invalid_argument if the Cartesian box or margin is malformed.
  GoalLimits(std::vector<std::string> joint_names,
            Eigen::VectorXd q_lower, Eigen::VectorXd q_upper,
            double joint_position_margin,
            std::vector<double> cartesian_min,
            std::vector<double> cartesian_max);

  // Joint-space box + Cartesian workspace box. 
  bool validate(const trajectory_msgs::msg::JointTrajectory& traj,
               const std::vector<size_t>& joint_map,
               const RobotModel& model,
               const rclcpp::Logger& logger) const;

  void reportViolations(const Eigen::VectorXd& q, const Eigen::Vector3d& tcp,
                        const rclcpp::Logger& logger, rclcpp::Clock& clock,
                        int throttle_ms) const;

  void logSummary(const rclcpp::Logger& logger) const;

  bool cartesianEnabled() const { return cartesian_enabled_; }

  bool validatePoint(const Eigen::VectorXd& q, const RobotModel& model,
                     const rclcpp::Logger& logger) const;

 private:
  bool checkPoint(const Eigen::VectorXd& q_pt, const RobotModel& model,
                  const rclcpp::Logger& logger, const std::string& label) const;

  std::vector<std::string> joint_names_;
  Eigen::VectorXd q_lower_;
  Eigen::VectorXd q_upper_;
  double joint_position_margin_;

  std::vector<double> cartesian_min_;
  std::vector<double> cartesian_max_;
  bool cartesian_enabled_{false};
};

}  // namespace openarm_impedance_controller