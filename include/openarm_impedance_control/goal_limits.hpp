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
  // Throws std::invalid_argument if the Cartesian box, margin, or per-joint
  // vectors (torque limits / motor gains) are malformed.
  GoalLimits(std::vector<std::string> joint_names,
            Eigen::VectorXd q_lower, Eigen::VectorXd q_upper,
            double joint_position_margin,
            std::vector<double> cartesian_min,
            std::vector<double> cartesian_max,
            std::vector<double> joint_torque_limits,
            std::vector<double> motor_kp,
            std::vector<double> motor_kd);

  bool validate(const trajectory_msgs::msg::JointTrajectory& traj,
               const std::vector<size_t>& joint_map,
               const Eigen::VectorXd& q_ref_start,
               const Eigen::VectorXd& dq_ref_start,
               const RobotModel& model,
               const rclcpp::Logger& logger) const;

  void reportViolations(const Eigen::VectorXd& q, const Eigen::Vector3d& tcp,
                        const rclcpp::Logger& logger, rclcpp::Clock& clock,
                        int throttle_ms) const;

  void logSummary(const rclcpp::Logger& logger) const;

  bool cartesianEnabled() const { return cartesian_enabled_; }

 private:
  std::vector<std::string> joint_names_;
  Eigen::VectorXd q_lower_;
  Eigen::VectorXd q_upper_;
  double joint_position_margin_;

  std::vector<double> cartesian_min_;
  std::vector<double> cartesian_max_;
  bool cartesian_enabled_{false};

  std::vector<double> joint_torque_limits_;
  std::vector<double> motor_kp_;
  std::vector<double> motor_kd_;

  // Estimates the worst-case joint torque commanded moving from (q0,dq0) to
  // (q1,dq1) in one segment.
  bool checkSegmentTorque(const Eigen::VectorXd& q0, const Eigen::VectorXd& dq0,
                          const Eigen::VectorXd& q1, const Eigen::VectorXd& dq1,
                          size_t point_index, const RobotModel& model,
                          const rclcpp::Logger& logger) const;
};

}  // namespace openarm_impedance_controller