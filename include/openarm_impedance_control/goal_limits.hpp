#pragma once

#include <string>
#include <vector>

#include "eigen3/Eigen/Core"
#include "rclcpp/rclcpp.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"

namespace openarm_impedance_controller {

class ImpedanceLaw;  // validate() only needs its (non-RT) tcpPosition()

class GoalLimits {
 public:
  // Throws std::invalid_argument if the Cartesian box or margin is malformed.
  GoalLimits(std::vector<std::string> joint_names,
            Eigen::VectorXd q_lower, Eigen::VectorXd q_upper,
            double joint_position_margin,
            std::vector<double> cartesian_min,
            std::vector<double> cartesian_max);

  // Checks if each waypoint in the trajectory is inside the joint-space and Cartesian boxes.
  bool validate(const trajectory_msgs::msg::JointTrajectory& traj,
               const std::vector<size_t>& joint_map,
               const ImpedanceLaw& impedance_law,
               const rclcpp::Logger& logger) const;

  // Loggs a summary of the joint and Cartesian limits, and which joints have no URDF limit.
  void reportViolations(const Eigen::VectorXd& q, const Eigen::Vector3d& tcp,
                        const rclcpp::Logger& logger, rclcpp::Clock& clock,
                        int throttle_ms) const;

  // One-time startup diagnostics: which joints have no URDF limit, and
  // whether the Cartesian box is configured at all.
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
};

}  // namespace openarm_impedance_controller