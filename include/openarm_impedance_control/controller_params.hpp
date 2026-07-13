#pragma once

#include <string>
#include <vector>

#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace openarm_impedance_controller {

struct ControllerParams {
  std::vector<std::string> joints;
  std::string robot_description;
  std::string ee_frame_name;

  std::vector<double> joint_torque_limits;
  std::vector<double> cartesian_wrench_limits;  

  std::vector<double> cartesian_position_min;
  std::vector<double> cartesian_position_max;
  double joint_position_margin{0.0};

  double cartesian_velocity_limit{0.0};

  bool do_gravity_compensation{true};

  std::string motor_gains_file;
};

ControllerParams readControllerParams(rclcpp_lifecycle::LifecycleNode& node);

}  // namespace openarm_impedance_controller