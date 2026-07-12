#pragma once

#include <string>
#include <vector>

#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "openarm_impedance_control/impedance_law.hpp"

namespace openarm_impedance_controller {

struct ControllerParams {
  std::vector<std::string> joints;
  std::string robot_description;
  std::string ee_frame_name;

  std::vector<double> joint_torque_limits;
  std::vector<double> cartesian_wrench_limits;

  std::vector<double> k_cartesian;
  std::vector<double> d_cartesian;
  std::vector<double> k_cartesian_scale;
  std::vector<double> d_cartesian_scale;
  std::vector<double> k_nullspace;
  std::vector<double> d_nullspace;

  std::vector<double> cartesian_position_min;
  std::vector<double> cartesian_position_max;
  double joint_position_margin{0.0};

  double cartesian_velocity_limit{0.0};

  ComplianceFrame compliance_frame{ComplianceFrame::kWorld};
  bool do_gravity_compensation{true};
  bool zero_motor_pd{true};
};

ControllerParams readControllerParams(rclcpp_lifecycle::LifecycleNode& node);

}  // namespace openarm_impedance_controller