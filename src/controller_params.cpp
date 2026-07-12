#include "openarm_impedance_control/controller_params.hpp"

#include <cmath>
#include <stdexcept>

namespace openarm_impedance_controller {

ControllerParams readControllerParams(rclcpp_lifecycle::LifecycleNode& node) {
  ControllerParams p;

  p.joints = node.get_parameter("joints").as_string_array();
  if (p.joints.empty()) {
    throw std::invalid_argument("`joints` parameter is empty");
  }

  p.robot_description = node.get_parameter("robot_description").as_string();
  if (p.robot_description.empty()) {
    throw std::invalid_argument("robot_description parameter is empty");
  }

  p.ee_frame_name = node.get_parameter("ee_frame_name").as_string();

  p.joint_torque_limits     = node.get_parameter("joint_torque_limits").as_double_array();
  p.cartesian_wrench_limits = node.get_parameter("cartesian_wrench_limits").as_double_array();

  p.k_cartesian       = node.get_parameter("k_cartesian").as_double_array();
  p.d_cartesian       = node.get_parameter("d_cartesian").as_double_array();
  p.k_cartesian_scale = node.get_parameter("k_cartesian_scale").as_double_array();
  p.d_cartesian_scale = node.get_parameter("d_cartesian_scale").as_double_array();
  p.k_nullspace       = node.get_parameter("k_nullspace").as_double_array();
  p.d_nullspace       = node.get_parameter("d_nullspace").as_double_array();

  p.cartesian_position_min = node.get_parameter("cartesian_position_min").as_double_array();
  p.cartesian_position_max = node.get_parameter("cartesian_position_max").as_double_array();
  p.joint_position_margin  = node.get_parameter("joint_position_margin").as_double();

  p.cartesian_velocity_limit = node.get_parameter("cartesian_velocity_limit").as_double();
  if (!std::isfinite(p.cartesian_velocity_limit) || p.cartesian_velocity_limit < 0.0) {
    throw std::invalid_argument(
        "cartesian_velocity_limit must be finite and non-negative (0 disables it)");
  }

  const std::string frame_str = node.get_parameter("compliance_frame").as_string();
  if (frame_str == "world") {
    p.compliance_frame = ComplianceFrame::kWorld;
  } else if (frame_str == "tcp") {
    p.compliance_frame = ComplianceFrame::kTcp;
  } else {
    throw std::invalid_argument(
        "compliance_frame must be 'world' or 'tcp', got '" + frame_str + "'");
  }

  p.do_gravity_compensation = node.get_parameter("do_gravity_compensation").as_bool();
  p.zero_motor_pd           = node.get_parameter("zero_motor_pd").as_bool();

  return p;
}

}  // namespace openarm_impedance_controller