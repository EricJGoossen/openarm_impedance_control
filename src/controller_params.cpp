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

  p.joint_torque_limits      = node.get_parameter("joint_torque_limits").as_double_array();
  p.cartesian_wrench_limits  = node.get_parameter("cartesian_wrench_limits").as_double_array();

  p.cartesian_position_min = node.get_parameter("cartesian_position_min").as_double_array();
  p.cartesian_position_max = node.get_parameter("cartesian_position_max").as_double_array();
  p.joint_position_margin  = node.get_parameter("joint_position_margin").as_double();

  p.cartesian_velocity_limit = node.get_parameter("cartesian_velocity_limit").as_double();
  if (!std::isfinite(p.cartesian_velocity_limit) || p.cartesian_velocity_limit < 0.0) {
    throw std::invalid_argument(
        "cartesian_velocity_limit must be finite and non-negative (0 disables it)");
  }

  p.do_gravity_compensation = node.get_parameter("do_gravity_compensation").as_bool();

  p.motor_gains_file = node.get_parameter("motor_gains_file").as_string();
  if (p.motor_gains_file.empty()) {
    throw std::invalid_argument("motor_gains_file parameter is empty");
  }

  return p;
}

}  // namespace openarm_impedance_controller