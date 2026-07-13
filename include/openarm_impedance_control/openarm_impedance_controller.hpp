#pragma once

#include <atomic>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "controller_interface/controller_interface.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/node_interfaces/node_parameters_interface.hpp"
#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Geometry"

#include "openarm_impedance_control/robot_model.hpp"
#include "openarm_impedance_control/goal_limits.hpp"

namespace openarm_impedance_controller {

class OpenArmImpedanceController : public controller_interface::ControllerInterface {
 public:
  OpenArmImpedanceController() = default;
  ~OpenArmImpedanceController() = default;

  controller_interface::CallbackReturn on_init() override;
  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State& state) override;
  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& state) override;
  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& state) override;

  controller_interface::return_type update(
      const rclcpp::Time& time, const rclcpp::Duration& period) override;

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

 private:
  using FollowJointTrajectoryAction = control_msgs::action::FollowJointTrajectory;
  using GoalHandle = rclcpp_action::ServerGoalHandle<FollowJointTrajectoryAction>;
  using JointTrajectory = trajectory_msgs::msg::JointTrajectory;

  // Configuration
  std::vector<std::string> joint_names_;
  std::optional<RobotModel> robot_model_;
  std::optional<GoalLimits> goal_limits_;

  std::vector<double> joint_torque_limits_;   // backstop clamp on the RT effort command
  std::atomic<bool> do_gravity_compensation_{true};

  // Runtime velocity trip (unrelated to the old impedance law -- a hard
  // safety backstop that stays regardless of control scheme).
  double cartesian_velocity_limit_{0.0};
  bool cartesian_velocity_limit_enabled_{false};

  // Trajectory tracking
  realtime_tools::RealtimeBuffer<std::shared_ptr<JointTrajectory>> trajectory_buffer_;
  rclcpp::Time trajectory_start_time_;

  // Current reference: the position/velocity setpoint handed straight to the
  // motors' onboard PD (position + velocity command interfaces).
  Eigen::VectorXd q_ref_;
  Eigen::VectorXd dq_ref_;

  // Action server
  rclcpp_action::Server<FollowJointTrajectoryAction>::SharedPtr action_server_;

  mutable std::mutex goal_mutex_;
  std::shared_ptr<GoalHandle> active_goal_;  // guarded by goal_mutex_

  // Feedback
  std::shared_ptr<FollowJointTrajectoryAction::Feedback> feedback_msg_;
  size_t feedback_count_{0};
  static constexpr size_t kFeedbackStride = 6;

  // Throttle period (ms) for the runtime limit warnings.
  static constexpr int kLimitWarnPeriodMs = 1000;

  // Parameter callback
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

  // Helpers
  void interpolateTrajectory(const JointTrajectory& traj, const rclcpp::Time& time);
  void updateFeedback(const Eigen::VectorXd& q, const Eigen::VectorXd& dq);

  // map[k] = index of joint_names_[k] within goal_names.
  std::vector<size_t> buildJointMap(const std::vector<std::string>& goal_names) const;

  // Clamps the gravity-comp effort to joint_torque_limits_ (a backstop --
  // goal validation should already prevent this from firing) and logs which
  // joints, throttled.
  void clampAndReportEffort(Eigen::VectorXd& tau);

  // True if the measured joint velocities or TCP speed exceed their limits.
  bool checkVelocityTrip(const Eigen::VectorXd& dq, const Eigen::Vector3d& tcp_vel);

  // Logs a hardware_interface set_value() failure. Throttled.
  void reportInterfaceWriteFailure(const char* what, size_t joint_idx);

  rcl_interfaces::msg::SetParametersResult onParameterChange(
      const std::vector<rclcpp::Parameter>& params);

  rclcpp_action::GoalResponse onGoalRequest(
      const rclcpp_action::GoalUUID& uuid,
      std::shared_ptr<const FollowJointTrajectoryAction::Goal> goal);

  rclcpp_action::CancelResponse onCancelRequest(
      std::shared_ptr<GoalHandle> goal_handle);

  void onGoalAccepted(std::shared_ptr<GoalHandle> goal_handle);
};

}  // namespace openarm_impedance_controller