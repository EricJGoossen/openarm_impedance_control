#include "openarm_impedance_control/openarm_impedance_controller.hpp"
#include "openarm_impedance_control/controller_params.hpp"
#include "openarm_impedance_control/motor_gains.hpp"

#include "pluginlib/class_list_macros.hpp"

#include <algorithm>
#include <cmath>

namespace openarm_impedance_controller {

// Lifecycle

controller_interface::CallbackReturn OpenArmImpedanceController::on_init() {
  auto_declare<std::vector<std::string>>("joints", std::vector<std::string>{});
  auto_declare<std::string>("robot_description", "");
  auto_declare<std::string>("ee_frame_name", "tool0");
  auto_declare<std::vector<double>>("joint_torque_limits", std::vector<double>(7, 1.0));
  auto_declare<std::vector<double>>("cartesian_position_min", std::vector<double>{});
  auto_declare<std::vector<double>>("cartesian_position_max", std::vector<double>{});
  auto_declare<double>("joint_position_margin", 0.0);
  auto_declare<double>("cartesian_velocity_limit", 0.0);
  auto_declare<bool>("do_gravity_compensation", true);
  auto_declare<std::string>("motor_gains_file", "");

  ControllerParams params;
  try {
    params = readControllerParams(*get_node());
  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_node()->get_logger(), "Bad parameters: %s", e.what());
    return CallbackReturn::ERROR;
  }

  joint_names_              = params.joints;
  joint_torque_limits_      = params.joint_torque_limits;
  do_gravity_compensation_  = params.do_gravity_compensation;
  cartesian_velocity_limit_ = params.cartesian_velocity_limit;
  cartesian_velocity_limit_enabled_ = cartesian_velocity_limit_ > 0.0;

  if (joint_torque_limits_.size() != joint_names_.size()) {
    RCLCPP_ERROR(get_node()->get_logger(),
      "joint_torque_limits must have %zu elements (one per joint)", joint_names_.size());
    return CallbackReturn::ERROR;
  }
  for (double t : joint_torque_limits_) {
    if (!std::isfinite(t) || t <= 0.0) {
      RCLCPP_ERROR(get_node()->get_logger(), "joint_torque_limits must be finite and positive");
      return CallbackReturn::ERROR;
    }
  }

  try {
    robot_model_.emplace(params.robot_description, joint_names_, params.ee_frame_name);
  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to construct RobotModel: %s", e.what());
    return CallbackReturn::ERROR;
  }

  std::vector<double> motor_kp(joint_names_.size()), motor_kd(joint_names_.size());
  try {
    MotorGains gains(params.motor_gains_file);
    for (size_t i = 0; i < joint_names_.size(); ++i) {
      gains.lookup(joint_names_[i], motor_kp[i], motor_kd[i]);
    }
  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_node()->get_logger(),
      "Failed to load motor_gains_file ('%s'): %s",
      params.motor_gains_file.c_str(), e.what());
    return CallbackReturn::ERROR;
  }

  if (!cartesian_velocity_limit_enabled_) {
    RCLCPP_WARN(get_node()->get_logger(),
      "No cartesian_velocity_limit configured -- TCP speed will not be "
      "checked by the runtime velocity trip.");
  }
  for (size_t i = 0; i < joint_names_.size(); ++i) {
    const double limit = robot_model_->jointVelocityLimits()[static_cast<Eigen::Index>(i)];
    if (!std::isfinite(limit) || limit <= 0.0) {
      RCLCPP_WARN(get_node()->get_logger(),
        "Joint '%s' has no velocity limit in the URDF -- it will not be "
        "checked by the runtime velocity trip.", joint_names_[i].c_str());
    }
  }

  try {
    goal_limits_.emplace(
        joint_names_,
        robot_model_->jointLowerLimits(),
        robot_model_->jointUpperLimits(),
        params.joint_position_margin,
        params.cartesian_position_min,
        params.cartesian_position_max,
        joint_torque_limits_,
        motor_kp,
        motor_kd);
  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to construct GoalLimits: %s", e.what());
    return CallbackReturn::ERROR;
  }
  goal_limits_->logSummary(get_node()->get_logger());

  RCLCPP_INFO(get_node()->get_logger(),
    "OpenArm Impedance Controller: motor onboard PD tracks the interpolated "
    "trajectory directly.");

  param_callback_handle_ = get_node()->add_on_set_parameters_callback(
      std::bind(&OpenArmImpedanceController::onParameterChange,
                this, std::placeholders::_1));

  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn OpenArmImpedanceController::on_configure(
    const rclcpp_lifecycle::State& /* state */) {
  const std::string action_name =
      std::string("/") + get_node()->get_name() + "/follow_joint_trajectory";

  action_server_ = rclcpp_action::create_server<FollowJointTrajectoryAction>(
      get_node(),
      action_name,
      std::bind(&OpenArmImpedanceController::onGoalRequest,
                this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&OpenArmImpedanceController::onCancelRequest,
                this, std::placeholders::_1),
      std::bind(&OpenArmImpedanceController::onGoalAccepted,
                this, std::placeholders::_1));

  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn OpenArmImpedanceController::on_activate(
    const rclcpp_lifecycle::State& /* state */) {
  const size_t n = joint_names_.size();

  if (state_interfaces_.size() != 2 * n || command_interfaces_.size() != n * 3) {
    RCLCPP_ERROR(get_node()->get_logger(), "Interface count mismatch");
    return CallbackReturn::ERROR;
  }

  // Seed the reference to the current joint state, so the arm holds in place.
  q_ref_  = Eigen::VectorXd(n);
  dq_ref_ = Eigen::VectorXd::Zero(n);
  for (size_t i = 0; i < n; ++i) {
    q_ref_[i] = state_interfaces_[i * 2].get_value();
  }

  Eigen::VectorXd tau = do_gravity_compensation_
      ? robot_model_->gravityTorqueRT(q_ref_)
      : Eigen::VectorXd::Zero(n);
  clampAndReportEffort(tau);

  for (size_t i = 0; i < n; ++i) {
    if (!command_interfaces_[i * 3].set_value(q_ref_[i])) {
      reportInterfaceWriteFailure("position", i);
    }
    if (!command_interfaces_[i * 3 + 1].set_value(dq_ref_[i])) {
      reportInterfaceWriteFailure("velocity", i);
    }
    if (!command_interfaces_[i * 3 + 2].set_value(tau[i])) {
      reportInterfaceWriteFailure("effort", i);
    }
  }

  const Eigen::Vector3d tcp = goal_limits_->cartesianEnabled()
      ? robot_model_->tcpPositionNonRT(q_ref_)
      : Eigen::Vector3d::Zero();
  goal_limits_->reportViolations(q_ref_, tcp, get_node()->get_logger(),
                                *get_node()->get_clock(), kLimitWarnPeriodMs);

  trajectory_buffer_.writeFromNonRT(nullptr);
  feedback_msg_ = std::make_shared<FollowJointTrajectoryAction::Feedback>();
  {
    std::lock_guard<std::mutex> lock(goal_mutex_);
    active_goal_.reset();
  }
  feedback_count_ = 0;
  trajectory_start_time_ = get_node()->get_clock()->now();

  RCLCPP_INFO(get_node()->get_logger(), "OpenArm Impedance Controller activated");
  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn OpenArmImpedanceController::on_deactivate(
    const rclcpp_lifecycle::State& /* state */) {
  {
    std::lock_guard<std::mutex> lock(goal_mutex_);
    if (active_goal_) {
      auto result = std::make_shared<FollowJointTrajectoryAction::Result>();
      result->error_code = -1;
      result->error_string = "Controller deactivated";
      active_goal_->abort(result);
      active_goal_.reset();
    }
  }

  // Command the measured position, zero velocity, zero effort. 
  for (size_t i = 0; i < joint_names_.size(); ++i) {
    const double pos = state_interfaces_[i * 2].get_value();
    if (!command_interfaces_[i * 3].set_value(pos)) {
      reportInterfaceWriteFailure("position", i);
    }
    if (!command_interfaces_[i * 3 + 1].set_value(0.0)) {
      reportInterfaceWriteFailure("velocity", i);
    }
    if (!command_interfaces_[i * 3 + 2].set_value(0.0)) {
      reportInterfaceWriteFailure("effort", i);
    }
  }

  RCLCPP_INFO(get_node()->get_logger(), "OpenArm Impedance Controller deactivated");
  return CallbackReturn::SUCCESS;
}

// Hardware Interface Configuration

controller_interface::InterfaceConfiguration
OpenArmImpedanceController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto& name : joint_names_) {
    config.names.push_back(name + "/position");
    config.names.push_back(name + "/velocity");
    config.names.push_back(name + "/effort");
  }
  return config;
}

controller_interface::InterfaceConfiguration
OpenArmImpedanceController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto& name : joint_names_) {
    config.names.push_back(name + "/position");
    config.names.push_back(name + "/velocity");
  }
  return config;
}

// RT Control Loop

controller_interface::return_type OpenArmImpedanceController::update(
    const rclcpp::Time& time, const rclcpp::Duration& /* period */) {
  const size_t n = joint_names_.size();
  Eigen::VectorXd q(n);
  Eigen::VectorXd dq(n);
  for (size_t i = 0; i < n; ++i) {
    q[i]  = state_interfaces_[i * 2].get_value();
    dq[i] = state_interfaces_[i * 2 + 1].get_value();
  }

  if (!q.allFinite() || !dq.allFinite()) {
    RCLCPP_ERROR(get_node()->get_logger(),
      "Non-finite joint state (NaN/Inf) from a state interface. Commanding "
      "zero effort and stopping the controller.");
    for (size_t i = 0; i < n; ++i) {
      if (!command_interfaces_[i * 3 + 2].set_value(0.0)) {
        reportInterfaceWriteFailure("effort", i);
      }
    }
    {
      std::lock_guard<std::mutex> lock(goal_mutex_);
      if (active_goal_) {
        auto result = std::make_shared<FollowJointTrajectoryAction::Result>();
        result->error_code = -1;
        result->error_string = "Non-finite joint state";
        active_goal_->abort(result);
        active_goal_.reset();
      }
    }
    trajectory_buffer_.writeFromNonRT(nullptr);
    return controller_interface::return_type::ERROR;
  }

  auto trajectory_ptr = trajectory_buffer_.readFromRT();
  const bool trajectory_active = (trajectory_ptr && *trajectory_ptr);
  if (trajectory_active) {
    interpolateTrajectory(**trajectory_ptr, time);
  }

  Eigen::VectorXd tau = do_gravity_compensation_
      ? robot_model_->gravityTorqueRT(q)
      : Eigen::VectorXd::Zero(n);

  if (!tau.allFinite()) {
    RCLCPP_ERROR(get_node()->get_logger(),
      "Non-finite gravity-comp torque. Commanding zero effort and stopping "
      "the controller.");
    for (size_t i = 0; i < n; ++i) {
      if (!command_interfaces_[i * 3 + 2].set_value(0.0)) {
        reportInterfaceWriteFailure("effort", i);
      }
    }
    {
      std::lock_guard<std::mutex> lock(goal_mutex_);
      if (active_goal_) {
        auto result = std::make_shared<FollowJointTrajectoryAction::Result>();
        result->error_code = -1;
        result->error_string = "Non-finite torque command";
        active_goal_->abort(result);
        active_goal_.reset();
      }
    }
    trajectory_buffer_.writeFromNonRT(nullptr);
    return controller_interface::return_type::ERROR;
  }
  clampAndReportEffort(tau);

  const auto& J = robot_model_->jacobianRT(q);
  const Eigen::Vector3d tcp     = robot_model_->lastTcpPositionRT();
  const Eigen::Vector3d tcp_vel = (J * dq).head<3>();

  if (checkVelocityTrip(dq, tcp_vel)) {
    for (size_t i = 0; i < n; ++i) {
      if (!command_interfaces_[i * 3 + 2].set_value(0.0)) {
        reportInterfaceWriteFailure("effort", i);
      }
    }
    q_ref_  = q;
    dq_ref_ = Eigen::VectorXd::Zero(n);
    {
      std::lock_guard<std::mutex> lock(goal_mutex_);
      if (active_goal_) {
        auto result = std::make_shared<FollowJointTrajectoryAction::Result>();
        result->error_code = -2;
        result->error_string = "Velocity limit exceeded; goal aborted";
        active_goal_->abort(result);
        active_goal_.reset();
      }
    }
    trajectory_buffer_.writeFromNonRT(nullptr);
    return controller_interface::return_type::OK;
  }

  // Hand the interpolated reference straight to the motors' onboard PD.
  for (size_t i = 0; i < n; ++i) {
    if (!command_interfaces_[i * 3].set_value(q_ref_[i])) {
      reportInterfaceWriteFailure("position", i);
    }
    if (!command_interfaces_[i * 3 + 1].set_value(dq_ref_[i])) {
      reportInterfaceWriteFailure("velocity", i);
    }
    if (!command_interfaces_[i * 3 + 2].set_value(tau[i])) {
      reportInterfaceWriteFailure("effort", i);
    }
  }

  goal_limits_->reportViolations(q, tcp, get_node()->get_logger(),
                                *get_node()->get_clock(), kLimitWarnPeriodMs);

  std::shared_ptr<GoalHandle> goal;
  {
    std::lock_guard<std::mutex> lock(goal_mutex_);
    goal = active_goal_;
  }

  if (!trajectory_active || !goal) {
    return controller_interface::return_type::OK;
  }

  if (goal->is_canceling()) {
    auto result = std::make_shared<FollowJointTrajectoryAction::Result>();
    result->error_code = FollowJointTrajectoryAction::Result::SUCCESSFUL;
    goal->canceled(result);
    {
      std::lock_guard<std::mutex> lock(goal_mutex_);
      active_goal_ = nullptr;
    }
    trajectory_buffer_.writeFromNonRT(nullptr);
    return controller_interface::return_type::OK;
  }

  const auto& points = (*trajectory_ptr)->points;
  const rclcpp::Duration elapsed = time - trajectory_start_time_;
  if (rclcpp::Duration(points.back().time_from_start) <= elapsed) {
    auto result = std::make_shared<FollowJointTrajectoryAction::Result>();
    result->error_code = FollowJointTrajectoryAction::Result::SUCCESSFUL;
    goal->succeed(result);
    {
      std::lock_guard<std::mutex> lock(goal_mutex_);
      active_goal_ = nullptr;
    }
    trajectory_buffer_.writeFromNonRT(nullptr);
    return controller_interface::return_type::OK;
  }

  if (++feedback_count_ % kFeedbackStride != 0) {
    return controller_interface::return_type::OK;
  }

  updateFeedback(q, dq);
  goal->publish_feedback(feedback_msg_);

  return controller_interface::return_type::OK;
}

void OpenArmImpedanceController::clampAndReportEffort(Eigen::VectorXd& tau) {
  for (size_t i = 0; i < joint_names_.size(); ++i) {
    const auto idx = static_cast<Eigen::Index>(i);
    const double limit = joint_torque_limits_[i];
    if (tau[idx] > limit) {
      tau[idx] = limit;
      RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(),
        kLimitWarnPeriodMs,
        "Gravity-comp effort on '%s' clamped at +%.2f Nm. This should not "
        "happen if goal validation is working -- investigate.",
        joint_names_[i].c_str(), limit);
    } else if (tau[idx] < -limit) {
      tau[idx] = -limit;
      RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(),
        kLimitWarnPeriodMs,
        "Gravity-comp effort on '%s' clamped at -%.2f Nm. This should not "
        "happen if goal validation is working -- investigate.",
        joint_names_[i].c_str(), limit);
    }
  }
}

// Velocity trip. Allocation-free.

bool OpenArmImpedanceController::checkVelocityTrip(
    const Eigen::VectorXd& dq, const Eigen::Vector3d& tcp_vel) {
  const Eigen::VectorXd& dq_limit = robot_model_->jointVelocityLimits();
  for (size_t i = 0; i < joint_names_.size(); ++i) {
    const auto idx = static_cast<Eigen::Index>(i);
    if (!std::isfinite(dq_limit[idx]) || dq_limit[idx] <= 0.0) {
      continue;  // unspecified in the URDF, not checked
    }
    if (std::abs(dq[idx]) > dq_limit[idx]) {
      RCLCPP_ERROR_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(),
        kLimitWarnPeriodMs,
        "Velocity trip: joint '%s' at %.3f rad/s exceeds its URDF limit of "
        "%.3f rad/s. Freezing and aborting the active goal.",
        joint_names_[i].c_str(), dq[idx], dq_limit[idx]);
      return true;
    }
  }
  if (cartesian_velocity_limit_enabled_) {
    const double speed = tcp_vel.norm();
    if (speed > cartesian_velocity_limit_) {
      RCLCPP_ERROR_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(),
        kLimitWarnPeriodMs,
        "Velocity trip: TCP speed %.3f m/s exceeds cartesian_velocity_limit "
        "(%.3f m/s). Freezing and aborting the active goal.",
        speed, cartesian_velocity_limit_);
      return true;
    }
  }
  return false;
}

void OpenArmImpedanceController::reportInterfaceWriteFailure(
    const char* what, size_t joint_idx) {
  RCLCPP_ERROR_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(),
    kLimitWarnPeriodMs,
    "hardware_interface set_value() failed writing %s for joint '%s'.",
    what, joint_names_[joint_idx].c_str());
}

// Trajectory Helpers

void OpenArmImpedanceController::interpolateTrajectory(
    const JointTrajectory& traj, const rclcpp::Time& time) {
  const auto& points = traj.points;
  const rclcpp::Duration elapsed = time - trajectory_start_time_;
  const size_t n = joint_names_.size();

  size_t seg = 0;
  for (size_t i = 0; i < points.size(); ++i) {
    if (rclcpp::Duration(points[i].time_from_start) <= elapsed) {
      seg = i;
    } else {
      break;
    }
  }

  auto to_eigen = [n](const std::vector<double>& v) {
    return Eigen::Map<const Eigen::VectorXd>(v.data(), n);
  };

  if (rclcpp::Duration(points.back().time_from_start) <= elapsed) {
    q_ref_  = to_eigen(points.back().positions);
    dq_ref_ = Eigen::VectorXd::Zero(n);
  } else if (seg + 1 < points.size()) {
    const auto& p0 = points[seg];
    const auto& p1 = points[seg + 1];
    const double t0 = rclcpp::Duration(p0.time_from_start).seconds();
    const double t1 = rclcpp::Duration(p1.time_from_start).seconds();
    const double alpha = (elapsed.seconds() - t0) / (t1 - t0);

    q_ref_  = to_eigen(p0.positions)  * (1 - alpha) + to_eigen(p1.positions)  * alpha;
    dq_ref_ = to_eigen(p0.velocities) * (1 - alpha) + to_eigen(p1.velocities) * alpha;
  }
}

void OpenArmImpedanceController::updateFeedback(
    const Eigen::VectorXd& q, const Eigen::VectorXd& dq) {
  const size_t n = joint_names_.size();
  feedback_msg_->joint_names = joint_names_;

  feedback_msg_->desired.positions.assign(q_ref_.data(),  q_ref_.data()  + n);
  feedback_msg_->desired.velocities.assign(dq_ref_.data(), dq_ref_.data() + n);

  feedback_msg_->actual.positions.assign(q.data(),  q.data()  + n);
  feedback_msg_->actual.velocities.assign(dq.data(), dq.data() + n);

  const Eigen::VectorXd qe  = q_ref_  - q;
  const Eigen::VectorXd dqe = dq_ref_ - dq;
  feedback_msg_->error.positions.assign(qe.data(),  qe.data()  + n);
  feedback_msg_->error.velocities.assign(dqe.data(), dqe.data() + n);
}

std::vector<size_t> OpenArmImpedanceController::buildJointMap(
    const std::vector<std::string>& goal_names) const {
  const size_t n = joint_names_.size();
  std::vector<size_t> map(n);
  for (size_t k = 0; k < n; ++k) {
    const auto it = std::find(goal_names.begin(), goal_names.end(), joint_names_[k]);
    map[k] = static_cast<size_t>(std::distance(goal_names.begin(), it));
  }
  return map;
}

// Action Server Callbacks

rclcpp_action::GoalResponse OpenArmImpedanceController::onGoalRequest(
    const rclcpp_action::GoalUUID& /* uuid */,
    std::shared_ptr<const FollowJointTrajectoryAction::Goal> goal) {
  if (goal->trajectory.points.empty()) {
    RCLCPP_WARN(get_node()->get_logger(), "Rejected empty trajectory");
    return rclcpp_action::GoalResponse::REJECT;
  }
  if (goal->trajectory.joint_names.size() != joint_names_.size()) {
    RCLCPP_WARN(get_node()->get_logger(), "Rejected: joint count mismatch");
    return rclcpp_action::GoalResponse::REJECT;
  }

  const auto& goal_names = goal->trajectory.joint_names;
  for (const auto& name : joint_names_) {
    if (std::find(goal_names.begin(), goal_names.end(), name) == goal_names.end()) {
      RCLCPP_WARN(get_node()->get_logger(),
                  "Rejected: joint '%s' not found in goal trajectory", name.c_str());
      return rclcpp_action::GoalResponse::REJECT;
    }
  }

  const size_t n = joint_names_.size();
  for (const auto& pt : goal->trajectory.points) {
    if (pt.positions.size() != n) {
      RCLCPP_WARN(get_node()->get_logger(),
                  "Rejected: trajectory point has %zu positions, expected %zu",
                  pt.positions.size(), n);
      return rclcpp_action::GoalResponse::REJECT;
    }
    if (!pt.velocities.empty() && pt.velocities.size() != n) {
      RCLCPP_WARN(get_node()->get_logger(), "Rejected: velocity size mismatch");
      return rclcpp_action::GoalResponse::REJECT;
    }
  }

  // Position limits (joint-space and Cartesian) plus predicted torque.
  if (!goal_limits_->validate(goal->trajectory, buildJointMap(goal_names),
                              q_ref_, dq_ref_, *robot_model_, get_node()->get_logger())) {
    return rclcpp_action::GoalResponse::REJECT;
  }

  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse OpenArmImpedanceController::onCancelRequest(
    std::shared_ptr<GoalHandle> /* goal_handle */) {
  return rclcpp_action::CancelResponse::ACCEPT;
}

void OpenArmImpedanceController::onGoalAccepted(std::shared_ptr<GoalHandle> goal_handle) {
  const size_t n = joint_names_.size();

  auto traj = std::make_shared<JointTrajectory>(goal_handle->get_goal()->trajectory);
  const auto& goal_names = traj->joint_names;

  const std::vector<size_t> map = buildJointMap(goal_names);

  for (auto& pt : traj->points) {
    std::vector<double> p(n), v(n, 0.0);
    for (size_t k = 0; k < n; ++k) {
      p[k] = pt.positions[map[k]];
      if (!pt.velocities.empty()) v[k] = pt.velocities[map[k]];
    }
    pt.positions = p;
    pt.velocities = v;
  }
  traj->joint_names = joint_names_;

  std::lock_guard<std::mutex> lock(goal_mutex_);
  if (active_goal_) {
    auto result = std::make_shared<FollowJointTrajectoryAction::Result>();
    result->error_code = FollowJointTrajectoryAction::Result::INVALID_GOAL;
    result->error_string = "Preempted by new goal";
    active_goal_->abort(result);
  }

  // Prepend the current reference as the start point at t=0.
  trajectory_msgs::msg::JointTrajectoryPoint start_pt;
  start_pt.positions.assign(q_ref_.data(), q_ref_.data() + n);
  start_pt.velocities.assign(n, 0.0);
  start_pt.time_from_start = rclcpp::Duration::from_seconds(0.0);
  traj->points.insert(traj->points.begin(), start_pt);

  trajectory_start_time_ = get_node()->get_clock()->now();
  active_goal_ = goal_handle;
  trajectory_buffer_.writeFromNonRT(traj);
}

// Parameter Callbacks

rcl_interfaces::msg::SetParametersResult
OpenArmImpedanceController::onParameterChange(const std::vector<rclcpp::Parameter>& params) {
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  for (const auto& p : params) {
    const std::string& name = p.get_name();
    if (name == "do_gravity_compensation") {
      do_gravity_compensation_ = p.as_bool();
    } else if (name == "joint_torque_limits" ||
               name == "cartesian_position_min" ||
               name == "cartesian_position_max" ||
               name == "joint_position_margin" ||
               name == "cartesian_velocity_limit" ||
               name == "motor_gains_file") {
      result.successful = false;
      result.reason = name + " is fixed at configure time; edit the YAML and relaunch";
    }
  }
  return result;
}

}  // namespace openarm_impedance_controller

PLUGINLIB_EXPORT_CLASS(
  openarm_impedance_controller::OpenArmImpedanceController,
  controller_interface::ControllerInterface)