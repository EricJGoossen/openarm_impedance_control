#include "openarm_impedance_control/openarm_impedance_controller.hpp"

#include "pluginlib/class_list_macros.hpp"

namespace openarm_impedance_control {

// Lifecycle

controller_interface::CallbackReturn OpenArmImpedanceController::on_init() {
  auto_declare<std::vector<std::string>>("joints", std::vector<std::string>{});
  auto_declare<std::string>("robot_description", "");
  auto_declare<std::vector<double>>("joint_torque_limits", std::vector<double>(7, INFINITY));
  auto_declare<std::vector<double>>("k_gains", std::vector<double>(6, 0.0));
  auto_declare<std::vector<double>>("d_gains", std::vector<double>(6, 0.0));
  auto_declare<std::string>("ee_frame_name", "tool0");

  joint_names_ = get_node()->get_parameter("joints").as_string_array();
  const auto joint_torque_limits =
      get_node()->get_parameter("joint_torque_limits").as_double_array();

  const std::string urdf_string =
      get_node()->get_parameter("robot_description").as_string();
  if (urdf_string.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(), "robot_description parameter is empty");
    return CallbackReturn::ERROR;
  }

  try {
    impedance_controller_.emplace(
      urdf_string,
      joint_names_,
      get_node()->get_parameter("ee_frame_name").as_string(),
      joint_torque_limits);
  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_node()->get_logger(),
      "Failed to construct Impedance: %s", e.what());
    return CallbackReturn::ERROR;
  }

  impedance_controller_->setStiffness(
      get_node()->get_parameter("k_gains").as_double_array());
  impedance_controller_->setDamping(
      get_node()->get_parameter("d_gains").as_double_array());

  param_callback_handle_ = get_node()->add_on_set_parameters_callback(
      std::bind(&OpenArmImpedanceController::onParameterChange,
                this, std::placeholders::_1));

  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn OpenArmImpedanceController::on_activate(
    const rclcpp_lifecycle::State& /* state */) {
      
  if (state_interfaces_.size() != 2 * n || command_interfaces_.size() != n) {
    RCLCPP_ERROR(get_node()->get_logger(), "Interface count mismatch");
    return CallbackReturn::ERROR;
  }

  for (auto& cmd : command_interfaces_) {
    (void)cmd.set_value(0.0);
  }

  trajectory_buffer_.writeFromNonRT(nullptr);
  feedback_msg_ = std::make_shared<FollowJointTrajectoryAction::Feedback>();
  {
    std::lock_guard<std::mutex> lock(goal_mutex_);
    active_goal_.reset();
  }
  feedback_count_ = 0;

  // Seed reference to current joint state so the arm holds in place
  const size_t n = joint_names_.size();
  q_ref_  = Eigen::VectorXd(n);
  dq_ref_ = Eigen::VectorXd::Zero(n);
  for (size_t i = 0; i < n; ++i) {
    q_ref_[i] = state_interfaces_[i * 2].get_value();
  }

  trajectory_start_time_ = get_node()->get_clock()->now();

  RCLCPP_INFO(get_node()->get_logger(), "OpenArm Impedance Controller activated");
  return controller_interface::CallbackReturn::SUCCESS;
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
  for (auto& cmd : command_interfaces_) {
    (void)cmd.set_value(0.0);
  }
  RCLCPP_INFO(get_node()->get_logger(), "OpenArm Impedance Controller deactivated");
  return controller_interface::CallbackReturn::SUCCESS;
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

// Hardware Interface Configuration

controller_interface::InterfaceConfiguration
OpenArmImpedanceController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto& name : joint_names_) {
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

  auto trajectory_ptr = trajectory_buffer_.readFromRT();
  const bool trajectory_active = (trajectory_ptr && *trajectory_ptr);
  if (trajectory_active) {
    interpolateTrajectory(**trajectory_ptr, time);
  }

  const Eigen::VectorXd torques =
      impedance_controller_->computeControl(q, dq, q_ref_, dq_ref_);
  for (size_t i = 0; i < n; ++i) {
    (void)command_interfaces_[i].set_value(torques[i]);
  }

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
    Eigen::VectorXd out = Eigen::VectorXd::Zero(n);
    for (size_t i = 0; i < n && i < v.size(); ++i) out[i] = v[i];
    return out;
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

// Action Server Callbacks

rclcpp_action::GoalResponse OpenArmImpedanceController::onGoalRequest(
    const rclcpp_action::GoalUUID& /* uuid */,
    std::shared_ptr<const FollowJointTrajectoryAction::Goal> goal) {
  if (goal->trajectory.points.empty()) {
    RCLCPP_WARN(get_node()->get_logger(), "Rejected empty trajectory");
    return rclcpp_action::GoalResponse::REJECT;
  }
  // Reject if joint set doesn't match (order-insensitive check)
  if (goal->trajectory.joint_names.size() != joint_names_.size()) {
    RCLCPP_WARN(get_node()->get_logger(), "Rejected: joint count mismatch");
    return rclcpp_action::GoalResponse::REJECT;
  }
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse OpenArmImpedanceController::onCancelRequest(
    std::shared_ptr<GoalHandle> /* goal_handle */) {
  return rclcpp_action::CancelResponse::ACCEPT;
}

void OpenArmImpedanceController::onGoalAccepted(std::shared_ptr<GoalHandle> goal_handle) {
  std::lock_guard<std::mutex> lock(goal_mutex_);
  if (active_goal_) {
    auto result = std::make_shared<FollowJointTrajectoryAction::Result>();
    result->error_code = FollowJointTrajectoryAction::Result::INVALID_GOAL;
    result->error_string = "Preempted by new goal";
    active_goal_->abort(result);
  }
  trajectory_start_time_ = get_node()->get_clock()->now();
  active_goal_ = goal_handle;

  auto traj = std::make_shared<JointTrajectory>(goal_handle->get_goal()->trajectory);

  const auto& goal_names = goal_handle->get_goal()->trajectory.joint_names;
  std::vector<size_t> map(goal_names.size()); 
  for (size_t k = 0; k < goal_names.size(); ++k) {
    auto it = std::find(goal_names.begin(), goal_names.end(), joint_names_[k]);
    if (it == goal_names.end()) { /* abort goal, return */ }
    map[k] = std::distance(goal_names.begin(), it);
  }
  // reorder each point's positions/velocities into joint_names_ order
  for (auto& pt : traj->points) {
    std::vector<double> p(n), v(n);
    for (size_t k = 0; k < n; ++k) {
      p[k] = pt.positions[map[k]];
      if (!pt.velocities.empty()) v[k] = pt.velocities[map[k]];
    }
    pt.positions = p;
    if (!pt.velocities.empty()) pt.velocities = v;
  }

  // Prepend current reference as the start point at t=0
  const size_t n = joint_names_.size();
  trajectory_msgs::msg::JointTrajectoryPoint start_pt;
  start_pt.positions.assign(q_ref_.data(), q_ref_.data() + n);
  start_pt.velocities.assign(n, 0.0);
  start_pt.time_from_start = rclcpp::Duration::from_seconds(0.0);
  traj->points.insert(traj->points.begin(), start_pt);

  trajectory_buffer_.writeFromNonRT(traj);
}

// Parameter Callbacks

rcl_interfaces::msg::SetParametersResult
OpenArmImpedanceController::onParameterChange(const std::vector<rclcpp::Parameter>& params) {
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  for (const auto& p : params) {
    if (p.get_name() == "k_gains") {
      impedance_controller_->setStiffness(p.as_double_array());
    } else if (p.get_name() == "d_gains") {
      impedance_controller_->setDamping(p.as_double_array());
    }
  }
  return result;
}

}  // namespace openarm_impedance_control

PLUGINLIB_EXPORT_CLASS(
  openarm_impedance_control::OpenArmImpedanceController,
  controller_interface::ControllerInterface)