#include "openarm_impedance_control/goal_limits.hpp"
#include "openarm_impedance_control/robot_model.hpp"

#include <cmath>
#include <stdexcept>

namespace openarm_impedance_controller {

namespace {
constexpr const char* kAxisName[3] = {"x", "y", "z"};
}  // namespace

GoalLimits::GoalLimits(std::vector<std::string> joint_names,
                       Eigen::VectorXd q_lower, Eigen::VectorXd q_upper,
                       double joint_position_margin,
                       std::vector<double> cartesian_min,
                       std::vector<double> cartesian_max,
                       std::vector<double> joint_torque_limits,
                       std::vector<double> motor_kp,
                       std::vector<double> motor_kd)
  : joint_names_(std::move(joint_names)),
    q_lower_(std::move(q_lower)),
    q_upper_(std::move(q_upper)),
    joint_position_margin_(joint_position_margin),
    cartesian_min_(std::move(cartesian_min)),
    cartesian_max_(std::move(cartesian_max)),
    joint_torque_limits_(std::move(joint_torque_limits)),
    motor_kp_(std::move(motor_kp)),
    motor_kd_(std::move(motor_kd))
{
  if (!std::isfinite(joint_position_margin_) || joint_position_margin_ < 0.0) {
    throw std::invalid_argument("joint_position_margin must be finite and non-negative");
  }
  for (size_t i = 0; i < joint_names_.size(); ++i) {
    const auto idx = static_cast<Eigen::Index>(i);
    if (!std::isfinite(q_lower_[idx]) || !std::isfinite(q_upper_[idx])) {
      continue;
    }
    if (q_upper_[idx] - q_lower_[idx] <= 2.0 * joint_position_margin_) {
      throw std::invalid_argument(
          "joint_position_margin (" + std::to_string(joint_position_margin_) +
          " rad) leaves no admissible range on '" + joint_names_[i] + "'");
    }
  }

  if (cartesian_min_.empty() != cartesian_max_.empty()) {
    throw std::invalid_argument(
        "cartesian_position_min and cartesian_position_max must both be set or both empty");
  }
  if (!cartesian_min_.empty()) {
    if (cartesian_min_.size() != 3 || cartesian_max_.size() != 3) {
      throw std::invalid_argument(
          "cartesian_position_min/max must have exactly 3 elements [x y z]");
    }
    for (size_t a = 0; a < 3; ++a) {
      if (!std::isfinite(cartesian_min_[a]) || !std::isfinite(cartesian_max_[a]) ||
          cartesian_min_[a] >= cartesian_max_[a]) {
        throw std::invalid_argument(
            std::string("cartesian_position limits on ") + kAxisName[a] +
            " are not a valid interval");
      }
    }
    cartesian_enabled_ = true;
  }

  const size_t n = joint_names_.size();
  if (joint_torque_limits_.size() != n || motor_kp_.size() != n || motor_kd_.size() != n) {
    throw std::invalid_argument(
        "joint_torque_limits / motor kp / motor kd must each have " +
        std::to_string(n) + " elements (one per joint)");
  }
  for (size_t i = 0; i < n; ++i) {
    if (!std::isfinite(joint_torque_limits_[i]) || joint_torque_limits_[i] <= 0.0) {
      throw std::invalid_argument("joint_torque_limits must be finite and positive");
    }
    if (!std::isfinite(motor_kp_[i]) || motor_kp_[i] < 0.0 ||
        !std::isfinite(motor_kd_[i]) || motor_kd_[i] < 0.0) {
      throw std::invalid_argument("motor kp/kd must be finite and non-negative");
    }
  }
}

bool GoalLimits::checkSegmentTorque(const Eigen::VectorXd& q0, const Eigen::VectorXd& dq0,
                                    const Eigen::VectorXd& q1, const Eigen::VectorXd& dq1,
                                    size_t point_index, const RobotModel& model,
                                    const rclcpp::Logger& logger) const {
  const Eigen::VectorXd g0 = model.gravityTorqueNonRT(q0);
  const Eigen::VectorXd g1 = model.gravityTorqueNonRT(q1);

  for (size_t k = 0; k < joint_names_.size(); ++k) {
    const auto idx = static_cast<Eigen::Index>(k);
    const double pos_err = std::abs(q1[idx] - q0[idx]);
    const double vel     = std::max(std::abs(dq0[idx]), std::abs(dq1[idx]));
    const double tau_pd  = motor_kp_[k] * pos_err + motor_kd_[k] * vel;
    const double tau_g   = std::max(std::abs(g0[idx]), std::abs(g1[idx]));
    const double tau_est = tau_pd + tau_g;

    if (tau_est > joint_torque_limits_[k]) {
      RCLCPP_WARN(logger,
        "Rejected: waypoint %zu estimated torque on joint '%s' is %.2f Nm "
        "(onboard PD ~%.2f Nm [kp=%.1f * %.4f rad + kd=%.2f * %.3f rad/s] + "
        "gravity ~%.2f Nm), exceeding joint_torque_limits of %.2f Nm. Slow "
        "down or shorten this segment.",
        point_index, joint_names_[k].c_str(), tau_est, tau_pd,
        motor_kp_[k], pos_err, motor_kd_[k], vel, tau_g, joint_torque_limits_[k]);
      return false;
    }
  }
  return true;
}

bool GoalLimits::validate(const trajectory_msgs::msg::JointTrajectory& traj,
                         const std::vector<size_t>& joint_map,
                         const Eigen::VectorXd& q_ref_start,
                         const Eigen::VectorXd& dq_ref_start,
                         const RobotModel& model,
                         const rclcpp::Logger& logger) const {
  const size_t n = joint_names_.size();
  Eigen::VectorXd q_pt(n), dq_pt(n);
  Eigen::VectorXd prev_q = q_ref_start;
  Eigen::VectorXd prev_dq = dq_ref_start;

  for (size_t p = 0; p < traj.points.size(); ++p) {
    const auto& pt = traj.points[p];
    for (size_t k = 0; k < n; ++k) {
      const auto idx = static_cast<Eigen::Index>(k);
      q_pt[idx]  = pt.positions[joint_map[k]];
      dq_pt[idx] = pt.velocities.empty() ? 0.0 : pt.velocities[joint_map[k]];
    }

    // Joint-space box (URDF limits, shrunk by the margin).
    for (size_t k = 0; k < n; ++k) {
      const auto idx = static_cast<Eigen::Index>(k);
      if (!std::isfinite(q_lower_[idx]) || !std::isfinite(q_upper_[idx])) {
        continue;
      }
      const double lo = q_lower_[idx] + joint_position_margin_;
      const double hi = q_upper_[idx] - joint_position_margin_;
      if (!std::isfinite(q_pt[idx]) || q_pt[idx] < lo || q_pt[idx] > hi) {
        RCLCPP_WARN(logger,
          "Rejected: waypoint %zu puts joint '%s' at %.4f rad, outside the "
          "admissible range [%.4f, %.4f] (URDF limit [%.4f, %.4f], margin %.3f).",
          p, joint_names_[k].c_str(), q_pt[idx], lo, hi,
          q_lower_[idx], q_upper_[idx], joint_position_margin_);
        return false;
      }
    }

    // Cartesian workspace box on the TCP.
    if (cartesian_enabled_) {
      Eigen::Vector3d tcp;
      try {
        tcp = model.tcpPositionNonRT(q_pt);
      } catch (const std::exception& e) {
        RCLCPP_WARN(logger, "Rejected: forward kinematics failed on waypoint %zu: %s",
                    p, e.what());
        return false;
      }
      for (size_t a = 0; a < 3; ++a) {
        const auto ai = static_cast<Eigen::Index>(a);
        if (tcp[ai] < cartesian_min_[a] || tcp[ai] > cartesian_max_[a]) {
          RCLCPP_WARN(logger,
            "Rejected: waypoint %zu puts the TCP at %s = %.4f m, outside the "
            "workspace box [%.4f, %.4f]. (TCP would be at [%.4f, %.4f, %.4f].)",
            p, kAxisName[a], tcp[ai], cartesian_min_[a], cartesian_max_[a],
            tcp[0], tcp[1], tcp[2]);
          return false;
        }
      }
    }

    // Predicted onboard-PD + gravity torque for this segment.
    if (!checkSegmentTorque(prev_q, prev_dq, q_pt, dq_pt, p, model, logger)) {
      return false;
    }

    prev_q = q_pt;
    prev_dq = dq_pt;
  }
  return true;
}

void GoalLimits::reportViolations(const Eigen::VectorXd& q, const Eigen::Vector3d& tcp,
                                 const rclcpp::Logger& logger, rclcpp::Clock& clock,
                                 int throttle_ms) const {
  for (size_t i = 0; i < joint_names_.size(); ++i) {
    const auto idx = static_cast<Eigen::Index>(i);
    if (!std::isfinite(q_lower_[idx]) || !std::isfinite(q_upper_[idx])) {
      continue;
    }
    if (q[idx] < q_lower_[idx] || q[idx] > q_upper_[idx]) {
      RCLCPP_WARN_THROTTLE(logger, clock, throttle_ms,
        "Joint '%s' outside its URDF limit: %.4f not in [%.4f, %.4f]. Not "
        "corrected here -- the limit is enforced at goal-acceptance time.",
        joint_names_[i].c_str(), q[idx], q_lower_[idx], q_upper_[idx]);
    }
  }

  if (!cartesian_enabled_) {
    return;
  }
  for (size_t a = 0; a < 3; ++a) {
    const auto ai = static_cast<Eigen::Index>(a);
    if (tcp[ai] < cartesian_min_[a] || tcp[ai] > cartesian_max_[a]) {
      RCLCPP_WARN_THROTTLE(logger, clock, throttle_ms,
        "TCP outside the workspace box on %s: %.4f not in [%.4f, %.4f].",
        kAxisName[a], tcp[ai], cartesian_min_[a], cartesian_max_[a]);
    }
  }
}

void GoalLimits::logSummary(const rclcpp::Logger& logger) const {
  for (size_t i = 0; i < joint_names_.size(); ++i) {
    const auto idx = static_cast<Eigen::Index>(i);
    if (!std::isfinite(q_lower_[idx]) || !std::isfinite(q_upper_[idx])) {
      RCLCPP_WARN(logger,
        "Joint '%s' has no finite limit in the URDF -- goals will not be "
        "position-checked on it.", joint_names_[i].c_str());
    }
  }
  if (!cartesian_enabled_) {
    RCLCPP_WARN(logger,
      "No cartesian_position_min/max configured -- goals will not be "
      "position-checked in Cartesian space.");
  }
  RCLCPP_INFO(logger,
    "Torque-limit check active: onboard motor gains (kp/kd) plus gravity "
    "comp, checked per segment against joint_torque_limits.");
}

}  // namespace openarm_impedance_controller