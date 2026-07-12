#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Geometry"
#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/multibody/model.hpp"
#include "pinocchio/multibody/data.hpp"

namespace openarm_impedance_controller {

enum class ComplianceFrame { kWorld, kTcp };

class ImpedanceLaw {
 public:
  ImpedanceLaw(
      const std::string& urdf_string,
      const std::vector<std::string>& joint_names,
      const std::string& ee_frame_name,
      std::vector<double> torque_limits,
      std::vector<double> cartesian_wrench_limits,
      bool do_gravity_compensation = true);

  // Absolute Cartesian impedance, [x y z rx ry rz].
  void setCartesianStiffness(const std::vector<double>& k_cart);
  void setCartesianDamping(const std::vector<double>& d_cart);

  // Per-axis scale in [0, 1]. 
  void setStiffnessScale(const std::vector<double>& kappa_k);
  void setDampingScale(const std::vector<double>& kappa_d);

  // Null-space joint gains (size = nv). 
  void setNullspaceStiffness(const std::vector<double>& k_null);
  void setNullspaceDamping(const std::vector<double>& d_null);

  void setComplianceFrame(ComplianceFrame frame) { compliance_frame_ = frame; }
  void setGravityCompensation(bool on) { do_gravity_compensation_ = on; }

  Eigen::VectorXd computeControl(
      const Eigen::VectorXd& q,
      const Eigen::VectorXd& dq,
      const Eigen::VectorXd& q_ref,
      const Eigen::VectorXd& dq_ref);

  int nv() const { return pinocchio_model_.nv; }
  int nq() const { return pinocchio_model_.nq; }

  // Joint position limits as declared in the URDF, in `joints` param order.
  // Continuous joints report +/-inf, so bounds checks on them pass vacuously.
  const Eigen::VectorXd& jointLowerLimits() const { return q_lower_; }
  const Eigen::VectorXd& jointUpperLimits() const { return q_upper_; }

  // FK for validating goals off the control thread. 
  Eigen::Vector3d tcpPosition(const Eigen::VectorXd& q) const;

  std::uint32_t torqueClampMask() const { return torque_clamp_mask_; }
  std::uint32_t wrenchClampMask() const { return wrench_clamp_mask_; }

  // the TCP for the q just passed in.
  Eigen::Vector3d lastTcpPosition() const {
    return pinocchio_data_.oMf[ee_frame_id_].translation();
  }

 private:
  using Vector6d = Eigen::Matrix<double, 6, 1>;
  using Matrix6d = Eigen::Matrix<double, 6, 6>;

  // Limits 
  const std::vector<double> torque_limits_;             // size nv, Nm
  const std::vector<double> cartesian_wrench_limits_;   // size 6, N / Nm
  Eigen::VectorXd q_lower_;   // joint position limits, from the URDF
  Eigen::VectorXd q_upper_;

  // Control flags
  bool do_gravity_compensation_;
  ComplianceFrame compliance_frame_{ComplianceFrame::kWorld};

  // Pinocchio
  pinocchio::Model        pinocchio_model_;
  pinocchio::Data         pinocchio_data_;    // RT thread
  mutable pinocchio::Data validation_data_;   // action / service threads
  pinocchio::FrameIndex   ee_frame_id_;

  // Damping factor for the pseudo-inverse used by the null-space projector.
  constexpr static double kLambda  = 0.1;
  constexpr static double kLambda2 = kLambda * kLambda;

  // Gains
  Vector6d k_cart_{Vector6d::Zero()};
  Vector6d d_cart_{Vector6d::Zero()};
  Vector6d kappa_k_{Vector6d::Ones()};
  Vector6d kappa_d_{Vector6d::Ones()};
  Eigen::VectorXd k_null_;
  Eigen::VectorXd d_null_;

  // Clamp reporting: written by computeControl, read by the controller.
  std::uint32_t torque_clamp_mask_{0};
  std::uint32_t wrench_clamp_mask_{0};

  // Scratch (preallocated: this all runs in the RT loop)
  Eigen::Matrix<double, 6, Eigen::Dynamic> J_;
  Eigen::MatrixXd J_pinv_;
  Eigen::MatrixXd N_;
  Eigen::VectorXd tau_;
  Eigen::VectorXd q_err_;
  Eigen::VectorXd dq_err_;

  // Helpers
  static Vector6d checkSix(const std::vector<double>& v, const char* label);
  Eigen::VectorXd checkNv(const std::vector<double>& v, const char* label) const;

  // Clamps v into +/- limits elementwise, returns a bitmask of the indices
  // that were clamped. Allocation- and I/O-free, so safe in the RT loop.
  static std::uint32_t clamp(Eigen::Ref<Eigen::VectorXd> v,
                             const std::vector<double>& limits);
};

}  // namespace openarm_impedance_controller