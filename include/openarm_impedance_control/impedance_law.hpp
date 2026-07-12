#pragma once

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

 private:
  using Vector6d = Eigen::Matrix<double, 6, 1>;
  using Matrix6d = Eigen::Matrix<double, 6, 6>;

  // Limits
  const std::vector<double> torque_limits_;

  // Control flags
  bool do_gravity_compensation_;
  ComplianceFrame compliance_frame_{ComplianceFrame::kWorld};

  // Pinocchio
  pinocchio::Model      pinocchio_model_;
  pinocchio::Data       pinocchio_data_;
  pinocchio::FrameIndex ee_frame_id_;

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
  void clamp(Eigen::Ref<Eigen::VectorXd> v, const std::vector<double>& limits,
             const std::string& context);
};

}  // namespace openarm_impedance_controller