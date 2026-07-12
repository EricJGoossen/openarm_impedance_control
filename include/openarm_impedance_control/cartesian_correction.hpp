#pragma once

#include <string>
#include <vector>

#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Geometry"
#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/multibody/model.hpp"
#include "pinocchio/multibody/data.hpp"

namespace openarm_impedance_control {

class CartesianCorrection {
 public:
  CartesianCorrection(
      const std::string& urdf_string,
      const std::vector<std::string>& joint_names,
      const std::string& ee_frame_name,
      std::vector<double> torque_limits,
      std::vector<double> k_joint_gains,
      std::vector<double> d_joint_gains,
      bool do_gravity_compensation = true);

  void setStiffness(const std::vector<double>& k_gains);
  void setDamping(const std::vector<double>& d_gains);

  Eigen::VectorXd computeControl(
      const Eigen::VectorXd& q,
      const Eigen::VectorXd& dq,
      const Eigen::VectorXd& q_ref,
      const Eigen::VectorXd& dq_ref);
    
 private:
  // Limits
  const std::vector<double> torque_limits_;

  // Control flag
  bool do_gravity_compensation_;

  // Pinocchio
  pinocchio::Model      pinocchio_model_;
  pinocchio::Data       pinocchio_data_;
  pinocchio::FrameIndex ee_frame_id_;

  // Impedance gains
  Eigen::Matrix<double, 6, 6> K_joint_;
  Eigen::Matrix<double, 6, 6> D_joint_;
  Eigen::Matrix<double, 6, 6> relax_K_; // Cartesian stiffness gain
  Eigen::Matrix<double, 6, 6> relax_D_; // Cartesian damping gain
  constexpr static double kEpsilon = 1.0;

  // Helper functions
  void applyRelaxGains(const std::vector<double>& gains, const char* label, Eigen::Matrix<double, 6, 6>& relax);
  void clamp(Eigen::Ref<Eigen::VectorXd> v, double lower_limit, double upper_limit, const std::string& context);
  void clamp(Eigen::Ref<Eigen::VectorXd> v, const std::vector<double>& limits, const std::string& context);

};

} // namespace openarm_cartesian_control