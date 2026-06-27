#pragma once

#include <string>
#include <vector>

#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Geometry"
#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/multibody/model.hpp"
#include "pinocchio/multibody/data.hpp"

namespace openarm_impedance_control {

class Impedance {
 public:
  Impedance(
      const std::string& urdf_string,
      const std::vector<std::string>& joint_names,
      const std::string& ee_frame_name,
      const std::vector<double>& torque_limits,
      bool do_gravity_compensation = true,
      bool do_damping = true,
      bool do_stiffness = true);

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

  // Control flags
  bool do_gravity_compensation_;
  bool do_damping_;
  bool do_stiffness_;

  // Pinocchio
  pinocchio::Model      pinocchio_model_;
  pinocchio::Data       pinocchio_data_;
  pinocchio::FrameIndex ee_frame_id_;

  // Impedance gains
  Eigen::Matrix<double, 6, 6> K_;
  Eigen::Matrix<double, 6, 6> D_;
  constexpr static double kEpsilon = 1.0;

  // Helper functions
  void clamp(Eigen::Ref<Eigen::VectorXd> v, const std::vector<double>& limits);

};

} // namespace openarm_cartesian_control