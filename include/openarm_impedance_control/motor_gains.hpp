#pragma once

#include <map>
#include <string>
#include <vector>

namespace openarm_impedance_controller {

class MotorGains {
 public:
  // Throws std::runtime_error if the file can't be read or parsed.
  explicit MotorGains(const std::string& path);

  // Finds the entry whose key is a suffix of `joint_name`. 
  void lookup(const std::string& joint_name, double& kp, double& kd) const;

 private:
  struct Gain { double kp; double kd; };
  std::map<std::string, Gain> gains_;
};

}  // namespace openarm_impedance_controller