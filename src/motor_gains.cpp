#include "openarm_impedance_control/motor_gains.hpp"

#include <stdexcept>

#include "yaml-cpp/yaml.h"

namespace openarm_impedance_controller {

MotorGains::MotorGains(const std::string& path) {
  YAML::Node root;
  try {
    root = YAML::LoadFile(path);
  } catch (const std::exception& e) {
    throw std::runtime_error("MotorGains: failed to load '" + path + "': " + e.what());
  }

  if (!root.IsMap()) {
    throw std::runtime_error("MotorGains: '" + path + "' is not a YAML map at the top level");
  }

  for (const auto& entry : root) {
    const std::string key = entry.first.as<std::string>();
    const YAML::Node& node = entry.second;
    if (!node["kp"] || !node["kd"]) {
      throw std::runtime_error("MotorGains: entry '" + key + "' in '" + path +
                               "' is missing kp or kd");
    }
    Gain g;
    g.kp = node["kp"].as<double>();
    g.kd = node["kd"].as<double>();
    if (!(g.kp >= 0.0) || !(g.kd >= 0.0)) {
      throw std::runtime_error("MotorGains: entry '" + key + "' has a negative or NaN gain");
    }
    gains_[key] = g;
  }

  if (gains_.empty()) {
    throw std::runtime_error("MotorGains: '" + path + "' contained no gain entries");
  }
}

void MotorGains::lookup(const std::string& joint_name, double& kp, double& kd) const {
  const std::string* best_key = nullptr;
  for (const auto& [key, gain] : gains_) {
    if (joint_name.size() >= key.size() &&
        joint_name.compare(joint_name.size() - key.size(), key.size(), key) == 0) {
      if (!best_key || key.size() > best_key->size()) {
        best_key = &key;
      }
    }
  }
  if (!best_key) {
    throw std::runtime_error("MotorGains: no gain entry matches joint '" + joint_name + "'");
  }
  const Gain& g = gains_.at(*best_key);
  kp = g.kp;
  kd = g.kd;
}

}  // namespace openarm_impedance_controller