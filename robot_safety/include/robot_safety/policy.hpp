#pragma once
#include "robot_safety/domain.hpp"
#include <map>
#include <string>
#include <vector>
namespace robot_safety {
struct Policy {
  std::string version;
  std::map<std::string, Domain::Axis> axes;
  std::vector<Domain::Association> associations;
  std::map<std::string, std::vector<std::string>> controllers;
  static Policy load(const std::string & path);
};
// Configures once per process/domain. A second owner must supply identical configuration.
// Conflicting policies and mixed legacy/grouped writers are rejected before motion.
std::shared_ptr<Domain> groupedDomain(const std::string & name, const std::string & path);
}
