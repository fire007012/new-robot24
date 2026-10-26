#include "robot_safety/policy.hpp"
#include <yaml-cpp/yaml.h>
#include <stdexcept>
#include <fstream>
#include <sstream>
namespace robot_safety {
namespace {
void unique(const YAML::Node & node) {
  if (node.IsMap()) {
    std::set<std::string> keys;
    for (const auto & entry : node) {
      if (!keys.insert(entry.first.as<std::string>()).second) throw std::invalid_argument("duplicate policy key");
      unique(entry.second);
    }
  } else if (node.IsSequence()) for (const auto & child : node) unique(child);
}
}
Policy Policy::load(const std::string & path) {
  auto root = YAML::LoadFile(path); unique(root); Policy out;
  out.version = root["version"].as<std::string>();
  if (out.version != "group-policy-v1") throw std::invalid_argument("unsupported group policy version");
  const std::set<std::string> groups{"arm", "flippers", "base", "gripper"};
  std::set<std::string> owners;
  for (const auto & entry : root["axes"]) {
    Domain::Axis axis; axis.group = entry.second["group"].as<std::string>();
    axis.backend = entry.second["backend"].as<std::string>();
    out.axes.emplace(entry.first.as<std::string>(), axis);
  }
  for (const auto & entry : root["groups"]) {
    const auto group = entry.first.as<std::string>();
    if (!groups.count(group)) throw std::invalid_argument("unknown policy group");
    auto & list = out.controllers[group];
    for (const auto & controller : entry.second["controllers"]) {
      const auto name = controller.as<std::string>();
      if (name.empty() || !owners.insert(name).second) throw std::invalid_argument("controller ownership conflict");
      list.push_back(name);
    }
    if (list.empty() || (group == "flippers" ?
      list.size() != 2 || entry.second["controller_selection"].as<std::string>() != "exactly_one" : list.size() != 1))
      throw std::invalid_argument("invalid controller selection");
  }
  if (out.controllers.size() != groups.size()) throw std::invalid_argument("missing policy group");
  for (const auto & entry : root["conditions"]) {
    if (entry.second["initial"].as<std::string>() != "unknown")
      throw std::invalid_argument("configuration cannot assert physical evidence");
  }
  for (const auto & entry : root["associations"]) {
    Domain::Association a{entry["source_group"].as<std::string>(), entry["category"].as<std::string>(),
      entry["dependent_group"].as<std::string>(), entry["condition"].as<std::string>(), entry["source"].as<std::string>()};
    if (entry["unknown"].as<std::string>() != "block" || !root["conditions"][a.condition])
      throw std::invalid_argument("association must block unknown evidence");
    out.associations.push_back(a);
  }
  const std::map<std::string, std::string> ownership{
    {"shoulder_yaw_joint","arm"},{"shoulder_pitch_joint","arm"},{"elbow_pitch_joint","arm"},
    {"wrist_pitch_joint","arm"},{"wrist_yaw_joint","arm"},{"wrist_roll_joint","arm"},
    {"left_front_arm_joint","flippers"},{"right_front_arm_joint","flippers"},
    {"left_rear_arm_joint","flippers"},{"right_rear_arm_joint","flippers"},
    {"left_front_wheel_joint","base"},{"right_front_wheel_joint","base"},{"left_gripper_finger_joint","gripper"}};
  if (out.axes.size() != ownership.size()) throw std::invalid_argument("v1 requires thirteen owned axes");
  for (const auto & axis : ownership) {
    const auto & actual = out.axes.at(axis.first);
    const auto expected_backend = axis.second == "flippers" || axis.first.rfind("shoulder_",0) == 0 ?
      "RobotCanopenSystem" : "CanDriverSystem";
    if (actual.group != axis.second || actual.backend != expected_backend)
      throw std::invalid_argument("v1 axis/group/backend ownership mismatch: " + axis.first);
  }
  for (const auto & required : std::vector<Domain::Association>{
      {"arm","*","base","arm_secured","commissioned_arm_pose_and_brake"},
      {"flippers","*","base","support_trusted","commissioned_support_state"},
      {"gripper","*","arm","payload_retained","commissioned_payload_state"}}) {
    bool found = false;
    for (const auto & a : out.associations) if (a.source_group == required.source_group &&
        a.category == required.category && a.dependent_group == required.dependent_group &&
        a.condition == required.condition && a.source == required.source) found = true;
    if (!found) throw std::invalid_argument("v1 mandatory mechanical association missing");
  }
  if (!root["events"]["known_emcy"].IsSequence() || root["events"]["known_emcy"].size() != 0 ||
      root["events"]["unknown_emcy"]["severity"].as<std::string>() != "group_stop")
    throw std::invalid_argument("unsupported EMCY classification; preserve unknown-code group stop");
  Domain validation; validation.configure(out.axes, out.associations, out.version);
  return out;
}
std::shared_ptr<Domain> groupedDomain(const std::string & name, const std::string & path) {
  std::ifstream stream(path);
  if (!stream) throw std::invalid_argument("cannot read group policy: " + path);
  std::ostringstream content; content << stream.rdbuf();
  static std::mutex mutex;
  static std::map<std::string, std::string> configured;
  std::lock_guard<std::mutex> lock(mutex);
  auto d = domain(name);
  auto it = configured.find(name);
  if (it == configured.end()) {
    auto policy = Policy::load(path); d->configure(policy.axes, policy.associations, policy.version);
    configured.emplace(name, content.str());
  } else if (it->second != content.str()) throw std::logic_error("conflicting group policy for domain " + name);
  return d;
}
}
