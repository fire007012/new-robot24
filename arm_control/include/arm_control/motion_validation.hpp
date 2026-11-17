#pragma once
#include <algorithm>
#include <cmath>
#include <map>
#include <set>
#include <string>
#include <vector>
#include <sensor_msgs/msg/joint_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

namespace arm_control {
using Values = std::map<std::string, double>;
inline const std::vector<std::string> arm_names{
  "shoulder_yaw_joint", "shoulder_pitch_joint", "elbow_pitch_joint",
  "wrist_pitch_joint", "wrist_yaw_joint", "wrist_roll_joint"};
// Only this group's fields determine its feedback health. Never merge partial samples.
inline bool arm_feedback(const sensor_msgs::msg::JointState & msg, Values & positions, Values & velocities) {
  if (msg.name.size()!=msg.position.size() || msg.name.size()!=msg.velocity.size()) return false;
  Values p,v;
  for (size_t i=0;i<msg.name.size();++i) {
    if (std::find(arm_names.begin(),arm_names.end(),msg.name[i])==arm_names.end()) continue;
    if (!std::isfinite(msg.position[i]) || !std::isfinite(msg.velocity[i]) || p.count(msg.name[i])) return false;
    p[msg.name[i]]=msg.position[i];v[msg.name[i]]=msg.velocity[i];
  }
  if (p.size()!=arm_names.size()) return false;
  positions=std::move(p);velocities=std::move(v);return true;
}
inline bool stationary(const Values & velocities, double tolerance=0.05) {
  for (const auto &name:arm_names) {
    auto v=velocities.find(name);
    if(v==velocities.end() || !std::isfinite(v->second) || std::abs(v->second)>tolerance) return false;
  }
  return true;
}
inline bool matches_start(const std::vector<std::string> & names, const std::vector<double> & values,
                          const Values & measured, double tolerance=0.01) {
  if(names.size()!=values.size())return false;
  std::set<std::string> seen;
  for(size_t i=0;i<names.size();++i) {
    if(std::find(arm_names.begin(),arm_names.end(),names[i])==arm_names.end())continue;
    auto q=measured.find(names[i]);
    // These are bounded multi-turn joints: do not wrap angles by 2*pi.
    if(!seen.insert(names[i]).second || q==measured.end() || !std::isfinite(q->second) ||
       !std::isfinite(values[i]) || std::abs(q->second-values[i])>tolerance)return false;
  }
  return seen.size()==arm_names.size();
}
struct Limits { double lower,upper,velocity; };
inline bool within_limits(const Values & positions, const std::map<std::string,Limits> & limits) {
  for(const auto &name:arm_names) {
    auto q=positions.find(name);auto l=limits.find(name);
    if(q==positions.end() || l==limits.end() || !std::isfinite(q->second) ||
       q->second<l->second.lower || q->second>l->second.upper)return false;
  }
  return true;
}
inline bool valid_trajectory(const trajectory_msgs::msg::JointTrajectory & trajectory,
                             const std::map<std::string,Limits> & limits) {
  if(trajectory.points.empty() || trajectory.joint_names.size()!=arm_names.size() ||
     std::set<std::string>(trajectory.joint_names.begin(),trajectory.joint_names.end())!=
     std::set<std::string>(arm_names.begin(),arm_names.end()))return false;
  double previous=-1.;
  for(const auto &p:trajectory.points) {
    double time=p.time_from_start.sec+p.time_from_start.nanosec*1e-9;
    if(p.time_from_start.sec<0 || p.time_from_start.nanosec>=1000000000 || time<=previous ||
       p.positions.size()!=arm_names.size() || (!p.velocities.empty() && p.velocities.size()!=arm_names.size()) ||
       (!p.accelerations.empty() && p.accelerations.size()!=arm_names.size()))return false;
    previous=time;
    for(size_t i=0;i<arm_names.size();++i) {
      auto l=limits.find(trajectory.joint_names[i]);if(l==limits.end())return false;
      if(!std::isfinite(p.positions[i]) || p.positions[i]<l->second.lower || p.positions[i]>l->second.upper)return false;
      if(!p.velocities.empty() && (!std::isfinite(p.velocities[i]) || std::abs(p.velocities[i])>l->second.velocity+1e-6))return false;
      if(!p.accelerations.empty() && !std::isfinite(p.accelerations[i]))return false;
    }
  }
  return true;
}
}
