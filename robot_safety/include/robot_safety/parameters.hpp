#pragma once
#include <map>
#include <string>
#include <stdexcept>
#include <cstdint>
#include <limits>
namespace robot_safety {
template <class Values>
inline int64_t millisecondsParameter(const Values & values,
                                     const std::string & name,int64_t fallback) {
  auto it=values.find(name);if(it==values.end())return fallback;
  std::size_t used=0;const auto value=std::stoll(it->second,&used);
  if(used!=it->second.size()||value<0||value>std::numeric_limits<int64_t>::max()/1000000)
    throw std::invalid_argument("invalid milliseconds parameter: "+name);
  return value;
}
}
