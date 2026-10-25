#pragma once
#include <cstdint>
#include <map>
#include <memory>
#include <mutex>
#include <string>
namespace robot_safety {
class Domain {
public:
  enum class State { Stopped, Armed, Running, Fault };
  void report(const std::string & component, bool ready);
  void fault(const std::string & reason);
  bool arm();
  bool run();
  void halt();
  bool permits(uint64_t epoch) const;
  uint64_t epoch() const;
  State state() const;
  std::string reason() const;
  void observePeriod(int64_t nanoseconds);
  std::string diagnostics() const;
private:
  mutable std::mutex mutex_;
  std::map<std::string, bool> ready_{{"RobotCanopenSystem", false}, {"CanDriverSystem", false}};
  State state_ = State::Stopped;
  uint64_t epoch_ = 0;
  std::string reason_ = "stopped";
  uint64_t samples_ = 0, sum_ns_ = 0, max_ns_ = 0, min_ns_ = UINT64_MAX, late_ = 0;
};
std::shared_ptr<Domain> domain(const std::string & name);
}
