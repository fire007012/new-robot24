#include "robot_safety/domain.hpp"
#include <algorithm>
#include <sstream>
#include <iomanip>
namespace robot_safety {
void Domain::report(const std::string & component, bool ready) {
  std::lock_guard<std::mutex> lock(mutex_);
  ready_[component] = ready;
  if (!ready && (state_ == State::Running || state_ == State::Armed)) {
    state_ = State::Fault; ++epoch_; reason_ = component + " lost readiness";
  }
}
void Domain::fault(const std::string & reason) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (state_ != State::Fault) ++epoch_;
  state_ = State::Fault; reason_ = reason;
}
bool Domain::arm() {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!std::all_of(ready_.begin(), ready_.end(), [](const auto & v) { return v.second; })) return false;
  ++epoch_; state_ = State::Armed; reason_ = "armed"; return true;
}
bool Domain::run() {
  std::lock_guard<std::mutex> lock(mutex_);
  if (state_ != State::Armed) return false;
  state_ = State::Running; reason_ = "running"; return true;
}
void Domain::halt() {
  std::lock_guard<std::mutex> lock(mutex_);
  ++epoch_; state_ = State::Stopped; reason_ = "halted";
}
bool Domain::permits(uint64_t epoch) const {
  std::lock_guard<std::mutex> lock(mutex_);
  return state_ == State::Running && epoch == epoch_;
}
uint64_t Domain::epoch() const { std::lock_guard<std::mutex> lock(mutex_); return epoch_; }
Domain::State Domain::state() const { std::lock_guard<std::mutex> lock(mutex_); return state_; }
std::string Domain::reason() const { std::lock_guard<std::mutex> lock(mutex_); return reason_; }
void Domain::observePeriod(int64_t nanoseconds) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (state_ != State::Running || nanoseconds <= 0) return;
  const auto ns = static_cast<uint64_t>(nanoseconds);
  ++samples_; sum_ns_ += ns; max_ns_ = std::max(max_ns_, ns); min_ns_ = std::min(min_ns_, ns);
  if (ns > 7500000) ++late_;  // Report observed misses against 200 Hz; never claim a real-time guarantee.
}
std::string Domain::diagnostics() const {
  std::lock_guard<std::mutex> lock(mutex_);
  std::ostringstream out;
  out << "{\"state\":" << static_cast<int>(state_) << ",\"epoch\":" << epoch_
      << ",\"reason\":" << std::quoted(reason_) << ",\"samples\":" << samples_
      << ",\"period_min_ns\":" << (samples_ ? min_ns_ : 0)
      << ",\"period_mean_ns\":" << (samples_ ? sum_ns_ / samples_ : 0)
      << ",\"period_max_ns\":" << max_ns_ << ",\"period_over_7_5ms\":" << late_ << "}";
  return out.str();
}
std::shared_ptr<Domain> domain(const std::string & name) {
  static std::mutex mutex;
  static std::map<std::string, std::shared_ptr<Domain>> domains;
  std::lock_guard<std::mutex> lock(mutex);
  auto & result = domains[name]; if (!result) result = std::make_shared<Domain>(); return result;
}
}
