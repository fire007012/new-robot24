#pragma once
#include <cstdint>
#include <stdexcept>

namespace robot_safety {
// One instance per axis/feedback channel. Call from the owning hardware thread.
// All times are monotonic nanoseconds supplied by the caller for deterministic tests.
class Freshness {
public:
  enum class State { Unknown, Healthy, Warn, Stop, Stabilizing };
  struct Thresholds { int64_t warn_ns, stop_ns, stable_ns; };
  explicit Freshness(Thresholds thresholds) : limits_(thresholds) {
    if (limits_.warn_ns <= 0 || limits_.stop_ns <= limits_.warn_ns || limits_.stable_ns < 0)
      throw std::invalid_argument("require 0 < warn < stop and stable >= 0");
  }
  State observe(int64_t now_ns, int64_t last_feedback_ns, bool valid, bool hard_fault = false) {
    if (now_ns < 0 || (seen_time_ && now_ns < last_now_)) {
      state_ = State::Stop; stable_since_ = -1; warning_latched_ = false;
      throw std::invalid_argument("monotonic time required");
    }
    seen_time_ = true; last_now_ = now_ns;
    const bool trustworthy = valid && last_feedback_ns >= 0 && last_feedback_ns <= now_ns;
    const auto age = trustworthy ? now_ns - last_feedback_ns : limits_.stop_ns;
    const bool gap = trustworthy && last_feedback_ >= 0 && last_feedback_ns > last_feedback_ &&
      last_feedback_ns - last_feedback_ >= limits_.stop_ns;
    if (trustworthy && last_feedback_ns > last_feedback_) last_feedback_ = last_feedback_ns;
    // A newly arrived frame cannot hide a deadline crossed since the previous frame.
    // No additional debounce on hard faults, invalid data, or the freshness deadline.
    if (hard_fault || gap || !trustworthy || age >= limits_.stop_ns) {
      state_ = State::Stop; stable_since_ = -1; warning_latched_ = false; return state_;
    }
    if (age >= limits_.warn_ns) {
      stable_since_ = -1;
      if (state_ == State::Healthy || state_ == State::Warn) {
        state_ = State::Warn; warning_latched_ = true;
      } else state_ = State::Stop;
      return state_;
    }
    if (state_ == State::Unknown && limits_.stable_ns == 0) return state_ = State::Healthy;
    if (state_ == State::Healthy) return state_;
    if (stable_since_ < 0) stable_since_ = now_ns;
    if (now_ns - stable_since_ >= limits_.stable_ns) {
      warning_latched_ = false; return state_ = State::Healthy;
    }
    // Warnings stay warnings while clearing; recovery after Stop never permits motion early.
    return state_ = warning_latched_ ? State::Warn : State::Stabilizing;
  }
  State state() const { return state_; }
  bool trustworthyForMotion() const { return state_ == State::Healthy || state_ == State::Warn; }
private:
  Thresholds limits_;
  State state_ = State::Unknown;
  int64_t stable_since_ = -1, last_now_ = 0, last_feedback_ = -1;
  bool seen_time_ = false, warning_latched_ = false;
};
}
