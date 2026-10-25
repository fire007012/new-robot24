#pragma once
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <vector>
namespace robot_hardware {
class VelocityWatchdog {
public:
  explicit VelocityWatchdog(size_t size = 0) : size_(size) {}
  uint64_t reset() { ++epoch_; valid_ = false; return epoch_; }
  bool accept(const std::vector<double> &values, double stamp, uint64_t epoch) {
    valid_ = epoch == epoch_ && values.size() == size_ && std::isfinite(stamp) &&
      std::all_of(values.begin(), values.end(), [](double v) { return std::isfinite(v); });
    if (valid_) { values_ = values; stamp_ = stamp; }
    return valid_;
  }
  std::vector<double> sample(double now, double timeout) const {
    if (!valid_ || now < stamp_ || now - stamp_ > timeout) return std::vector<double>(size_, 0);
    return values_;
  }
private:
  size_t size_; uint64_t epoch_ = 0; bool valid_ = false;
  double stamp_ = 0; std::vector<double> values_;
};
}
