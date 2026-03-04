#pragma once
#include <cstdint>

#include "roa_interfaces/msg/rsu_solution.hpp"

namespace roa_policy_controller {

struct RsuUsableConfig {
  uint64_t timeout_ns = 30'000'000; // 30ms 기본 (50Hz 정책 + 여유)
};

inline uint64_t stamp_to_ns(const builtin_interfaces::msg::Time& t) {
  return static_cast<uint64_t>(t.sec) * 1'000'000'000ull + static_cast<uint64_t>(t.nanosec);
}

class RsuLatch {
public:
  explicit RsuLatch(RsuUsableConfig cfg) : cfg_(cfg) {}

  void write(const roa_interfaces::msg::RsuSolution& msg) {
    last_ = msg;
    has_ = true;
  }

  bool has() const { return has_; }

  // update()에서 now_ns를 넣어서 usable 판단 (절대 기다리지 않음)
  bool usable(uint64_t now_ns) const {
    if (!has_) return false;
    if (!last_.feasible) return false;
    const uint64_t t = stamp_to_ns(last_.header.stamp);
    if (now_ns < t) return false;
    return (now_ns - t) < cfg_.timeout_ns;
  }

  const roa_interfaces::msg::RsuSolution& last() const { return last_; }

private:
  RsuUsableConfig cfg_;
  roa_interfaces::msg::RsuSolution last_{};
  bool has_{false};
};

}  // namespace roa_policy_controller