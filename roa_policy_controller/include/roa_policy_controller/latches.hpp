#pragma once
#include <atomic>
#include <cstdint>

namespace roa_policy_controller {

// 아주 단순 latch: writer(ROS callback) -> reader(update)
// update()에서는 "복사만" 수행. (락/대기 금지)
template <typename T>
class AtomicLatch {
public:
  AtomicLatch() : seq_(0) {}

  void write(const T& v) {
    // 2-phase commit: data_ 먼저 쓰고 seq 증가
    data_ = v;
    seq_.fetch_add(1, std::memory_order_release);
  }

  // 최신값 snapshot. 성공하면 true
  bool read(T& out) const {
    const uint64_t s1 = seq_.load(std::memory_order_acquire);
    out = data_;
    const uint64_t s2 = seq_.load(std::memory_order_acquire);
    return (s1 == s2);
  }

private:
  // T는 trivially copy 가능한 형태를 권장(고정 크기 struct)
  mutable T data_{};
  mutable std::atomic<uint64_t> seq_;
};

// 필요한 최소 입력 예시 (나중에 확장 가능)
struct ImuOmega {
  double x{0}, y{0}, z{0};
};

struct CmdVel {
  double vx{0}, vy{0}, wz{0};
};

}  // namespace roa_policy_controller