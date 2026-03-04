#pragma once
#include <array>
#include <algorithm>
#include <cmath>

#include "roa_policy_controller/rsu_latch.hpp"

namespace roa_policy_controller {

struct CommandLimits {
  double pos_min = -3.14;
  double pos_max = +3.14;
  double max_step = 0.2; // tick당 rate limit(예시)
};

inline double clamp(double x, double lo, double hi) {
  return std::max(lo, std::min(hi, x));
}

inline bool finite_all(const std::array<double, 12>& a) {
  for (double v : a) if (!std::isfinite(v)) return false;
  return true;
}

// policy action을 12DOF position target으로 확장/합성하는 자리
// 초기 버전: "policy가 이미 12개를 준다"거나, "10DOF->12DOF 매핑" 등은 여기서 통일
class CommandComposer {
public:
  explicit CommandComposer(CommandLimits lim) : lim_(lim) {}

  // prev_cmd: 이전 tick에서 보낸 안전 cmd
  // action12: policy 출력(또는 확장된 12) 가정
  // rsu: usable이면 추가 반영(초기 버전에서는 ankle 관련 값만 future hook)
  bool compose(const std::array<double, 12>& prev_cmd,
               const std::array<double, 12>& action12,
               const RsuLatch& rsu,
               bool rsu_usable,
               std::array<double, 12>& out_cmd) const
  {
    out_cmd = prev_cmd;

    // 기본: action을 목표로
    for (size_t i = 0; i < 12; ++i) {
      double target = action12[i];
      if (!std::isfinite(target)) return false;

      // clamp
      target = clamp(target, lim_.pos_min, lim_.pos_max);

      // rate limit
      const double delta = clamp(target - prev_cmd[i], -lim_.max_step, lim_.max_step);
      out_cmd[i] = prev_cmd[i] + delta;
    }

    // RSU actuator 반영은 "actuator_net 미구현"이므로 초기에는 hook만 둔다.
    // rsu_usable이 false면: actuator command hold (스펙 6️⃣)
    (void)rsu;
    (void)rsu_usable;

    return finite_all(out_cmd);
  }

private:
  CommandLimits lim_;
};

}  // namespace roa_policy_controller