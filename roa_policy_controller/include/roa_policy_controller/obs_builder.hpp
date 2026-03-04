#pragma once
#include <array>
#include <cmath>

#include "roa_policy_controller/latches.hpp"

namespace roa_policy_controller {

// NOTE: typed_policy_driver::Obs 타입은 roa_policy_driver의 wrapper에 맞춰 include를 바꿔야 함.
// 여기서는 형태만 고정하고, 실제 include는 policy_controller.hpp에서 처리해도 됨.

struct JointSnapshot {
  std::array<double, 12> q{};
  std::array<double, 12> qd{};
};

// NaN guard
inline bool finite_all(const std::array<double, 12>& a) {
  for (double v : a) if (!std::isfinite(v)) return false;
  return true;
}
inline bool finite3(const ImuOmega& w) {
  return std::isfinite(w.x) && std::isfinite(w.y) && std::isfinite(w.z);
}

}  // namespace roa_policy_controller