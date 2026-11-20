#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>

namespace rsu_manager_v3 {
struct TorqueConfig {
  std::array<double, 2> kp{34.25,25.0}, kd{1.644,1.2};
  double virtual_limit{11.9}, motor_limit{11.9}, slew_rate{200.0};
  double condition_limit{100.0};
  void validate() const {
    for (auto x : {kp[0],kp[1],kd[0],kd[1]})
      if (!std::isfinite(x) || x < 0) throw std::invalid_argument("invalid virtual gains");
    for (auto x : {virtual_limit,motor_limit,slew_rate,condition_limit})
      if (!std::isfinite(x) || x <= 0) throw std::invalid_argument("invalid torque limits");
  }
};
struct TorqueResult {
  std::array<double,2> virtual_torque{}, requested{}, applied{};
  bool valid{false}, saturated{false};
};
// A = d(alpha)/d(q), row-major. Solve A^T tau_motor = tau_virtual.
// Simultaneous right-side q/alpha mirroring leaves A unchanged in external axes.
inline TorqueResult map_torque(const std::array<double,4>& a,
    const std::array<double,2>& q, const std::array<double,2>& qd,
    const std::array<double,2>& target, const std::array<double,2>& previous,
    double dt, const TorqueConfig& c) {
  TorqueResult out;
  for (auto x : a) if (!std::isfinite(x)) return out;
  for (const auto& values : {q,qd,target,previous})
    for (auto x : values) if (!std::isfinite(x)) return out;
  if (!std::isfinite(dt) || dt <= 0 || dt > .1) return out;
  const double det=a[0]*a[3]-a[1]*a[2];
  const double norm=std::max(std::abs(a[0])+std::abs(a[1]),std::abs(a[2])+std::abs(a[3]));
  const double invnorm=std::max(std::abs(a[3])+std::abs(a[1]),std::abs(a[2])+std::abs(a[0]));
  if (!std::isfinite(det) || std::abs(det)<1e-10 || norm*invnorm/std::abs(det)>c.condition_limit) return out;
  for (int i=0;i<2;++i) {
    const double raw=c.kp[i]*(target[i]-q[i])-c.kd[i]*qd[i];
    if (!std::isfinite(raw)) return TorqueResult{};
    out.virtual_torque[i]=std::clamp(raw,-c.virtual_limit,c.virtual_limit);
    out.saturated |= out.virtual_torque[i]!=raw;
  }
  out.requested={(a[3]*out.virtual_torque[0]-a[2]*out.virtual_torque[1])/det,
                 (-a[1]*out.virtual_torque[0]+a[0]*out.virtual_torque[1])/det};
  for (int i=0;i<2;++i) {
    if (!std::isfinite(out.requested[i])) return TorqueResult{};
    const double bounded=std::clamp(out.requested[i],-c.motor_limit,c.motor_limit);
    out.applied[i]=std::clamp(bounded,previous[i]-c.slew_rate*dt,previous[i]+c.slew_rate*dt);
    out.applied[i]=std::clamp(out.applied[i],-c.motor_limit,c.motor_limit);
    out.saturated |= out.applied[i]!=out.requested[i];
  }
  out.valid=true;
  return out;
}
}  // namespace rsu_manager_v3
