#include "rsu_manager_v2/rsu_model.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace rsu_manager_v2
{
namespace
{
bool solve_symmetric_2x2(double a, double b, double d, double y0, double y1, std::array<double, 2> & x)
{
  const double determinant = a * d - b * b;
  if (!std::isfinite(determinant) || std::abs(determinant) < 1e-15) {return false;}
  x = {(d * y0 - b * y1) / determinant, (-b * y0 + a * y1) / determinant};
  return std::isfinite(x[0]) && std::isfinite(x[1]);
}

std::array<double, 2> damped_inverse(
  const std::array<double, 4> & j, const std::array<double, 2> & y, double lambda, bool & ok)
{
  const double a = j[0] * j[0] + j[2] * j[2] + lambda;
  const double b = j[0] * j[1] + j[2] * j[3];
  const double d = j[1] * j[1] + j[3] * j[3] + lambda;
  const double y0 = j[0] * y[0] + j[2] * y[1];
  const double y1 = j[1] * y[0] + j[3] * y[1];
  std::array<double, 2> x{};
  ok = solve_symmetric_2x2(a, b, d, y0, y1, x);
  return x;
}

double norm2(const std::array<double, 2> & x) {return std::hypot(x[0], x[1]);}
}  // namespace

StateEstimator::StateEstimator(const RsuLut & lut) : lut_(lut) {}

void StateEstimator::reset(const std::array<double, 2> & q, const std::array<double, 2> & alpha)
{
  q_prev_ = q;
  qd_prev_ = {0.0, 0.0};
  alpha_seed_ = alpha;
  initialized_ = true;
}

StateResult StateEstimator::update(
  const std::array<double, 2> & motor_position,
  const std::array<double, 2> & motor_velocity, double dt) noexcept
{
  StateResult out;
  if (!std::isfinite(dt) || dt < 1e-5 || dt > 0.1) {return out;}
  for (double value : motor_position) {if (!std::isfinite(value)) {return out;}}
  for (double value : motor_velocity) {if (!std::isfinite(value)) {return out;}}
  auto q = q_prev_;
  LutQuery query;
  for (int iteration = 0; iteration < 8; ++iteration) {
    query = lut_.query(q[0], q[1]);
    if (!query.valid) {return out;}
    std::array<double, 2> residual{
      wrap_to_pi(query.alpha[0] - motor_position[0]),
      wrap_to_pi(query.alpha[1] - motor_position[1])};
    if (norm2(residual) < 1e-8) {break;}
    bool ok = false;
    auto correction = damped_inverse(query.jacobian, residual, 1e-6, ok);
    if (!ok) {return out;}
    const double correction_norm = norm2(correction);
    if (correction_norm > M_PI / 90.0) {
      correction[0] *= (M_PI / 90.0) / correction_norm;
      correction[1] *= (M_PI / 90.0) / correction_norm;
    }
    q[0] = std::clamp(q[0] - correction[0], lut_.roll_min(), lut_.roll_max());
    q[1] = std::clamp(q[1] - correction[1], lut_.pitch_min(), lut_.pitch_max());
  }
  query = lut_.query(q[0], q[1]);
  if (!query.valid) {return out;}
  const std::array<double, 2> final_residual{
    wrap_to_pi(query.alpha[0] - motor_position[0]), wrap_to_pi(query.alpha[1] - motor_position[1])};
  if (norm2(final_residual) > 1e-3) {return out;}

  bool velocity_ok = false;
  const auto qd_jac = damped_inverse(query.jacobian, motor_velocity, 3e-7, velocity_ok);
  if (!velocity_ok) {return out;}
  std::array<double, 2> qd_fd{};
  if (initialized_) {
    qd_fd = {wrap_to_pi(q[0] - q_prev_[0]) / dt, wrap_to_pi(q[1] - q_prev_[1]) / dt};
  }
  constexpr double beta = 0.95;
  constexpr double tau = 1.0 / (2.0 * M_PI * 1.5);
  const double gamma = dt / (tau + dt);
  for (std::size_t i = 0; i < 2; ++i) {
    const double raw = beta * qd_jac[i] + (1.0 - beta) * qd_fd[i];
    out.qd[i] = std::clamp(qd_prev_[i] + gamma * (raw - qd_prev_[i]), -5.235987756, 5.235987756);
  }
  out.q = q;
  out.jacobian = query.jacobian;
  out.valid = true;
  q_prev_ = q;
  qd_prev_ = out.qd;
  alpha_seed_ = query.alpha;
  initialized_ = true;
  return out;
}

ImpedanceMapper::ImpedanceMapper(ImpedanceConfig config)
: config_(config), previous_kp_(config.default_kp), previous_kd_(config.default_kd) {}

ImpedanceMapper::Fit ImpedanceMapper::fit(
  const std::array<double, 4> & j, double roll_gain, double pitch_gain,
  const std::array<double, 2> & lower, const std::array<double, 2> & upper) const noexcept
{
  const double root2 = std::sqrt(2.0);
  const std::array<double, 6> a{
    j[0] * j[0], j[2] * j[2],
    root2 * j[0] * j[1], root2 * j[2] * j[3],
    j[1] * j[1], j[3] * j[3]};
  const std::array<double, 3> b{roll_gain, 0.0, pitch_gain};
  const double n00 = a[0] * a[0] + a[2] * a[2] + a[4] * a[4];
  const double n01 = a[0] * a[1] + a[2] * a[3] + a[4] * a[5];
  const double n11 = a[1] * a[1] + a[3] * a[3] + a[5] * a[5];
  const double y0 = a[0] * b[0] + a[2] * b[1] + a[4] * b[2];
  const double y1 = a[1] * b[0] + a[3] * b[1] + a[5] * b[2];
  std::array<double, 2> unconstrained{};
  if (!solve_symmetric_2x2(n00, n01, n11, y0, y1, unconstrained)) {return {};}
  std::vector<std::array<double, 2>> candidates;
  auto add = [&](double x0, double x1) {
      if (std::isfinite(x0) && std::isfinite(x1) && x0 >= lower[0] && x0 <= upper[0] &&
        x1 >= lower[1] && x1 <= upper[1]) {candidates.push_back({x0, x1});}
    };
  add(unconstrained[0], unconstrained[1]);
  for (double x0 : {lower[0], upper[0]}) {
    add(x0, std::clamp((y1 - n01 * x0) / n11, lower[1], upper[1]));
  }
  for (double x1 : {lower[1], upper[1]}) {
    add(std::clamp((y0 - n01 * x1) / n00, lower[0], upper[0]), x1);
  }
  for (double x0 : {lower[0], upper[0]}) {for (double x1 : {lower[1], upper[1]}) {add(x0, x1);}}
  if (candidates.empty()) {return {};}
  double best_cost = std::numeric_limits<double>::infinity();
  std::array<double, 2> best{};
  for (const auto & x : candidates) {
    const double e0 = a[0] * x[0] + a[1] * x[1] - b[0];
    const double e1 = a[2] * x[0] + a[3] * x[1] - b[1];
    const double e2 = a[4] * x[0] + a[5] * x[1] - b[2];
    const double cost = e0 * e0 + e1 * e1 + e2 * e2;
    if (cost < best_cost) {best_cost = cost; best = x;}
  }
  Fit result;
  result.gain = best;
  result.error = std::sqrt(best_cost) / std::max(std::hypot(roll_gain, pitch_gain), 1e-12);
  result.saturated = std::abs(best[0] - unconstrained[0]) > 1e-8 || std::abs(best[1] - unconstrained[1]) > 1e-8;
  result.valid = std::isfinite(result.error);
  return result;
}

ImpedanceResult ImpedanceMapper::compute(
  const std::array<double, 4> & j, bool estimator_valid) noexcept
{
  ImpedanceResult out;
  out.kp = has_valid_ ? previous_kp_ : config_.default_kp;
  out.kd = has_valid_ ? previous_kd_ : config_.default_kd;
  if (!estimator_valid) {return out;}
  const double s00 = j[0] * j[0] + j[1] * j[1];
  const double s01 = j[0] * j[2] + j[1] * j[3];
  const double s11 = j[2] * j[2] + j[3] * j[3];
  const double trace = s00 + s11;
  const double disc = std::sqrt(std::max(0.0, (s00 - s11) * (s00 - s11) + 4.0 * s01 * s01));
  const double sigma_max = std::sqrt(std::max(0.0, 0.5 * (trace + disc)));
  const double sigma_min = std::sqrt(std::max(0.0, 0.5 * (trace - disc)));
  out.condition = sigma_min > 1e-15 ? sigma_max / sigma_min : std::numeric_limits<double>::infinity();
  if (!std::isfinite(out.condition) || out.condition > config_.condition_fail || sigma_min < config_.sigma_min) {
    return out;
  }
  const auto kp = fit(j, config_.virtual_pitch_kp * config_.virtual_roll_scale,
      config_.virtual_pitch_kp, config_.kp_min, config_.kp_max);
  const auto kd = fit(j, config_.virtual_pitch_kd * config_.virtual_roll_scale,
      config_.virtual_pitch_kd, config_.kd_min, config_.kd_max);
  if (!kp.valid || !kd.valid || kp.error > config_.fit_error_fail || kd.error > config_.fit_error_fail) {return out;}
  out.kp = kp.gain;
  out.kd = kd.gain;
  out.saturated = kp.saturated || kd.saturated;
  out.valid = true;
  previous_kp_ = out.kp;
  previous_kd_ = out.kd;
  has_valid_ = true;
  return out;
}

}  // namespace rsu_manager_v2
