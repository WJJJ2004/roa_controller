#pragma once

#include <array>

#include "rsu_manager_v2/rsu_lut.hpp"

namespace rsu_manager_v2
{

struct StateResult
{
  std::array<double, 2> q{};
  std::array<double, 2> qd{};
  std::array<double, 4> jacobian{};
  bool valid{false};
  bool degraded{false};
};

class StateEstimator
{
public:
  explicit StateEstimator(const RsuLut & lut);
  void reset(const std::array<double, 2> & q, const std::array<double, 2> & alpha);
  StateResult update(
    const std::array<double, 2> & motor_position,
    const std::array<double, 2> & motor_velocity, double dt) noexcept;

private:
  const RsuLut & lut_;
  std::array<double, 2> q_prev_{};
  std::array<double, 2> qd_prev_{};
  std::array<double, 2> alpha_seed_{};
  bool initialized_{false};
};

struct ImpedanceConfig
{
  double virtual_pitch_kp{25.0};
  double virtual_pitch_kd{1.2};
  double virtual_roll_scale{1.37};
  std::array<double, 2> kp_min{5.0, 5.0};
  std::array<double, 2> kp_max{25.0, 25.0};
  std::array<double, 2> kd_min{0.2, 0.2};
  // Main controller accepts kd strictly below 5.0.
  std::array<double, 2> kd_max{4.9, 4.9};
  std::array<double, 2> default_kp{9.0, 9.0};
  std::array<double, 2> default_kd{2.25, 2.25};
  double condition_fail{50.0};
  double sigma_min{1e-4};
  double fit_error_fail{0.50};
};

struct ImpedanceResult
{
  std::array<double, 2> kp{};
  std::array<double, 2> kd{};
  double condition{0.0};
  bool saturated{false};
  bool valid{false};
};

class ImpedanceMapper
{
public:
  explicit ImpedanceMapper(ImpedanceConfig config = {});
  ImpedanceResult compute(const std::array<double, 4> & jacobian, bool estimator_valid) noexcept;

private:
  struct Fit {std::array<double, 2> gain{}; double error{0.0}; bool saturated{false}; bool valid{false};};
  Fit fit(const std::array<double, 4> & jacobian, double roll_gain, double pitch_gain,
    const std::array<double, 2> & lower, const std::array<double, 2> & upper) const noexcept;
  ImpedanceConfig config_;
  std::array<double, 2> previous_kp_{};
  std::array<double, 2> previous_kd_{};
  bool has_valid_{false};
};

}  // namespace rsu_manager_v2
