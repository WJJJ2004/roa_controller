#pragma once

#include <array>
#include <cstdint>
#include <string>
#include <vector>

namespace rsu_manager_v2
{

struct LutQuery
{
  std::array<double, 2> alpha{};
  // Row-major d(alpha[0:2]) / d(roll,pitch).
  std::array<double, 4> jacobian{};
  bool valid{false};
};

class RsuLut
{
public:
  void load(const std::string & path);
  LutQuery query(double roll, double pitch) const noexcept;

  double roll_min() const noexcept {return roll_min_;}
  double roll_max() const noexcept {return roll_max_;}
  double pitch_min() const noexcept {return pitch_min_;}
  double pitch_max() const noexcept {return pitch_max_;}
  std::array<double, 2> actuator_min() const noexcept {return actuator_min_;}
  std::array<double, 2> actuator_max() const noexcept {return actuator_max_;}

private:
  struct Cell {std::array<float, 6> value{};};
  std::uint32_t rows_{0};
  std::uint32_t cols_{0};
  double roll_min_{0.0};
  double roll_max_{0.0};
  double pitch_min_{0.0};
  double pitch_max_{0.0};
  double resolution_{0.0};
  std::vector<Cell> cells_;
  std::array<double, 2> actuator_min_{};
  std::array<double, 2> actuator_max_{};
};

double wrap_to_pi(double value) noexcept;

}  // namespace rsu_manager_v2
