#include "rsu_manager_v2/rsu_lut.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <fstream>
#include <limits>
#include <stdexcept>

namespace rsu_manager_v2
{

namespace
{
template<typename T>
void read_exact(std::ifstream & stream, T & output)
{
  stream.read(reinterpret_cast<char *>(&output), sizeof(T));
  if (!stream) {throw std::runtime_error("truncated RSU LUT asset");}
}
}  // namespace

double wrap_to_pi(double value) noexcept
{
  constexpr double two_pi = 2.0 * M_PI;
  value = std::fmod(value + M_PI, two_pi);
  if (value < 0.0) {value += two_pi;}
  return value - M_PI;
}

void RsuLut::load(const std::string & path)
{
  std::ifstream stream(path, std::ios::binary);
  if (!stream) {throw std::runtime_error("cannot open RSU LUT: " + path);}
  std::array<char, 8> magic{};
  std::uint32_t version = 0;
  read_exact(stream, magic);
  read_exact(stream, version);
  read_exact(stream, rows_);
  read_exact(stream, cols_);
  read_exact(stream, roll_min_);
  read_exact(stream, roll_max_);
  read_exact(stream, pitch_min_);
  read_exact(stream, pitch_max_);
  read_exact(stream, resolution_);
  if (std::memcmp(magic.data(), "RSULUT2", 7) != 0 || version != 1 || rows_ < 2 || cols_ < 2) {
    throw std::runtime_error("invalid RSU LUT header: " + path);
  }
  cells_.resize(static_cast<std::size_t>(rows_) * cols_);
  stream.read(reinterpret_cast<char *>(cells_.data()), static_cast<std::streamsize>(cells_.size() * sizeof(Cell)));
  if (!stream) {throw std::runtime_error("truncated RSU LUT payload: " + path);}

  actuator_min_.fill(std::numeric_limits<double>::infinity());
  actuator_max_.fill(-std::numeric_limits<double>::infinity());
  for (const auto & cell : cells_) {
    for (std::size_t actuator = 0; actuator < 2; ++actuator) {
      actuator_min_[actuator] = std::min(actuator_min_[actuator], static_cast<double>(cell.value[actuator]));
      actuator_max_[actuator] = std::max(actuator_max_[actuator], static_cast<double>(cell.value[actuator]));
    }
  }
}

LutQuery RsuLut::query(double roll, double pitch) const noexcept
{
  LutQuery out;
  if (!std::isfinite(roll) || !std::isfinite(pitch) || cells_.empty() ||
    roll < roll_min_ || roll > roll_max_ || pitch < pitch_min_ || pitch > pitch_max_)
  {
    return out;
  }
  const double row_coordinate = (roll - roll_min_) / (roll_max_ - roll_min_) * (rows_ - 1);
  const double col_coordinate = (pitch - pitch_min_) / (pitch_max_ - pitch_min_) * (cols_ - 1);
  const auto i = std::min<std::uint32_t>(static_cast<std::uint32_t>(std::floor(row_coordinate)), rows_ - 2);
  const auto j = std::min<std::uint32_t>(static_cast<std::uint32_t>(std::floor(col_coordinate)), cols_ - 2);
  const double u = std::clamp(row_coordinate - i, 0.0, 1.0);
  const double v = std::clamp(col_coordinate - j, 0.0, 1.0);
  const auto & c00 = cells_[static_cast<std::size_t>(i) * cols_ + j].value;
  const auto & c10 = cells_[static_cast<std::size_t>(i + 1) * cols_ + j].value;
  const auto & c01 = cells_[static_cast<std::size_t>(i) * cols_ + j + 1].value;
  const auto & c11 = cells_[static_cast<std::size_t>(i + 1) * cols_ + j + 1].value;
  std::array<double, 6> value{};
  for (std::size_t k = 0; k < value.size(); ++k) {
    value[k] = (1.0 - u) * (1.0 - v) * c00[k] + u * (1.0 - v) * c10[k] +
      (1.0 - u) * v * c01[k] + u * v * c11[k];
    if (!std::isfinite(value[k])) {return out;}
  }
  // The selected physical branch is continuous throughout the certified rectangle.
  const double base0 = c00[0];
  const double base1 = c00[1];
  auto interpolate_wrapped = [&](std::size_t k, double base) {
      const double a00 = base + wrap_to_pi(c00[k] - base);
      const double a10 = base + wrap_to_pi(c10[k] - base);
      const double a01 = base + wrap_to_pi(c01[k] - base);
      const double a11 = base + wrap_to_pi(c11[k] - base);
      return wrap_to_pi((1.0 - u) * (1.0 - v) * a00 + u * (1.0 - v) * a10 +
        (1.0 - u) * v * a01 + u * v * a11);
    };
  out.alpha = {interpolate_wrapped(0, base0), interpolate_wrapped(1, base1)};
  out.jacobian = {value[2], value[3], value[4], value[5]};
  out.valid = true;
  return out;
}

}  // namespace rsu_manager_v2
