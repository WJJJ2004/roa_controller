#pragma once

#include <array>
#include <cmath>
#include <limits>
#include <geometry_msgs/msg/twist.hpp>

namespace roa_main_controller
{
// A lost publisher must not leave the policy walking on its last command.
inline std::array<float, 3> policy_command(
  const geometry_msgs::msg::Twist * msg, bool fresh)
{
  if (msg == nullptr || !fresh) {
    return {};
  }
  const std::array<double, 3> values{msg->linear.x, msg->linear.y, msg->angular.z};
  for (const auto value : values) {
    if (!std::isfinite(value) || std::abs(value) > std::numeric_limits<float>::max()) {
      return {};
    }
  }
  return {
    static_cast<float>(values[0]),
    static_cast<float>(values[1]),
    static_cast<float>(values[2])};
}
}  // namespace roa_main_controller
