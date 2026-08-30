#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>

#include <roa_common/joint_pose.hpp>

namespace roa::common::joint_limits
{

struct VirtualJointLimits12Dof
{
  VirtualJointPose12Dof lower{};
  VirtualJointPose12Dof upper{};
};

struct ClipResult
{
  bool valid{true};
  std::size_t clipped_count{0};
};

// Hard position limits imported from the Isaac training USD (roa-v2.usd).
// All values are radians in the 12-DoF virtual-joint frame.
constexpr VirtualJointLimits12Dof make_training_usd_virtual_joint_limits()
{
  VirtualJointLimits12Dof limits{};

  limits.lower.left_hip_pitch = -1.570796f;
  limits.upper.left_hip_pitch = 1.570796f;
  limits.lower.right_hip_pitch = -1.570796f;
  limits.upper.right_hip_pitch = 1.570796f;

  limits.lower.left_hip_roll = -0.150000f;
  limits.upper.left_hip_roll = 0.468000f;
  limits.lower.right_hip_roll = -0.468000f;
  limits.upper.right_hip_roll = 0.150000f;

  limits.lower.left_hip_yaw = -1.570796f;
  limits.upper.left_hip_yaw = 1.570796f;
  limits.lower.right_hip_yaw = -1.570796f;
  limits.upper.right_hip_yaw = 1.570796f;

  limits.lower.left_knee_pitch = -1.570796f;
  limits.upper.left_knee_pitch = 1.570796f;
  limits.lower.right_knee_pitch = -1.570796f;
  limits.upper.right_knee_pitch = 1.570796f;

  limits.lower.left_ankle_pitch = -0.628319f;
  limits.upper.left_ankle_pitch = 0.689405f;
  limits.lower.right_ankle_pitch = -0.689405f;
  limits.upper.right_ankle_pitch = 0.628319f;

  limits.lower.left_ankle_roll = -0.314159f;
  limits.upper.left_ankle_roll = 0.305433f;
  limits.lower.right_ankle_roll = -0.305433f;
  limits.upper.right_ankle_roll = 0.314159f;

  return limits;
}

inline constexpr auto kTrainingUsdVirtualJointLimits =
  make_training_usd_virtual_joint_limits();

// Clips a policy-ordered virtual-joint target transactionally. If any input
// is NaN/Inf, no element is modified and valid=false is returned.
template<typename Policy>
ClipResult clip_policy_joint_target(
  std::array<float, Policy::kActDim>& q_target,
  const VirtualJointLimits12Dof& limits = kTrainingUsdVirtualJointLimits) noexcept
{
  static_assert(Policy::kActDim == 12, "Virtual joint limits require a 12-DoF policy");

  for (const float value : q_target) {
    if (!std::isfinite(value)) {
      return ClipResult{false, 0};
    }
  }

  const auto lower = to_policy_joint_array<Policy>(limits.lower);
  const auto upper = to_policy_joint_array<Policy>(limits.upper);

  ClipResult result{};
  for (std::size_t i = 0; i < q_target.size(); ++i) {
    const float clipped = std::clamp(q_target[i], lower[i], upper[i]);
    if (clipped != q_target[i]) {
      ++result.clipped_count;
      q_target[i] = clipped;
    }
  }
  return result;
}

}  // namespace roa::common::joint_limits
