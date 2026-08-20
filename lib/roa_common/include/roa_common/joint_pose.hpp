#pragma once

#include <array>

namespace roa::common
{

// Joint angles expressed in the 12-DoF virtual joint frame used by policy and
// inference code. RSU actuator angles must never be stored in this type.
struct VirtualJointPose12Dof
{
  float left_hip_pitch{};
  float right_hip_pitch{};
  float left_hip_roll{};
  float right_hip_roll{};
  float left_hip_yaw{};
  float right_hip_yaw{};
  float left_knee_pitch{};
  float right_knee_pitch{};
  float left_ankle_pitch{};
  float right_ankle_pitch{};
  float left_ankle_roll{};
  float right_ankle_roll{};
};

// Joint angles expressed in the physical actuator frame. The ankle entries
// are RSU upper/lower motor angles, not virtual ankle pitch/roll angles.
struct ActuatorPose12Dof
{
  float left_hip_pitch{};
  float right_hip_pitch{};
  float left_hip_roll{};
  float right_hip_roll{};
  float left_hip_yaw{};
  float right_hip_yaw{};
  float left_knee_pitch{};
  float right_knee_pitch{};
  float left_rsu_upper{};
  float right_rsu_upper{};
  float left_rsu_lower{};
  float right_rsu_lower{};
};

// This is the only place where a named virtual pose is mapped to a policy's
// numeric joint order. Policy must expose the existing 12-DoF Joint enum.
template<typename Policy>
constexpr std::array<float, Policy::kDof>
to_policy_joint_array(const VirtualJointPose12Dof& pose)
{
  static_assert(Policy::kDof == 12, "VirtualJointPose12Dof requires a 12-DoF policy");

  std::array<float, Policy::kDof> out{};
  out[Policy::L_HIP_PITCH] = pose.left_hip_pitch;
  out[Policy::R_HIP_PITCH] = pose.right_hip_pitch;
  out[Policy::L_HIP_ROLL] = pose.left_hip_roll;
  out[Policy::R_HIP_ROLL] = pose.right_hip_roll;
  out[Policy::L_HIP_YAW] = pose.left_hip_yaw;
  out[Policy::R_HIP_YAW] = pose.right_hip_yaw;
  out[Policy::L_KNEE_PITCH] = pose.left_knee_pitch;
  out[Policy::R_KNEE_PITCH] = pose.right_knee_pitch;
  out[Policy::L_ANKLE_PITCH] = pose.left_ankle_pitch;
  out[Policy::R_ANKLE_PITCH] = pose.right_ankle_pitch;
  out[Policy::L_ANKLE_ROLL] = pose.left_ankle_roll;
  out[Policy::R_ANKLE_ROLL] = pose.right_ankle_roll;
  return out;
}

}  // namespace roa::common
