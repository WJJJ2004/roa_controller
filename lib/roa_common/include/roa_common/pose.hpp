#pragma once

#include <roa_common/fixed_joint_poses.hpp>
#include <roa_packet_manager/packet_manager.hpp>

namespace roa::common
{

inline roa_packet_manager::PacketManager::Command12Dof make_init_pose()
{
  const auto& pose = roa::common::fixed_pose::kHardwareBootActuatorPose;
  roa_packet_manager::PacketManager::Command12Dof init_pos{};

  init_pos.left_hip_pitch = pose.left_hip_pitch;
  init_pos.right_hip_pitch = pose.right_hip_pitch;
  init_pos.left_hip_roll = pose.left_hip_roll;
  init_pos.right_hip_roll = pose.right_hip_roll;
  init_pos.left_hip_yaw = pose.left_hip_yaw;
  init_pos.right_hip_yaw = pose.right_hip_yaw;
  init_pos.left_knee_pitch = pose.left_knee_pitch;
  init_pos.right_knee_pitch = pose.right_knee_pitch;
  init_pos.left_rsu_upper = pose.left_rsu_upper;
  init_pos.right_rsu_upper = pose.right_rsu_upper;
  init_pos.left_rsu_lower = pose.left_rsu_lower;
  init_pos.right_rsu_lower = pose.right_rsu_lower;

  return init_pos;
}

}  // namespace roa::common
