#pragma once

#include <roa_common/constants.hpp>
#include <roa_packet_manager/packet_manager.hpp>

namespace roa::common
{

inline roa_packet_manager::PacketManager::Command12Dof make_init_pose()
{
  roa_packet_manager::PacketManager::Command12Dof init_pos{};

  init_pos.left_hip_pitch   = -roa::constants::HIP_INIT_POS;
  init_pos.left_hip_roll    = 0.0f;
  init_pos.left_hip_yaw     = 0.0f;
  init_pos.left_knee_pitch  =  roa::constants::KNEE_INIT_POS;

  init_pos.right_hip_pitch  =  roa::constants::HIP_INIT_POS;
  init_pos.right_hip_roll   = 0.0f;
  init_pos.right_hip_yaw    = 0.0f;
  init_pos.right_knee_pitch = -roa::constants::KNEE_INIT_POS;

  init_pos.left_rsu_upper   = -roa::constants::ANKLE_INIT_POS;
  init_pos.left_rsu_lower   =  roa::constants::ANKLE_INIT_POS;
  init_pos.right_rsu_upper  =  roa::constants::ANKLE_INIT_POS;
  init_pos.right_rsu_lower  = -roa::constants::ANKLE_INIT_POS;

  return init_pos;
}

}  // namespace roa::common