#pragma once

#include <roa_common/joint_pose.hpp>

/*
  LAST EDITED: 2026-08-20

  NOTE:
  * ALL VALUE IS IN RADIAN
  * IF YOU WANT TO CHANGE BOOT SEQ DEFAULT POSE, YOU SHOULD CHANGE make_blend_start_virtual_pose() AND make_hardware_boot_actuator_pose() TOGETHER.
*/

namespace roa::common::fixed_pose
{

// Training-time default joint angles. These values are the reference for
// obs.q_rel and for mapping policy actions to absolute virtual joint targets.
// Keep all 12 joints explicit even when the pose is symmetric.
constexpr VirtualJointPose12Dof make_policy_default_virtual_pose()
{
  // WARN: DO NOT EDIT THESE VALUES. UNLESS YOU EDIT THE POLICY'S TRAINING DATA.
  VirtualJointPose12Dof pose{};
  pose.left_hip_pitch = -0.42616745829582214f;
  pose.right_hip_pitch = 0.42616745829582214f;
  pose.left_hip_roll = -0.04507143050432205f;
  pose.right_hip_roll = 0.04507143050432205f;
  pose.left_hip_yaw = -0.13962633907794952f;
  pose.right_hip_yaw = 0.13962633907794952f;
  pose.left_knee_pitch = 0.76794487237930300f;
  pose.right_knee_pitch = -0.76794487237930300f;
  pose.left_ankle_pitch = -0.36651915311813354f;
  pose.right_ankle_pitch = 0.36651915311813354f;
  pose.left_ankle_roll = 0.0f;
  pose.right_ankle_roll = 0.0f;
  // WARN: DO NOT EDIT THESE VALUES. UNLESS YOU EDIT THE POLICY'S TRAINING DATA.
  return pose;
}

/*
 * GRAVITY COMPENSATION TARGET POSE TABLE 
 
  Joint                    q_des          q_target
  -------------------------------------------------
  left_hip_pitch       -0.4261674583   -0.4462451952
  left_hip_roll        -0.0450714305   -0.0502048484
  left_hip_yaw         -0.1396263391   -0.1383852648
  left_knee_pitch       0.7679448724    0.7768609102
  left_ankle_pitch     -0.3665191531   -0.3632938872
  left_ankle_roll       0.0000000000   -0.0027236028

  right_hip_pitch       0.4261674583    0.4462506146
  right_hip_roll        0.0450714305    0.0502306974
  right_hip_yaw         0.1396263391    0.1383626377
  right_knee_pitch     -0.7679448724   -0.7768565353
  right_ankle_pitch     0.3665191531    0.3632933700
  right_ankle_roll      0.0000000000    0.0027240991

*/

// START POSITION OF ACTION BLENDING FOR WALKING
constexpr VirtualJointPose12Dof make_blend_start_virtual_pose()
{

  VirtualJointPose12Dof pose{};
  
  // ---------- NON RSU JOINT ----------  
  pose.left_hip_pitch = -0.4462451952f;
  pose.right_hip_pitch = 0.4462506146f;

  pose.left_hip_roll = -0.0502048484f;
  pose.right_hip_roll = 0.0502306974f;

  pose.left_hip_yaw = -0.1383852648f;
  pose.right_hip_yaw = 0.1383626377f;

  pose.left_knee_pitch = 0.7768609102f;
  pose.right_knee_pitch = -0.7768565353f;

  // ---------- RSU JOINT ----------  
  pose.left_ankle_pitch = -0.3632938872f;
  pose.right_ankle_pitch = 0.3632938872f;
  pose.left_ankle_roll = 0.0f;
  pose.right_ankle_roll = 0.0f;
  return pose;
}



// Physical actuator pose used while booting or waiting for inference. Ankle
// values are RSU upper/lower motor angles in the actuator frame.
constexpr ActuatorPose12Dof make_hardware_boot_actuator_pose()
{
  ActuatorPose12Dof pose{};

  // ---------- NON RSU JOINT ----------    
  pose.left_hip_pitch = -0.4462451952f;
  pose.right_hip_pitch = 0.4462506146f;

  pose.left_hip_roll = -0.0502048484f;
  pose.right_hip_roll = 0.0502306974f;

  pose.left_hip_yaw = -0.1383852648f;
  pose.right_hip_yaw = 0.1383626377f;

  pose.left_knee_pitch = 0.7768609102f;
  pose.right_knee_pitch = -0.7768565353f;
  
  // ---------- RSU JOINT ----------  
  pose.left_rsu_upper = -0.3632938872f;
  pose.right_rsu_upper = 0.3632938872f;
  pose.left_rsu_lower = 0.3632938872f;
  pose.right_rsu_lower = -0.3632938872f;
  return pose;
}

inline constexpr auto kPolicyDefaultVirtualPose =
  make_policy_default_virtual_pose();

// Walking target blending starts from the policy's default virtual pose.
// Referencing the default pose directly keeps both values identical.
inline constexpr auto kInferenceBlendStartVirtualPose =
  make_blend_start_virtual_pose();

inline constexpr auto kHardwareBootActuatorPose =
  make_hardware_boot_actuator_pose();

}  // namespace roa::common::fixed_pose
