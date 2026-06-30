#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "roa_interfaces/msg/motor_command.hpp"
#include "roa_interfaces/msg/motor_command_array.hpp"
#include "roa_interfaces/msg/motor_state_array.hpp"

namespace roa_packet_manager
{

class PacketManager
{
public:
  static constexpr float kp_scale = 1.0f;
  static constexpr float kd_scale = 1.0f;

  struct Command12Dof
  {
    float left_hip_pitch   = 0.0f;
    float right_hip_pitch  = 0.0f;
    float left_hip_roll    = 0.0f;
    float right_hip_roll   = 0.0f;
    float left_hip_yaw     = 0.0f;
    float right_hip_yaw    = 0.0f;
    float left_knee_pitch  = 0.0f;
    float right_knee_pitch = 0.0f;
    float left_rsu_upper   = 0.0f;
    float right_rsu_upper  = 0.0f;
    float left_rsu_lower   = 0.0f;
    float right_rsu_lower  = 0.0f;

    float left_rsu_upper_kp  = 15.75f;
    float right_rsu_upper_kp = 15.75f;
    float left_rsu_lower_kp  = 15.75f;
    float right_rsu_lower_kp = 15.75f;

    float left_rsu_upper_kd  = 2.5f;
    float right_rsu_upper_kd = 2.5f;
    float left_rsu_lower_kd  = 2.5f;
    float right_rsu_lower_kd = 2.5f;
  };

  struct JointMeta
  {
    const char* name;
    uint16_t motor_id;
    float kp;
    float kd;
  };

  struct MotorSample
  {
    float position = 0.0f;
    float velocity = 0.0f;
    float current = 0.0f;
    bool valid = false;
  };

  struct HardwareState
  {
    MotorSample torso_yaw;
    MotorSample left_hip_pitch;
    MotorSample right_hip_pitch;
    MotorSample left_hip_roll;
    MotorSample right_hip_roll;
    MotorSample left_hip_yaw;
    MotorSample right_hip_yaw;
    MotorSample left_knee_pitch;
    MotorSample right_knee_pitch;
    MotorSample left_rsu_upper;
    MotorSample right_rsu_upper;
    MotorSample left_rsu_lower;
    MotorSample right_rsu_lower;
  };

  static constexpr std::size_t kMotorCount = 13;
  static constexpr int kMinMotorId = 9;
  static constexpr int kMaxMotorId = 21;

  static const std::array<JointMeta, kMotorCount> kJointMetaTable;

  static bool valid_motor_cmd(const Command12Dof& cmd);
  static constexpr int motor_id_to_slot(int motor_id)
  {
    return (motor_id >= kMinMotorId && motor_id <= kMaxMotorId)
      ? (motor_id - kMinMotorId)
      : -1;
  }

  static const char* motor_id_to_name(int motor_id);

  static bool decode_motor_state(
    const roa_interfaces::msg::MotorStateArray& msg,
    HardwareState& out,
    std::string* error = nullptr);

  static roa_interfaces::msg::MotorCommandArray build(
    const Command12Dof& cmd,
    const rclcpp::Time& stamp,
    const std::string& frame_id = "");

private:
  static roa_interfaces::msg::MotorCommand make_command(
    uint16_t motor_id,
    float position,
    float kp,
    float kd);
};

}  // namespace roa_packet_manager