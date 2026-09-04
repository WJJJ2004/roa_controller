#include "roa_packet_manager/packet_manager.hpp"

#include <cmath>
#include <stdexcept>

namespace roa_packet_manager
{

const std::array<PacketManager::JointMeta, PacketManager::kMotorCount>
PacketManager::kJointMetaTable{{
  {"torso_yaw",          9,   75.0f,   2.5f},
  {"left_hip_pitch",    10,  150.0f,  24.722f},
  {"right_hip_pitch",   11,  150.0f,  24.722f},
  {"left_hip_roll",     12,  200.0f,  26.387f},
  {"right_hip_roll",    13,  200.0f,  26.387f},
  {"left_hip_yaw",      14,  100.0f,   3.419f},
  {"right_hip_yaw",     15,  100.0f,   3.419f},
  {"left_knee_pitch",   16,  150.0f,   8.654f},
  {"right_knee_pitch",  17,  150.0f,   8.654f},
  {"left_rsu_upper",    18,  15.75f,  2.5f},
  {"right_rsu_upper",   19,  15.75f,  2.5f},
  {"left_rsu_lower",    20,  15.75f,  2.5f},
  {"right_rsu_lower",   21,  15.75f,  2.5f},
}};

bool PacketManager::valid_motor_cmd(const Command12Dof& cmd)
{
  const auto valid_target = [](float q) {
    return std::isfinite(q) && q >= -3.142f && q <= 3.142f;
  };

  const auto valid_gain = [](float kp, float kd) {
    return std::isfinite(kp) && std::isfinite(kd) &&
           kp > 0.0f && kd > 0.0f &&
           kp < 1000.0f && kd < 100.0f;
  };

  return
    valid_target(cmd.left_hip_pitch) &&
    valid_target(cmd.right_hip_pitch) &&
    valid_target(cmd.left_hip_roll) &&
    valid_target(cmd.right_hip_roll) &&
    valid_target(cmd.left_hip_yaw) &&
    valid_target(cmd.right_hip_yaw) &&
    valid_target(cmd.left_knee_pitch) &&
    valid_target(cmd.right_knee_pitch) &&
    valid_target(cmd.left_rsu_upper) &&
    valid_target(cmd.right_rsu_upper) &&
    valid_target(cmd.left_rsu_lower) &&
    valid_target(cmd.right_rsu_lower) &&
    valid_gain(cmd.left_rsu_upper_kp, cmd.left_rsu_upper_kd) &&
    valid_gain(cmd.right_rsu_upper_kp, cmd.right_rsu_upper_kd) &&
    valid_gain(cmd.left_rsu_lower_kp, cmd.left_rsu_lower_kd) &&
    valid_gain(cmd.right_rsu_lower_kp, cmd.right_rsu_lower_kd);
}

const char* PacketManager::motor_id_to_name(int motor_id)
{
  const int slot = motor_id_to_slot(motor_id);
  return slot >= 0
    ? kJointMetaTable[static_cast<std::size_t>(slot)].name
    : "invalid_motor_id";
}

bool PacketManager::decode_motor_state(
  const roa_interfaces::msg::MotorStateArray& msg,
  HardwareState& out,
  std::string* error)
{
  std::array<MotorSample, kMotorCount> samples{};

  for (const auto& state : msg.states) {
    const int slot = motor_id_to_slot(static_cast<int>(state.motor_id));
    if (slot < 0) {
      continue;
    }

    if (!std::isfinite(state.position) ||
        !std::isfinite(state.velocity) ||
        !std::isfinite(state.current)) {
      if (error != nullptr) {
        *error = std::string("Non-finite motor state: ") +
                 motor_id_to_name(state.motor_id);
      }
      return false;
    }

    auto& sample = samples[static_cast<std::size_t>(slot)];
    sample.position = state.position;
    sample.velocity = state.velocity;
    sample.current = state.current;
    sample.valid = true;
  }

  const auto require = [&samples](int motor_id) -> const MotorSample* {
    const int slot = motor_id_to_slot(motor_id);
    if (slot < 0) {
      return nullptr;
    }

    const auto& sample = samples[static_cast<std::size_t>(slot)];
    return sample.valid ? &sample : nullptr;
  };

  const auto* torso_yaw = require(9);
  const auto* left_hip_pitch = require(10);
  const auto* right_hip_pitch = require(11);
  const auto* left_hip_roll = require(12);
  const auto* right_hip_roll = require(13);
  const auto* left_hip_yaw = require(14);
  const auto* right_hip_yaw = require(15);
  const auto* left_knee_pitch = require(16);
  const auto* right_knee_pitch = require(17);
  const auto* left_rsu_upper = require(18);
  const auto* right_rsu_upper = require(19);
  const auto* left_rsu_lower = require(20);
  const auto* right_rsu_lower = require(21);

  if (!torso_yaw || !left_hip_pitch || !right_hip_pitch ||
      !left_hip_roll || !right_hip_roll ||
      !left_hip_yaw || !right_hip_yaw ||
      !left_knee_pitch || !right_knee_pitch ||
      !left_rsu_upper || !right_rsu_upper ||
      !left_rsu_lower || !right_rsu_lower) {
    if (error != nullptr) {
      *error = "Required motor states missing";
    }
    return false;
  }

  out.torso_yaw = *torso_yaw;
  out.left_hip_pitch = *left_hip_pitch;
  out.right_hip_pitch = *right_hip_pitch;
  out.left_hip_roll = *left_hip_roll;
  out.right_hip_roll = *right_hip_roll;
  out.left_hip_yaw = *left_hip_yaw;
  out.right_hip_yaw = *right_hip_yaw;
  out.left_knee_pitch = *left_knee_pitch;
  out.right_knee_pitch = *right_knee_pitch;
  out.left_rsu_upper = *left_rsu_upper;
  out.right_rsu_upper = *right_rsu_upper;
  out.left_rsu_lower = *left_rsu_lower;
  out.right_rsu_lower = *right_rsu_lower;

  return true;
}

roa_interfaces::msg::MotorCommandArray PacketManager::build(
  const Command12Dof& cmd,
  const rclcpp::Time& stamp,
  const std::string& frame_id)
{
  if (!valid_motor_cmd(cmd)) {
    throw std::invalid_argument("Invalid motor command");
  }

  roa_interfaces::msg::MotorCommandArray msg;
  msg.header.stamp = stamp;
  msg.header.frame_id = frame_id;
  msg.commands.reserve(kMotorCount);

  msg.commands.push_back(make_command(
    kJointMetaTable[0].motor_id,
    0.0f,
    kp_scale * kJointMetaTable[0].kp,
    kd_scale * kJointMetaTable[0].kd));

  msg.commands.push_back(make_command(kJointMetaTable[1].motor_id, cmd.left_hip_pitch, kp_scale * kJointMetaTable[1].kp, kd_scale * kJointMetaTable[1].kd));
  msg.commands.push_back(make_command(kJointMetaTable[2].motor_id, cmd.right_hip_pitch, kp_scale * kJointMetaTable[2].kp, kd_scale * kJointMetaTable[2].kd));
  msg.commands.push_back(make_command(kJointMetaTable[3].motor_id, cmd.left_hip_roll, kp_scale * kJointMetaTable[3].kp, kd_scale * kJointMetaTable[3].kd));
  msg.commands.push_back(make_command(kJointMetaTable[4].motor_id, cmd.right_hip_roll, kp_scale * kJointMetaTable[4].kp, kd_scale * kJointMetaTable[4].kd));
  msg.commands.push_back(make_command(kJointMetaTable[5].motor_id, cmd.left_hip_yaw, kp_scale * kJointMetaTable[5].kp, kd_scale * kJointMetaTable[5].kd));
  msg.commands.push_back(make_command(kJointMetaTable[6].motor_id, cmd.right_hip_yaw, kp_scale * kJointMetaTable[6].kp, kd_scale * kJointMetaTable[6].kd));
  msg.commands.push_back(make_command(kJointMetaTable[7].motor_id, cmd.left_knee_pitch, kp_scale * kJointMetaTable[7].kp, kd_scale * kJointMetaTable[7].kd));
  msg.commands.push_back(make_command(kJointMetaTable[8].motor_id, cmd.right_knee_pitch, kp_scale * kJointMetaTable[8].kp, kd_scale * kJointMetaTable[8].kd));
  msg.commands.push_back(make_command(kJointMetaTable[9].motor_id, cmd.left_rsu_upper, cmd.left_rsu_upper_kp, cmd.left_rsu_upper_kd));
  msg.commands.push_back(make_command(kJointMetaTable[10].motor_id, cmd.right_rsu_upper, cmd.right_rsu_upper_kp, cmd.right_rsu_upper_kd));
  msg.commands.push_back(make_command(kJointMetaTable[11].motor_id, cmd.left_rsu_lower, cmd.left_rsu_lower_kp, cmd.left_rsu_lower_kd));
  msg.commands.push_back(make_command(kJointMetaTable[12].motor_id, cmd.right_rsu_lower, cmd.right_rsu_lower_kp, cmd.right_rsu_lower_kd));

  return msg;
}

roa_interfaces::msg::MotorCommand PacketManager::make_command(
  uint16_t motor_id,
  float position,
  float kp,
  float kd)
{
  roa_interfaces::msg::MotorCommand cmd;
  cmd.motor_id = motor_id;
  cmd.torque = 0.0f;
  cmd.position = position;
  cmd.velocity = 0.0f;
  cmd.kp = kp;
  cmd.kd = kd;
  return cmd;
}

}  // namespace roa_packet_manager