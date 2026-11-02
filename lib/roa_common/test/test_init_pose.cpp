#include <iostream>

#include <roa_common/pose.hpp>

int main()
{
  using roa_packet_manager::PacketManager;
  const auto init = PacketManager::build(
    roa::common::make_init_pose(), rclcpp::Time(0, 0), "INIT POS HOLD");
  const auto baseline = PacketManager::build(
    PacketManager::Command12Dof{}, rclcpp::Time(0, 0), "baseline");
  if (init.commands.size() != baseline.commands.size()) {return 1;}
  int ankles = 0;
  for (std::size_t i = 0; i < init.commands.size(); ++i) {
    const auto & cmd = init.commands[i];
    const auto & original = baseline.commands[i];
    if (cmd.motor_id != original.motor_id) {return 1;}
    const bool ankle = cmd.motor_id >= 18 && cmd.motor_id <= 21;
    const float scale = ankle ? 2.0f : 1.0f;
    if (cmd.kp != scale * original.kp || cmd.kd != scale * original.kd) {
      std::cerr << "Unexpected boot gain at motor " << cmd.motor_id << '\n';
      return 1;
    }
    if (ankle) {
      ++ankles;
      if (cmd.kp != 31.5f || cmd.kd != 5.0f) {return 1;}
    }
  }
  return ankles == 4 ? 0 : 1;
}
