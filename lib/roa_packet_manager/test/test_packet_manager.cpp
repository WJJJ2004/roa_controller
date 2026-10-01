#include <array>
#include <chrono>
#include <cstdint>
#include <memory>
#include <set>
#include <thread>

#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time.hpp"
#include "roa_packet_manager/packet_manager.hpp"

namespace
{

using roa_packet_manager::PacketManager;

const roa_interfaces::msg::MotorCommand* find_command(
  const roa_interfaces::msg::MotorCommandArray& message,
  std::uint16_t motor_id)
{
  for (const auto& command : message.commands) {
    if (command.motor_id == motor_id) {
      return &command;
    }
  }
  return nullptr;
}

TEST(PacketManager, BuildsComplete23MotorPacket)
{
  const auto message = PacketManager::build(
    PacketManager::Command12Dof{}, rclcpp::Time(123, 456), "test");

  EXPECT_EQ(message.commands.size(), PacketManager::kMotorCount);
  EXPECT_EQ(message.header.frame_id, "test");

  std::set<std::uint16_t> ids;
  for (const auto& command : message.commands) {
    ids.insert(command.motor_id);
  }

  EXPECT_EQ(ids.size(), PacketManager::kMotorCount);
  for (std::uint16_t id = 0; id <= 23; ++id) {
    EXPECT_EQ(ids.count(id), id == 8 ? 0U : 1U) << "motor_id=" << id;
  }
}

TEST(PacketManager, HoldsUpperBodyAtZeroWithTorsoGains)
{
  const auto message = PacketManager::build(
    PacketManager::Command12Dof{}, rclcpp::Time(0, 0), "test");
  constexpr std::array<std::uint16_t, 11> upper_body_ids{
    0, 1, 2, 3, 4, 5, 6, 7, 9, 22, 23};

  for (const auto id : upper_body_ids) {
    const auto* command = find_command(message, id);
    ASSERT_NE(command, nullptr) << "motor_id=" << id;
    EXPECT_FLOAT_EQ(command->torque, 0.0f) << "motor_id=" << id;
    EXPECT_FLOAT_EQ(command->position, 0.0f) << "motor_id=" << id;
    EXPECT_FLOAT_EQ(command->velocity, 0.0f) << "motor_id=" << id;
    EXPECT_FLOAT_EQ(command->kp, 50.0f) << "motor_id=" << id;
    EXPECT_FLOAT_EQ(command->kd, 2.0f) << "motor_id=" << id;
  }
}

TEST(PacketManager, PreservesControlledJointTargetsAndRsuGains)
{
  PacketManager::Command12Dof input;
  input.left_hip_pitch = 0.10f;
  input.right_hip_pitch = 0.11f;
  input.left_hip_roll = 0.12f;
  input.right_hip_roll = 0.13f;
  input.left_hip_yaw = 0.14f;
  input.right_hip_yaw = 0.15f;
  input.left_knee_pitch = 0.16f;
  input.right_knee_pitch = 0.17f;
  input.left_rsu_upper = 0.18f;
  input.right_rsu_upper = 0.19f;
  input.left_rsu_lower = 0.20f;
  input.right_rsu_lower = 0.21f;
  input.left_rsu_upper_kp = 18.1f;
  input.left_rsu_upper_kd = 1.81f;

  const auto message = PacketManager::build(input, rclcpp::Time(0, 0), "test");

  ASSERT_NE(find_command(message, 10), nullptr);
  EXPECT_FLOAT_EQ(find_command(message, 10)->position, input.left_hip_pitch);
  ASSERT_NE(find_command(message, 17), nullptr);
  EXPECT_FLOAT_EQ(find_command(message, 17)->position, input.right_knee_pitch);
  ASSERT_NE(find_command(message, 18), nullptr);
  EXPECT_FLOAT_EQ(find_command(message, 18)->position, input.left_rsu_upper);
  EXPECT_FLOAT_EQ(find_command(message, 18)->kp, input.left_rsu_upper_kp);
  EXPECT_FLOAT_EQ(find_command(message, 18)->kd, input.left_rsu_upper_kd);
  ASSERT_NE(find_command(message, 21), nullptr);
  EXPECT_FLOAT_EQ(find_command(message, 21)->position, input.right_rsu_lower);
}

TEST(PacketManager, StateDecoderIgnoresUpperBodyMotors)
{
  roa_interfaces::msg::MotorStateArray states;
  for (std::uint16_t id = 9; id <= 21; ++id) {
    roa_interfaces::msg::MotorState state;
    state.motor_id = id;
    state.position = static_cast<float>(id);
    state.velocity = static_cast<float>(id) + 0.1f;
    state.current = static_cast<float>(id) + 0.2f;
    states.states.push_back(state);
  }
  roa_interfaces::msg::MotorState upper_body;
  upper_body.motor_id = 22;
  upper_body.position = 1.0f;
  upper_body.velocity = 2.0f;
  upper_body.current = 3.0f;
  states.states.push_back(upper_body);

  PacketManager::HardwareState decoded;
  std::string error;
  ASSERT_TRUE(PacketManager::decode_motor_state(states, decoded, &error)) << error;
  EXPECT_FLOAT_EQ(decoded.torso_yaw.position, 9.0f);
  EXPECT_FLOAT_EQ(decoded.right_rsu_lower.position, 21.0f);
}

TEST(PacketManager, PublishesCompletePacketThroughRos)
{
  int argc = 0;
  char** argv = nullptr;
  rclcpp::init(argc, argv);

  auto publisher_node = std::make_shared<rclcpp::Node>("packet_manager_test_publisher");
  auto subscriber_node = std::make_shared<rclcpp::Node>("packet_manager_test_subscriber");
  auto publisher = publisher_node->create_publisher<
    roa_interfaces::msg::MotorCommandArray>("/packet_manager_test/command", 10);

  roa_interfaces::msg::MotorCommandArray::SharedPtr received;
  auto subscription = subscriber_node->create_subscription<
    roa_interfaces::msg::MotorCommandArray>(
    "/packet_manager_test/command", 10,
    [&received](roa_interfaces::msg::MotorCommandArray::SharedPtr message) {
      received = std::move(message);
    });

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(publisher_node);
  executor.add_node(subscriber_node);

  const auto message = PacketManager::build(
    PacketManager::Command12Dof{}, publisher_node->now(), "dds_test");
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
  while (!received && std::chrono::steady_clock::now() < deadline) {
    publisher->publish(message);
    executor.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  executor.remove_node(subscriber_node);
  executor.remove_node(publisher_node);
  subscription.reset();
  publisher.reset();
  subscriber_node.reset();
  publisher_node.reset();
  rclcpp::shutdown();

  ASSERT_NE(received, nullptr);
  EXPECT_EQ(received->header.frame_id, "dds_test");
  EXPECT_EQ(received->commands.size(), PacketManager::kMotorCount);
  const auto* upper_body_command = find_command(*received, 23);
  ASSERT_NE(upper_body_command, nullptr);
  EXPECT_FLOAT_EQ(upper_body_command->position, 0.0f);
  EXPECT_FLOAT_EQ(upper_body_command->kp, 50.0f);
  EXPECT_FLOAT_EQ(upper_body_command->kd, 2.0f);
}

}  // namespace
