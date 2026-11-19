#include <gtest/gtest.h>
#include <limits>
#include "node/policy_command.hpp"

using roa_main_controller::policy_command;

TEST(PolicyCommand, MovingThenStaleThenRecoveredThenNeutral)
{
  geometry_msgs::msg::Twist msg;
  msg.linear.x = 0.4;
  msg.linear.y = -0.2;
  msg.angular.z = 0.6;
  const std::array<float, 3> moving{0.4f, -0.2f, 0.6f};
  const std::array<float, 3> zero{};
  EXPECT_EQ(policy_command(&msg, true), moving);
  EXPECT_EQ(policy_command(&msg, false), zero);
  EXPECT_EQ(policy_command(nullptr, false), zero);
  EXPECT_EQ(policy_command(&msg, true), moving);
  msg = geometry_msgs::msg::Twist{};
  EXPECT_EQ(policy_command(&msg, true), zero);
  EXPECT_EQ(policy_command(&msg, false), zero);
}

TEST(PolicyCommand, InvalidComponentZerosWholeCommand)
{
  const std::array<float, 3> zero{};
  for (const double value : {
      std::numeric_limits<double>::quiet_NaN(),
      std::numeric_limits<double>::infinity(),
      -std::numeric_limits<double>::infinity(),
      std::numeric_limits<double>::max()})
  {
    for (int axis = 0; axis < 3; ++axis) {
      geometry_msgs::msg::Twist msg;
      msg.linear.x = 0.4;
      msg.linear.y = -0.2;
      msg.angular.z = 0.6;
      if (axis == 0) {msg.linear.x = value;}
      if (axis == 1) {msg.linear.y = value;}
      if (axis == 2) {msg.angular.z = value;}
      EXPECT_EQ(policy_command(&msg, true), zero);
    }
  }
}
