#include <algorithm>
#include <gtest/gtest.h>

#include <cmath>

#include "rsu_manager_v2/rsu_lut.hpp"
#include "rsu_manager_v2/rsu_model.hpp"

using rsu_manager_v2::ImpedanceMapper;
using rsu_manager_v2::RsuLut;
using rsu_manager_v2::StateEstimator;

TEST(RsuLut, BoundsAndNeutral)
{
  RsuLut lut;
  lut.load(RSU_V2_TEST_LUT);
  EXPECT_TRUE(lut.query(0.0, 0.0).valid);
  EXPECT_TRUE(lut.query(lut.roll_min(), lut.pitch_min()).valid);
  EXPECT_TRUE(lut.query(lut.roll_max(), lut.pitch_max()).valid);
  EXPECT_FALSE(lut.query(lut.roll_min() - 1e-6, 0.0).valid);
  const auto neutral = lut.query(0.0, 0.0);
  EXPECT_NEAR(neutral.alpha[0], 0.0, 1e-7);
  EXPECT_NEAR(neutral.alpha[1], 0.0, 1e-7);
}

TEST(RsuModel, NeutralStateAndImpedance)
{
  RsuLut lut;
  lut.load(RSU_V2_TEST_LUT);
  StateEstimator estimator(lut);
  estimator.reset({0.0, 0.0}, {0.0, 0.0});
  const auto state = estimator.update({0.0, 0.0}, {0.0, 0.0}, 1.0 / 200.0);
  ASSERT_TRUE(state.valid);
  EXPECT_NEAR(state.q[0], 0.0, 1e-7);
  EXPECT_NEAR(state.q[1], 0.0, 1e-7);
  ImpedanceMapper mapper;
  const auto impedance = mapper.compute(state.jacobian, state.valid);
  ASSERT_TRUE(impedance.valid);
  EXPECT_GT(impedance.kp[0], 5.0);
  EXPECT_LT(impedance.kp[0], 25.0);
  EXPECT_GT(impedance.kd[1], 0.2);
  EXPECT_LT(impedance.kd[1], 6.0);
}

TEST(RsuModel, VelocityTracksJacobianImmediatelyWithObservationClip)
{
  RsuLut lut;
  lut.load(RSU_V2_TEST_LUT);
  const std::array<double, 2> q{0.0, -0.4};
  const auto query = lut.query(q[0], q[1]);
  ASSERT_TRUE(query.valid);
  StateEstimator estimator(lut);
  estimator.reset(q, query.alpha);
  // In-range velocities pass immediately; out-of-range velocities saturate
  // at the original observation bounds, without filter history.
  for (const auto & expected : {std::array<double, 2>{2.0, -3.0},
                               std::array<double, 2>{8.0, -7.0},
                               std::array<double, 2>{-8.0, 7.0},
                               std::array<double, 2>{0.0, 0.0}}) {
    const std::array<double, 2> motor_velocity{
      query.jacobian[0] * expected[0] + query.jacobian[1] * expected[1],
      query.jacobian[2] * expected[0] + query.jacobian[3] * expected[1]};
    const auto state = estimator.update(query.alpha, motor_velocity, 0.005);
    ASSERT_TRUE(state.valid);
    EXPECT_NEAR(state.qd[0], std::clamp(expected[0], -5.235987756, 5.235987756), 1e-4);
    EXPECT_NEAR(state.qd[1], std::clamp(expected[1], -5.235987756, 5.235987756), 1e-4);
  }
}

TEST(RsuModel, PositionDifferenceDoesNotContaminateMeasuredZeroVelocity)
{
  RsuLut lut;
  lut.load(RSU_V2_TEST_LUT);
  StateEstimator estimator(lut);
  const std::array<double, 2> initial{0.0, -0.4};
  estimator.reset(initial, lut.query(initial[0], initial[1]).alpha);
  const auto shifted = lut.query(0.01, -0.39);
  ASSERT_TRUE(shifted.valid);
  const auto state = estimator.update(shifted.alpha, {0.0, 0.0}, 0.005);
  ASSERT_TRUE(state.valid);
  EXPECT_NEAR(state.q[0], 0.01, 1e-4);
  EXPECT_NEAR(state.q[1], -0.39, 1e-4);
  EXPECT_DOUBLE_EQ(state.qd[0], 0.0);
  EXPECT_DOUBLE_EQ(state.qd[1], 0.0);
}
