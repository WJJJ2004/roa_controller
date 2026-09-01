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
  const auto state = estimator.update({0.0, 0.0}, {0.0, 0.0}, 1.0 / 300.0);
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
