#include <roa_common/joint_limits.hpp>

#include <array>
#include <cassert>
#include <cmath>
#include <limits>

namespace
{

struct TestPolicy
{
  static constexpr int kActDim = 12;
  static constexpr int kDof = 12;

  enum Joint
  {
    L_HIP_PITCH = 0,
    R_HIP_PITCH = 1,
    L_HIP_ROLL = 2,
    R_HIP_ROLL = 3,
    L_HIP_YAW = 4,
    R_HIP_YAW = 5,
    L_KNEE_PITCH = 6,
    R_KNEE_PITCH = 7,
    L_ANKLE_PITCH = 8,
    R_ANKLE_PITCH = 9,
    L_ANKLE_ROLL = 10,
    R_ANKLE_ROLL = 11,
  };
};

}  // namespace

int main()
{
  using roa::common::joint_limits::clip_policy_joint_target;

  std::array<float, TestPolicy::kActDim> target{};
  target[TestPolicy::L_HIP_ROLL] = -1.0f;
  target[TestPolicy::R_HIP_ROLL] = 1.0f;
  target[TestPolicy::L_ANKLE_PITCH] = -1.0f;
  target[TestPolicy::R_ANKLE_ROLL] = 1.0f;

  const auto clipped = clip_policy_joint_target<TestPolicy>(target);
  assert(clipped.valid);
  assert(clipped.clipped_count == 4);
  assert(target[TestPolicy::L_HIP_ROLL] == -0.150000f);
  assert(target[TestPolicy::R_HIP_ROLL] == 0.150000f);
  assert(target[TestPolicy::L_ANKLE_PITCH] == -0.628319f);
  assert(target[TestPolicy::R_ANKLE_ROLL] == 0.314159f);

  const auto before_invalid = target;
  target[TestPolicy::L_HIP_PITCH] = std::numeric_limits<float>::quiet_NaN();
  const auto invalid_input = target;
  const auto invalid = clip_policy_joint_target<TestPolicy>(target);
  assert(!invalid.valid);
  assert(invalid.clipped_count == 0);
  assert(std::isnan(target[TestPolicy::L_HIP_PITCH]));
  for (std::size_t i = 1; i < target.size(); ++i) {
    assert(target[i] == invalid_input[i]);
  }
  (void)before_invalid;

  return 0;
}
