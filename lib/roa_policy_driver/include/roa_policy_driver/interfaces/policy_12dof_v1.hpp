#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <string>

#ifndef ROA_POLICY_DRIVER_SHARE_DIR
#define ROA_POLICY_DRIVER_SHARE_DIR "/usr/local/share/roa_policy_driver"
#endif


namespace roa::policy::iface {

struct Policy12DofV1 {
  static constexpr int kDof = 12;
  static constexpr int kObsDim = 42;  // 3 + 12 + 12 + 3 + 12
  static constexpr int kActDim = 12;
  static constexpr float kActionScale = 0.5f;

  enum Joint : int {
    L_HIP_PITCH = 0,
    R_HIP_PITCH = 1,
    L_HIP_ROLL  = 2,
    R_HIP_ROLL  = 3,
    L_HIP_YAW   = 4,
    R_HIP_YAW   = 5,
    L_KNEE_PITCH= 6,
    R_KNEE_PITCH= 7,
    L_ANKLE_PITCH=8,
    R_ANKLE_PITCH=9,
		L_ANKLE_ROLL=10,
		R_ANKLE_ROLL=11,
  };

  struct Obs {
    std::array<float, 3> cmd{};                 // [vx, vy, yaw_rate]
    std::array<float, kDof> q_rel{};            // q - default_angles
    std::array<float, kDof> qd_rel{};           // qd
    std::array<float, 3> imu_omega_body{};      // gyro body
    std::array<float, kDof> last_action{};      // raw prev action 
  };

  struct Act {
    std::array<float, kDof> action{};           // raw policy output
  };

  // Pack order: cmd(3), q_rel(12), qd_rel(12), imu(3), last_action(12)
  static inline void pack_obs(const Obs& o, float* out36) noexcept {
    float* p = out36;
    std::memcpy(p, o.cmd.data(), sizeof(float) * 3); p += 3;
    std::memcpy(p, o.q_rel.data(), sizeof(float) * kDof); p += kDof;
    std::memcpy(p, o.qd_rel.data(), sizeof(float) * kDof); p += kDof;
    std::memcpy(p, o.imu_omega_body.data(), sizeof(float) * 3); p += 3;
    std::memcpy(p, o.last_action.data(), sizeof(float) * kDof);
  }

  // NEW: unpack (reverse of pack) — for roundtrip test / debug
  static inline void unpack_obs(const float* in36, Obs& o) noexcept {
    const float* p = in36;
    std::memcpy(o.cmd.data(), p, sizeof(float) * 3); p += 3;
    std::memcpy(o.q_rel.data(), p, sizeof(float) * kDof); p += kDof;
    std::memcpy(o.qd_rel.data(), p, sizeof(float) * kDof); p += kDof;
    std::memcpy(o.imu_omega_body.data(), p, sizeof(float) * 3); p += 3;
    std::memcpy(o.last_action.data(), p, sizeof(float) * kDof);
  }

  static inline void unpack_act(const float* in10, Act& a) noexcept {
    std::memcpy(a.action.data(), in10, sizeof(float) * kDof);
  }

  static inline std::string default_model_path() {
    return std::string(ROA_POLICY_DRIVER_SHARE_DIR) + "/onnx/12dof/policy.onnx";
  }

  static inline void pack_act(const Act& a, float* out10) noexcept {
    std::memcpy(out10, a.action.data(), sizeof(float) * kDof);
  }

  static inline std::array<float, kActDim> action_to_q_target(
      const std::array<float, kActDim>& raw_action,
      const std::array<float, kActDim>& default_angles,
      float action_scale) noexcept
  {
    std::array<float, kActDim> q_target{};
    for (int i = 0; i < kActDim; ++i) {
      q_target[i] = default_angles[i] + action_scale * raw_action[i];
    }
    return q_target;
  }

  static inline std::array<float, kActDim> q_target_to_action(
      const std::array<float, kActDim>& q_target,
      const std::array<float, kActDim>& default_angles,
      float action_scale) noexcept
  {
    std::array<float, kActDim> action{};
    for (int i = 0; i < kActDim; ++i) {
      action[i] = (q_target[i] - default_angles[i]) / action_scale;
    }
    return action;
  }
};

}  // namespace roa::policy::iface