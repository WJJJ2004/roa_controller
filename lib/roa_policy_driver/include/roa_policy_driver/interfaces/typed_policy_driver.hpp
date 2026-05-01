#pragma once
#include <array>
#include <string>
#include <type_traits>

#include <roa_policy_driver/policy_driver.hpp>

namespace roa::policy {

// Spec 컨셉 체크(컴파일 타임)
template <class Spec>
struct is_valid_spec {
  static constexpr bool value =
      (Spec::kObsDim > 0) && (Spec::kActDim > 0);
};

template <class Spec>
class TypedPolicyDriver {
  static_assert(is_valid_spec<Spec>::value, "Spec must define kObsDim/kActDim > 0");

public:
  using Obs = typename Spec::Obs;
  using Act = typename Spec::Act;

  bool load(const std::string& model_path, const Options& opt) {
    if (!core_.load(model_path, opt)) return false;

    // 차원 강제 (interface와 모델 mismatch 방지)
    if (core_.input_dim() != Spec::kObsDim) return false;
    if (core_.output_dim() != Spec::kActDim) return false;
    return true;
  }

  bool is_loaded() const noexcept { return core_.is_loaded(); }

  // RT loop용: 성공 경로에서 동적할당 없음
  bool run(const Obs& obs, Act& act) noexcept {
    Spec::pack_obs(obs, obs_buf_.data());

    const bool ok = core_.run(
        obs_buf_.data(), Spec::kObsDim,
        act_buf_.data(), Spec::kActDim);

    if (!ok) return false;

    Spec::unpack_act(act_buf_.data(), act);
    return true;
  }

  // 필요하면 core의 info 노출
  int input_dim() const noexcept { return core_.input_dim(); }
  int output_dim() const noexcept { return core_.output_dim(); }
  const std::string& input_name() const noexcept { return core_.input_name(); }
  const std::string& output_name() const noexcept { return core_.output_name(); }

private:
  PolicyDriver core_;

  // RT-safe 고정 버퍼 (stack이 아니라 멤버라서 크기도 고정)
  std::array<float, Spec::kObsDim> obs_buf_{};
  std::array<float, Spec::kActDim> act_buf_{};
};

}  // namespace roa::policy