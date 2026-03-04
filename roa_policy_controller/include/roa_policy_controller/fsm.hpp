#pragma once
#include <cstdint>

namespace roa_policy_controller {

enum class FsmState : uint8_t { BOOT=0, RUN=1, SAFE_HOLD=2 };

struct FsmConfig {
  uint32_t max_fail_count = 5;          // 연속 실패 누적 임계
  uint32_t recover_ok_count = 50;       // 정상 입력 회복 카운트 (예: 500Hz 기준 0.1s)
};

class Fsm {
public:
  explicit Fsm(FsmConfig cfg) : cfg_(cfg) {}

  void reset() {
    state_ = FsmState::BOOT;
    fail_count_ = 0;
    recover_count_ = 0;
  }

  FsmState state() const { return state_; }

  // update()에서 매 tick 호출: input_ok / nan_guard_ok / inference_ok 같은 결과를 반영
  void step(bool ok) {
    switch (state_) {
      case FsmState::BOOT:
        // BOOT는 on_configure에서 바로 RUN으로 넘기는 것이 보통이지만, 안전하게 1회 ok를 요구할 수도 있음.
        state_ = FsmState::RUN;
        fail_count_ = 0;
        recover_count_ = 0;
        break;

      case FsmState::RUN:
        if (ok) {
          fail_count_ = 0;
        } else {
          if (++fail_count_ >= cfg_.max_fail_count) {
            state_ = FsmState::SAFE_HOLD;
            recover_count_ = 0;
          }
        }
        break;

      case FsmState::SAFE_HOLD:
        if (ok) {
          if (++recover_count_ >= cfg_.recover_ok_count) {
            state_ = FsmState::RUN;
            fail_count_ = 0;
            recover_count_ = 0;
          }
        } else {
          recover_count_ = 0;
        }
        break;
    }
  }

private:
  FsmConfig cfg_;
  FsmState state_{FsmState::BOOT};
  uint32_t fail_count_{0};
  uint32_t recover_count_{0};
};

}  // namespace roa_policy_controller