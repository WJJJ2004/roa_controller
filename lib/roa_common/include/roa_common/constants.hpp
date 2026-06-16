#pragma once

namespace roa::constants {

// NOTE:
// 하드웨어 제어 기준 초기 자세
// 실제 관절 좌표계 기준 초기 자세 -> 하드웨어 제어시 초기 pose로 사용됨
// 해당 값은 controller와 system manager에서 초기 pose를 잡을 때 사용함
// rsu manager의 튜닝 스크립트로 수동으로 값을 찾아 튜닝할 수 있다.
constexpr float HIP_INIT_POS   = 0.5457f;
constexpr float KNEE_INIT_POS  = 0.8034f;
constexpr float ANKLE_INIT_POS = 0.4238f;

// NOTE:
// 가성 관절 좌표계 기준 초기 자세 
// 하드웨어 제어시 초기 inference에서 q blend를 위해 사용되는 가상 초기 자세를 지정할때 사용함
// 이때 Init pos와 비교했을 때, HIP과 KNEE는 동일하게, Ankle은 서로 RSU solver의 파라메터와 해의 관계를 가진다.
constexpr float HIP_INIT_INF   = 0.5457f;
constexpr float KNEE_INIT_INF  = 0.8034f;
constexpr float ANKLE_INIT_INF = 0.4238f;

// NOTE:
// 추론에 사용되는 q는 모두 상대 각도이고 이때 default angle은 상대각도(obs.q_rel)의 기준점이자, policy의 출력(act)이 매핑되는 절대각도 기준점임.
// 학습 단계에서 adult.py 스크립트와 동일하게 유지하되, 학습 단계에서 해당 파라메터를 수정시 제어기에서도 동일하게 수정이 요구됨
constexpr float HIP_PITCH_DEF   = 0.349066f; // 20 degree
constexpr float KNEE_PITCH_DEF  = 0.872665f;	// 50 degree
constexpr float ANKLE_PITCH_DEF = 0.523599f; // 30 degree

}
