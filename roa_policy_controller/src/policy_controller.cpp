#include "roa_policy_controller/policy_controller.hpp"

#include <string>

namespace roa_policy_controller {

RoaPolicyController::RoaPolicyController()
: controller_interface::ControllerInterface() {}

controller_interface::CallbackReturn RoaPolicyController::on_init() {
  // 파라미터 선언(예: rsu timeout, fail count 등) — update()에서 읽지 말고 configure에서 캐시 추천
  auto_declare<int>("rsu.timeout_ms", 30);
  auto_declare<int>("fsm.max_fail_count", 5);
  auto_declare<int>("fsm.recover_ok_count", 50);
  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn RoaPolicyController::on_configure(
  const rclcpp_lifecycle::State&) {

  // fsm cfg 반영
  FsmConfig fcfg;
  fcfg.max_fail_count = static_cast<uint32_t>(get_node()->get_parameter("fsm.max_fail_count").as_int());
  fcfg.recover_ok_count = static_cast<uint32_t>(get_node()->get_parameter("fsm.recover_ok_count").as_int());
  fsm_ = Fsm(fcfg);
  fsm_.reset();

  // rsu latch cfg
  RsuUsableConfig rcfg;
  rcfg.timeout_ns = static_cast<uint64_t>(get_node()->get_parameter("rsu.timeout_ms").as_int()) * 1'000'000ull;
  rsu_latch_ = RsuLatch(rcfg);

  // RSU solution sub (QoS: keep_last=1, best_effort)
  rclcpp::QoS qos(rclcpp::KeepLast(1));
  qos.best_effort();

  sub_rsu_solution_ = get_node()->create_subscription<roa_interfaces::msg::RsuSolution>(
    "/rsu/solution", qos,
    [this](const roa_interfaces::msg::RsuSolution::SharedPtr msg) {
      // callback은 non-RT thread
      rsu_latch_.write(*msg);
    });

  // RSU target pub (RT-safe publisher)
  rt_pub_rsu_target_ =
    std::make_shared<realtime_tools::RealtimePublisher<roa_interfaces::msg::RsuTarget>>(
      get_node(), "/rsu/target", qos);

  // 안전 커맨드 초기화
  last_safe_cmd_.fill(0.0);
  last_action12_.fill(0.0);
  tick_ = 0;

  // TODO: policy driver load / init (BOOT 단계)
  // - file path param을 받아 PolicyDriver 생성
  // - 실패 시 configure 실패 처리
  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn RoaPolicyController::on_activate(
  const rclcpp_lifecycle::State&) {
  // 활성화 시점: last_safe_cmd_를 현재 관절 위치로 초기화하는 것도 가능
  tick_ = 0;
  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn RoaPolicyController::on_deactivate(
  const rclcpp_lifecycle::State&) {
  return CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
RoaPolicyController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration cfg;
  cfg.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  // 스펙 4️⃣:
  // State Interfaces:
  // 12 DOF joint position
  // 12 DOF joint velocity
  // IMU gyro 3축
  //
  // 여기서는 joint 이름이 파라미터로 들어온다고 가정해야 함.
  // 초기 스켈레톤에서는 비워두고, 다음 단계에서 joint_names 파라미터를 받아 채운다.
  return cfg;
}

controller_interface::InterfaceConfiguration
RoaPolicyController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration cfg;
  cfg.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  // 스펙 4️⃣:
  // Command Interfaces: 12 DOF joint position target
  // 마찬가지로 joint_names 기반으로 채운다.
  return cfg;
}

// ---- helpers (스켈레톤) ----
bool RoaPolicyController::read_state_snapshot(JointSnapshot& js, ImuOmega& omega) {
  // TODO:
  // state_interfaces_에서 12 pos/vel, imu gyro를 읽어서 js/omega 채우기
  // 여기서는 성공했다고 가정하지 말고, 실제 구현 시 index mapping을 configure에서 캐싱해야 RT에 좋음.
  (void)js;
  (void)omega;
  return true;
}

bool RoaPolicyController::write_command(const std::array<double, 12>& cmd) {
  // TODO:
  // command_interfaces_에 12 position target write
  (void)cmd;
  return true;
}

// ---- update(): 스펙 9️⃣ 순서 고정 ----
controller_interface::return_type RoaPolicyController::update(
  const rclcpp::Time& time, const rclcpp::Duration& period) {

  (void)period;

  // 1) state snapshot 읽기
  JointSnapshot js;
  ImuOmega omega;
  const bool state_ok = read_state_snapshot(js, omega);

  // 2) rsu solution usable 체크
  const uint64_t now_ns = now_ns_from_time(time);
  const bool rsu_ok = rsu_latch_.usable(now_ns);

  // 3) command write (기본: last_safe_cmd_ 또는 RUN cmd)
  //   - RUN에서 policy cmd를 사용
  //   - SAFE_HOLD면 last_safe_cmd_ 유지
  bool ok_for_fsm = state_ok;

  // 50Hz tick 여부(decimation)
  const bool do_policy = ((tick_ % kDecimation) == 0);

  if (do_policy && state_ok) {
    // 4) obs build
    // TODO: typed_policy_driver::Obs obs = ...
    // 5) inference
    //   - 실패 시 last_action 유지
    //   - NaN이면 ok_for_fsm=false
    bool inference_ok = true;

    // TODO: 실제 policy inference 결과로 last_action12_ 갱신
    // if (!inference_ok) keep last_action12_

    ok_for_fsm = ok_for_fsm && inference_ok;

    // 6) last_action 갱신 (성공 시에만)
    // -> 위 TODO로 처리

    // 7) ankle R/P 생성
    // TODO: action/상태 기반으로 left/right roll/pitch 생성

    // 8) RSU target publish (RT-safe)
    if (rt_pub_rsu_target_ && rt_pub_rsu_target_->trylock()) {
      auto& msg = rt_pub_rsu_target_->msg_;
      msg.header.stamp = time;
      msg.seq = tick_;

      // TODO: 생성한 roll/pitch를 넣기
      msg.left_roll = 0.0f;
      msg.left_pitch = 0.0f;
      msg.right_roll = 0.0f;
      msg.right_pitch = 0.0f;

      rt_pub_rsu_target_->unlockAndPublish();
    }
  }

  // command compose
  std::array<double, 12> cmd{};
  const bool composed_ok = composer_.compose(last_safe_cmd_, last_action12_, rsu_latch_, rsu_ok, cmd);
  ok_for_fsm = ok_for_fsm && composed_ok;

  // FSM step (SAFE_HOLD 전이 로직)
  fsm_.step(ok_for_fsm);

  // SAFE_HOLD면 마지막 안전 cmd 유지, RUN이면 cmd를 안전 cmd로 갱신
  if (fsm_.state() == FsmState::SAFE_HOLD) {
    // hold last_safe_cmd_
    (void)write_command(last_safe_cmd_);
  } else {
    // RUN: cmd write + last_safe_cmd 갱신
    if (write_command(cmd)) {
      last_safe_cmd_ = cmd;
    } else {
      // write 실패도 실패로 보고 hold
      fsm_.step(false);
      (void)write_command(last_safe_cmd_);
    }
  }

  ++tick_;
  return controller_interface::return_type::OK;
}

}  // namespace roa_policy_controller