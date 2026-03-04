#pragma once
#include <array>
#include <cstdint>
#include <memory>

#include "controller_interface/controller_interface.hpp"
#include "realtime_tools/realtime_publisher.h"
#include "rclcpp/rclcpp.hpp"

#include "roa_interfaces/msg/rsu_target.hpp"
#include "roa_interfaces/msg/rsu_solution.hpp"

#include "roa_policy_controller/fsm.hpp"
#include "roa_policy_controller/latches.hpp"
#include "roa_policy_controller/rsu_latch.hpp"
#include "roa_policy_controller/obs_builder.hpp"
#include "roa_policy_controller/command_composer.hpp"

// TODO: 여기에 roa_policy_driver wrapper include 추가
// 예) #include "roa_policy_driver/policy_10dof_v1.hpp"

namespace roa_policy_controller {

class RoaPolicyController : public controller_interface::ControllerInterface {
public:
  RoaPolicyController();

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::return_type update(const rclcpp::Time& time,
                                           const rclcpp::Duration& period) override;

  CallbackReturn on_init() override;
  CallbackReturn on_configure(const rclcpp_lifecycle::State& prev_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State& prev_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& prev_state) override;

private:
  // --- timing ---
  static constexpr uint32_t kHwHz = 500;     // 목표
  static constexpr uint32_t kPolicyHz = 50;  // 고정
  static constexpr uint32_t kDecimation = kHwHz / kPolicyHz; // 10
  uint32_t tick_{0};

  // --- fsm ---
  Fsm fsm_{FsmConfig{}};

  // --- latches ---
  AtomicLatch<ImuOmega> imu_latch_;
  RsuLatch rsu_latch_{RsuUsableConfig{}};

  // --- policy state ---
  std::array<double, 12> last_safe_cmd_{};   // SAFE_HOLD에서 유지
  std::array<double, 12> last_action12_{};   // inference 결과 latch (실패 시 유지)

  // --- composer ---
  CommandComposer composer_{CommandLimits{}};

  // --- RSU target publisher (RT-safe) ---
  std::shared_ptr<realtime_tools::RealtimePublisher<roa_interfaces::msg::RsuTarget>> rt_pub_rsu_target_;

  // --- ROS entities (non-RT) ---
  rclcpp::Subscription<roa_interfaces::msg::RsuSolution>::SharedPtr sub_rsu_solution_;

  // --- helpers ---
  bool read_state_snapshot(JointSnapshot& js, ImuOmega& omega);
  bool write_command(const std::array<double, 12>& cmd);

  uint64_t now_ns_from_time(const rclcpp::Time& t) const {
    return static_cast<uint64_t>(t.nanoseconds());
  }
};

}  // namespace roa_policy_controller
