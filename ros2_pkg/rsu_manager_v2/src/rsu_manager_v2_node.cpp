#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <realtime_tools/realtime_buffer.hpp>

#include <roa_interfaces/msg/motor_state_array.hpp>
#include <roa_interfaces/msg/rsu_imp_sol.hpp>
#include <roa_interfaces/msg/rsu_state_array.hpp>
#include <roa_interfaces/msg/rsu_target.hpp>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "rsu_manager_v2/rsu_lut.hpp"
#include "rsu_manager_v2/rsu_model.hpp"

namespace rsu_manager_v2
{
using namespace std::chrono_literals;

namespace
{
std::int64_t steady_now_ns()
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
}

std::int64_t stamp_ns(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<std::int64_t>(stamp.sec) * 1000000000LL + stamp.nanosec;
}

builtin_interfaces::msg::Time ns_to_stamp(std::int64_t value)
{
  builtin_interfaces::msg::Time stamp;
  if (value <= 0) {return stamp;}
  stamp.sec = static_cast<std::int32_t>(value / 1000000000LL);
  stamp.nanosec = static_cast<std::uint32_t>(value % 1000000000LL);
  return stamp;
}

template<typename Clock, typename Duration>
void advance_deadline(std::chrono::time_point<Clock, Duration> & deadline, const Duration & period)
{
  deadline += period;
  const auto now = Clock::now();
  if (deadline < now - period) {deadline = now + period;}
  std::this_thread::sleep_until(deadline);
}

template<typename T>
std::array<double, 2> array_parameter(rclcpp::Node & node, const std::string & name, const T & defaults)
{
  const auto values = node.declare_parameter<std::vector<double>>(name, {defaults[0], defaults[1]});
  if (values.size() != 2) {throw std::runtime_error(name + " must contain two values");}
  return {values[0], values[1]};
}
}  // namespace

struct TargetInput
{
  std::uint64_t generation{0};
  std::int64_t received_steady_ns{0};
  std::int64_t source_stamp_ns{0};
  std::uint32_t source_seq{0};
  std::array<double, 4> virtual_target{};  // L roll,pitch,R roll,pitch external convention
  bool finite{false};
};

struct FeedbackInput
{
  std::uint64_t generation{0};
  std::int64_t received_steady_ns{0};
  std::int64_t source_stamp_ns{0};
  std::array<double, 4> position{};  // L1,L2,R1,R2 physical convention
  std::array<double, 4> velocity{};
  bool complete{false};
  bool finite{false};
};

struct TargetSnapshot
{
  std::uint64_t generation{0};
  std::int64_t received_steady_ns{0};
  std::int64_t source_stamp_ns{0};
  std::uint32_t source_seq{0};
  std::array<double, 4> actuator_target{};
  std::uint32_t clipped_axes{0};
  bool valid{false};
};

struct StateSnapshot
{
  std::uint64_t generation{0};
  std::int64_t received_steady_ns{0};
  std::int64_t source_stamp_ns{0};
  std::array<double, 4> q{};   // external virtual convention
  std::array<double, 4> qd{};
  std::array<double, 4> kp{};  // physical actuator ordering L1,L2,R1,R2
  std::array<double, 4> kd{};
  bool valid{false};
  bool degraded{false};
};

template<typename T, std::size_t Capacity>
class SpscQueue
{
public:
  bool push(const T & value) noexcept
  {
    const auto write = write_.load(std::memory_order_relaxed);
    const auto next = (write + 1) % Capacity;
    if (next == read_.load(std::memory_order_acquire)) {return false;}
    storage_[write] = value;
    write_.store(next, std::memory_order_release);
    return true;
  }

  bool pop(T & value) noexcept
  {
    const auto read = read_.load(std::memory_order_relaxed);
    if (read == write_.load(std::memory_order_acquire)) {return false;}
    value = storage_[read];
    read_.store((read + 1) % Capacity, std::memory_order_release);
    return true;
  }

private:
  std::array<T, Capacity> storage_{};
  alignas(64) std::atomic<std::size_t> write_{0};
  alignas(64) std::atomic<std::size_t> read_{0};
};

class RsuManagerV2Node final : public rclcpp::Node
{
public:
  RsuManagerV2Node()
  : Node("rsu_manager_v2")
  {
    const auto default_lut = ament_index_cpp::get_package_share_directory("rsu_manager_v2") +
      "/data/rsu_lut_v2.bin";
    auto lut_file = declare_parameter<std::string>("lut_file", "");
    if (lut_file.empty()) {lut_file = default_lut;}
    lut_.load(lut_file);

    target_poll_hz_ = declare_parameter<double>("target_poll_hz", 300.0);
    state_poll_hz_ = declare_parameter<double>("state_poll_hz", 600.0);
    command_rate_hz_ = declare_parameter<double>("command_rate_hz", 300.0);
    target_timeout_ns_ = static_cast<std::int64_t>(declare_parameter<double>("target_timeout_ms", 100.0) * 1e6);
    feedback_timeout_ns_ = static_cast<std::int64_t>(declare_parameter<double>("feedback_timeout_ms", 20.0) * 1e6);
    feedback_margin_ = declare_parameter<double>("feedback_actuator_margin_deg", 2.0) * M_PI / 180.0;
    publish_state_every_control_tick_ = declare_parameter<bool>("publish_state_every_control_tick", true);
    if (target_poll_hz_ <= 0.0 || state_poll_hz_ <= 0.0 || command_rate_hz_ <= 0.0 ||
      target_timeout_ns_ <= 0 || feedback_timeout_ns_ <= 0) {
      throw std::runtime_error("rates and freshness timeouts must be positive");
    }

    const auto left_ids = declare_parameter<std::vector<std::int64_t>>("left_motor_ids", {18, 20});
    const auto right_ids = declare_parameter<std::vector<std::int64_t>>("right_motor_ids", {19, 21});
    if (left_ids.size() != 2 || right_ids.size() != 2) {throw std::runtime_error("motor ID arrays must contain two IDs");}
    motor_ids_ = {left_ids[0], left_ids[1], right_ids[0], right_ids[1]};

    const auto initial_q = array_parameter(*this, "initial_q", std::array<double, 2>{0.0, -0.5672320});
    const auto initial_alpha = array_parameter(*this, "initial_alpha", std::array<double, 2>{-0.458105, 0.458105});
    left_estimator_ = std::make_unique<StateEstimator>(lut_);
    right_estimator_ = std::make_unique<StateEstimator>(lut_);
    left_estimator_->reset(initial_q, initial_alpha);
    right_estimator_->reset(initial_q, initial_alpha);

    ImpedanceConfig impedance;
    impedance.virtual_pitch_kp = declare_parameter<double>("virtual_pitch_kp", 25.0);
    impedance.virtual_pitch_kd = declare_parameter<double>("virtual_pitch_kd", 1.2);
    impedance.virtual_roll_scale = declare_parameter<double>("virtual_roll_scale", 1.37);
    impedance.kp_min = array_parameter(*this, "actuator_kp_min", std::array<double, 2>{5.0, 5.0});
    impedance.kp_max = array_parameter(*this, "actuator_kp_max", std::array<double, 2>{25.0, 25.0});
    impedance.kd_min = array_parameter(*this, "actuator_kd_min", std::array<double, 2>{0.2, 0.2});
    impedance.kd_max = array_parameter(*this, "actuator_kd_max", std::array<double, 2>{4.9, 4.9});
    impedance.default_kp = array_parameter(*this, "default_kp", std::array<double, 2>{9.0, 9.0});
    impedance.default_kd = array_parameter(*this, "default_kd", std::array<double, 2>{2.25, 2.25});
    left_mapper_ = std::make_unique<ImpedanceMapper>(impedance);
    right_mapper_ = std::make_unique<ImpedanceMapper>(impedance);

    target_snapshot_buffer_.initRT(TargetSnapshot{});
    state_snapshot_buffer_.initRT(StateSnapshot{});

    auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();
    target_sub_ = create_subscription<roa_interfaces::msg::RsuTarget>(
      "/rsu/target", qos, std::bind(&RsuManagerV2Node::on_target, this, std::placeholders::_1));
    feedback_sub_ = create_subscription<roa_interfaces::msg::MotorStateArray>(
      "/hardware_interface/state", qos, std::bind(&RsuManagerV2Node::on_feedback, this, std::placeholders::_1));
    solution_pub_ = create_publisher<roa_interfaces::msg::RsuImpSol>("/rsu/imp_solution", qos);
    state_pub_ = create_publisher<roa_interfaces::msg::RsuStateArray>("/rsu/state", qos);

    running_.store(true);
    target_thread_ = std::thread(&RsuManagerV2Node::target_loop, this);
    state_thread_ = std::thread(&RsuManagerV2Node::state_loop, this);
    command_thread_ = std::thread(&RsuManagerV2Node::command_loop, this);
    RCLCPP_INFO(get_logger(),
      "RSU manager v2 ready: target poll %.1f Hz, state poll %.1f Hz, command %.1f Hz, LUT=%s",
      target_poll_hz_, state_poll_hz_, command_rate_hz_, lut_file.c_str());
  }

  ~RsuManagerV2Node() override
  {
    running_.store(false);
    if (target_thread_.joinable()) {target_thread_.join();}
    if (state_thread_.joinable()) {state_thread_.join();}
    if (command_thread_.joinable()) {command_thread_.join();}
    RCLCPP_INFO(get_logger(),
      "RSU v2 counters: feedback received=%lu processed=%lu queue_dropped=%lu target_clipped_axes=%lu",
      feedback_received_.load(), feedback_processed_.load(), feedback_queue_dropped_.load(),
      target_clip_count_.load());
  }

private:
  void on_target(const roa_interfaces::msg::RsuTarget::SharedPtr message)
  {
    TargetInput input;
    input.generation = ++target_input_generation_;
    input.received_steady_ns = steady_now_ns();
    input.source_stamp_ns = stamp_ns(message->header.stamp);
    input.source_seq = message->seq;
    input.virtual_target = {message->left_roll, message->left_pitch, message->right_roll, message->right_pitch};
    input.finite = std::all_of(input.virtual_target.begin(), input.virtual_target.end(),
      [](double value) {return std::isfinite(value);});
    std::atomic_store(&target_input_, std::make_shared<const TargetInput>(input));
  }

  void on_feedback(const roa_interfaces::msg::MotorStateArray::SharedPtr message)
  {
    FeedbackInput input;
    input.generation = ++feedback_input_generation_;
    input.received_steady_ns = steady_now_ns();
    input.source_stamp_ns = stamp_ns(message->header.stamp);
    std::array<bool, 4> found{};
    for (const auto & state : message->states) {
      for (std::size_t i = 0; i < motor_ids_.size(); ++i) {
        if (state.motor_id == motor_ids_[i]) {
          input.position[i] = state.position;
          input.velocity[i] = state.velocity;
          found[i] = true;
        }
      }
    }
    input.complete = std::all_of(found.begin(), found.end(), [](bool value) {return value;});
    input.finite = input.complete &&
      std::all_of(input.position.begin(), input.position.end(), [](double value) {return std::isfinite(value);}) &&
      std::all_of(input.velocity.begin(), input.velocity.end(), [](double value) {return std::isfinite(value);});
    feedback_received_.fetch_add(1);
    if (!feedback_queue_.push(input)) {feedback_queue_dropped_.fetch_add(1);}
  }

  void target_loop()
  {
    const auto period = std::chrono::nanoseconds(static_cast<std::int64_t>(1e9 / target_poll_hz_));
    auto deadline = std::chrono::steady_clock::now();
    std::uint64_t processed = 0;
    while (running_.load()) {
      const auto input_pointer = std::atomic_load(&target_input_);
      const TargetInput input = input_pointer ? *input_pointer : TargetInput{};
      if (input.generation != 0 && input.generation != processed) {
        processed = input.generation;
        TargetSnapshot output;
        output.generation = input.generation;
        output.received_steady_ns = input.received_steady_ns;
        output.source_stamp_ns = input.source_stamp_ns;
        output.source_seq = input.source_seq;
        if (input.finite) {
          std::array<double, 4> internal{
            input.virtual_target[0], input.virtual_target[1],
            -input.virtual_target[2], -input.virtual_target[3]};
          for (std::size_t foot = 0; foot < 2; ++foot) {
            const auto roll_index = foot * 2;
            const double clipped_roll = std::clamp(internal[roll_index], lut_.roll_min(), lut_.roll_max());
            const double clipped_pitch = std::clamp(internal[roll_index + 1], lut_.pitch_min(), lut_.pitch_max());
            output.clipped_axes += clipped_roll != internal[roll_index];
            output.clipped_axes += clipped_pitch != internal[roll_index + 1];
            internal[roll_index] = clipped_roll;
            internal[roll_index + 1] = clipped_pitch;
          }
          const auto left = lut_.query(internal[0], internal[1]);
          const auto right = lut_.query(internal[2], internal[3]);
          if (left.valid && right.valid) {
            const auto minimum = lut_.actuator_min();
            const auto maximum = lut_.actuator_max();
            output.actuator_target = {
              std::clamp(left.alpha[0], minimum[0], maximum[0]),
              std::clamp(left.alpha[1], minimum[1], maximum[1]),
              -std::clamp(right.alpha[0], minimum[0], maximum[0]),
              -std::clamp(right.alpha[1], minimum[1], maximum[1])};
            output.valid = true;
          }
        }
        if (output.clipped_axes > 0) {target_clip_count_.fetch_add(output.clipped_axes);}
        target_snapshot_buffer_.writeFromNonRT(output);
      }
      advance_deadline(deadline, period);
    }
  }

  void state_loop()
  {
    const auto period = std::chrono::nanoseconds(static_cast<std::int64_t>(1e9 / state_poll_hz_));
    auto deadline = std::chrono::steady_clock::now();
    std::int64_t previous_source_stamp = 0;
    while (running_.load()) {
      FeedbackInput input;
      while (feedback_queue_.pop(input)) {
        feedback_processed_.fetch_add(1);
        StateSnapshot output;
        output.generation = input.generation;
        output.received_steady_ns = input.received_steady_ns;
        output.source_stamp_ns = input.source_stamp_ns;
        double dt = 0.0;
        if (previous_source_stamp > 0) {dt = (input.source_stamp_ns - previous_source_stamp) * 1e-9;}
        previous_source_stamp = input.source_stamp_ns;
        const auto minimum = lut_.actuator_min();
        const auto maximum = lut_.actuator_max();
        bool feedback_in_range = input.finite;
        std::array<double, 2> left_position{input.position[0], input.position[1]};
        std::array<double, 2> left_velocity{input.velocity[0], input.velocity[1]};
        std::array<double, 2> right_position{-input.position[2], -input.position[3]};
        std::array<double, 2> right_velocity{-input.velocity[2], -input.velocity[3]};
        for (const auto & position : {left_position, right_position}) {
          for (std::size_t i = 0; i < 2; ++i) {
            feedback_in_range = feedback_in_range && position[i] >= minimum[i] - feedback_margin_ &&
              position[i] <= maximum[i] + feedback_margin_;
          }
        }
        if (feedback_in_range && dt >= 1e-5 && dt <= 0.1) {
          const auto left = left_estimator_->update(left_position, left_velocity, dt);
          const auto right = right_estimator_->update(right_position, right_velocity, dt);
          const auto left_imp = left_mapper_->compute(left.jacobian, left.valid);
          const auto right_imp = right_mapper_->compute(right.jacobian, right.valid);
          output.q = {left.q[0], left.q[1], -right.q[0], -right.q[1]};
          output.qd = {left.qd[0], left.qd[1], -right.qd[0], -right.qd[1]};
          output.kp = {left_imp.kp[0], left_imp.kp[1], right_imp.kp[0], right_imp.kp[1]};
          output.kd = {left_imp.kd[0], left_imp.kd[1], right_imp.kd[0], right_imp.kd[1]};
          output.valid = left.valid && right.valid && left_imp.valid && right_imp.valid;
          output.degraded = left.degraded || right.degraded || left_imp.saturated || right_imp.saturated;
        }
        state_snapshot_buffer_.writeFromNonRT(output);
      }
      advance_deadline(deadline, period);
    }
  }

  void command_loop()
  {
    const auto period = std::chrono::nanoseconds(static_cast<std::int64_t>(1e9 / command_rate_hz_));
    auto deadline = std::chrono::steady_clock::now();
    while (running_.load()) {
      const auto target = *target_snapshot_buffer_.readFromRT();
      const auto state = *state_snapshot_buffer_.readFromRT();
      const auto now_steady = steady_now_ns();
      const bool target_fresh = target.valid && now_steady - target.received_steady_ns >= 0 &&
        now_steady - target.received_steady_ns <= target_timeout_ns_;
      const bool state_fresh = state.valid && now_steady - state.received_steady_ns >= 0 &&
        now_steady - state.received_steady_ns <= feedback_timeout_ns_;

      roa_interfaces::msg::RsuImpSol solution;
      // The header describes the feedback epoch used for the state-dependent
      // impedance, not merely the ROS publication time.
      if (state.source_stamp_ns > 0) {
        solution.header.stamp = ns_to_stamp(state.source_stamp_ns);
      } else {
        solution.header.stamp = now();
      }
      solution.header.frame_id = "rsu_state";
      // Preserve the legacy protocol meaning: seq identifies the policy target
      // whose q_target is being held across the 300 Hz command publications.
      solution.seq = target.source_seq;
      solution.left_actuator_1.q_target = target.actuator_target[0];
      solution.left_actuator_2.q_target = target.actuator_target[1];
      solution.right_actuator_1.q_target = target.actuator_target[2];
      solution.right_actuator_2.q_target = target.actuator_target[3];
      solution.left_actuator_1.kp_eqv = state.kp[0];
      solution.left_actuator_2.kp_eqv = state.kp[1];
      solution.right_actuator_1.kp_eqv = state.kp[2];
      solution.right_actuator_2.kp_eqv = state.kp[3];
      solution.left_actuator_1.kd_eqv = state.kd[0];
      solution.left_actuator_2.kd_eqv = state.kd[1];
      solution.right_actuator_1.kd_eqv = state.kd[2];
      solution.right_actuator_2.kd_eqv = state.kd[3];
      solution.feasible = target_fresh && state_fresh;
      solution_pub_->publish(solution);

      if (publish_state_every_control_tick_) {
        roa_interfaces::msg::RsuStateArray message;
        message.header.stamp = solution.header.stamp;
        message.header.frame_id = "rsu_state";
        message.seq = command_sequence_;
        message.q.left_rsu_roll = state.q[0];
        message.q.left_rsu_pitch = state.q[1];
        message.q.right_rsu_roll = state.q[2];
        message.q.right_rsu_pitch = state.q[3];
        message.q_dot.left_rsu_roll = state.qd[0];
        message.q_dot.left_rsu_pitch = state.qd[1];
        message.q_dot.right_rsu_roll = state.qd[2];
        message.q_dot.right_rsu_pitch = state.qd[3];
        message.feasible = state_fresh;
        state_pub_->publish(message);
      }
      ++command_sequence_;
      advance_deadline(deadline, period);
    }
  }

  RsuLut lut_;
  std::unique_ptr<StateEstimator> left_estimator_;
  std::unique_ptr<StateEstimator> right_estimator_;
  std::unique_ptr<ImpedanceMapper> left_mapper_;
  std::unique_ptr<ImpedanceMapper> right_mapper_;
  std::array<std::int64_t, 4> motor_ids_{};
  double target_poll_hz_{300.0};
  double state_poll_hz_{600.0};
  double command_rate_hz_{300.0};
  std::int64_t target_timeout_ns_{100000000};
  std::int64_t feedback_timeout_ns_{20000000};
  double feedback_margin_{0.0};
  bool publish_state_every_control_tick_{true};

  std::shared_ptr<const TargetInput> target_input_;
  SpscQueue<FeedbackInput, 256> feedback_queue_;
  realtime_tools::RealtimeBuffer<TargetSnapshot> target_snapshot_buffer_;
  realtime_tools::RealtimeBuffer<StateSnapshot> state_snapshot_buffer_;
  std::atomic<std::uint64_t> target_input_generation_{0};
  std::atomic<std::uint64_t> feedback_input_generation_{0};
  std::atomic<std::uint64_t> target_clip_count_{0};
  std::atomic<std::uint64_t> feedback_received_{0};
  std::atomic<std::uint64_t> feedback_processed_{0};
  std::atomic<std::uint64_t> feedback_queue_dropped_{0};
  std::atomic<bool> running_{false};
  std::uint32_t command_sequence_{0};
  std::thread target_thread_;
  std::thread state_thread_;
  std::thread command_thread_;

  rclcpp::Subscription<roa_interfaces::msg::RsuTarget>::SharedPtr target_sub_;
  rclcpp::Subscription<roa_interfaces::msg::MotorStateArray>::SharedPtr feedback_sub_;
  rclcpp::Publisher<roa_interfaces::msg::RsuImpSol>::SharedPtr solution_pub_;
  rclcpp::Publisher<roa_interfaces::msg::RsuStateArray>::SharedPtr state_pub_;
};

}  // namespace rsu_manager_v2

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<rsu_manager_v2::RsuManagerV2Node>();
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
    executor.add_node(node);
    executor.spin();
  } catch (const std::exception & error) {
    std::cerr << "rsu_manager_v2 fatal: " << error.what() << std::endl;
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
