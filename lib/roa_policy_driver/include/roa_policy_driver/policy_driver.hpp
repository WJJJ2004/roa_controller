#pragma once

#include <cstddef>
#include <memory>
#include <string>
#include <vector>

namespace roa::policy {

/**
 * Pure ONNX inference driver (no ROS, no observation building).
 *
 * Contract:
 * - load() loads a single-input single-output policy.
 * - run() takes float32 obs and returns float32 action.
 * - input/output dims are read from the model; caller must respect them.
 *
 * Real-time note:
 * - Use pointer-based run() to avoid allocations.
 * - Internal ORT IO tensors are pre-allocated when possible.
 */
struct Options {
  int intra_op_threads = 1;
  int inter_op_threads = 1;
  bool enable_basic_optimizations = true;   // ORT_ENABLE_BASIC
  bool enable_all_optimizations = false;    // ORT_ENABLE_ALL
  bool use_arena = true;                    // ORT memory arena
};

class PolicyDriver {
public:
  PolicyDriver();
  ~PolicyDriver();

  PolicyDriver(const PolicyDriver&) = delete;
  PolicyDriver& operator=(const PolicyDriver&) = delete;

  bool load(const std::string& onnx_path, const Options& opt = Options{});

  int input_dim() const noexcept;
  int output_dim() const noexcept;

  const std::string& input_name() const noexcept;
  const std::string& output_name() const noexcept;

  // RT-friendly: no allocations expected on success path.
  // Returns false if not loaded or dimension mismatch.
  bool run(const float* obs, int obs_dim, float* action, int action_dim) noexcept;

  // Convenience overloads (may allocate).
  bool run(const std::vector<float>& obs, std::vector<float>& action);

  bool is_loaded() const noexcept;

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace roa::policy
