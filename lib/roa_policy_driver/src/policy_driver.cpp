#include "roa_policy_driver/policy_driver.hpp"

#include <algorithm>
#include <cstring>
#include <iostream>
#include <stdexcept>

#include <onnxruntime_cxx_api.h>

namespace roa::policy {

struct PolicyDriver::Impl {
  // ORT core
  Ort::Env env{ORT_LOGGING_LEVEL_WARNING, "roa_policy_driver"};
  Ort::SessionOptions sess_opt;
  std::unique_ptr<Ort::Session> session;

  Ort::AllocatorWithDefaultOptions allocator;

  std::string in_name;
  std::string out_name;

  int in_dim = -1;
  int out_dim = -1;

  // Preallocated buffers (to reduce allocations in run)
  std::vector<float> in_buf;   // size = in_dim
  std::vector<float> out_buf;  // size = out_dim

  // IO Ort::Value buffers (created per run or reused)
  // Note: In ORT C++ API, Ort::Value owns its memory only if you allocate it via ORT;
  // here we bind to preallocated vectors via CreateTensor.
  Ort::MemoryInfo mem_info = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);

  bool loaded = false;

  template <typename TensorInfoT>
	static int read_single_dim(const TensorInfoT& ti) {
		auto shape = ti.GetShape();
		if (shape.empty()) return -1;
		if (shape.size() == 1) {
			return (shape[0] > 0) ? static_cast<int>(shape[0]) : -1;
		}
		auto last = shape.back();
		return (last > 0) ? static_cast<int>(last) : -1;
	}
};

PolicyDriver::PolicyDriver() : impl_(std::make_unique<Impl>()) {}

PolicyDriver::~PolicyDriver() = default;

bool PolicyDriver::load(const std::string& onnx_path, const Options& opt) {
  try {
    impl_->sess_opt = Ort::SessionOptions{};

    impl_->sess_opt.SetIntraOpNumThreads(std::max(1, opt.intra_op_threads));
    impl_->sess_opt.SetInterOpNumThreads(std::max(1, opt.inter_op_threads));

    if (opt.enable_all_optimizations) {
      impl_->sess_opt.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
    } else if (opt.enable_basic_optimizations) {
      impl_->sess_opt.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_BASIC);
    } else {
      impl_->sess_opt.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_DISABLE_ALL);
    }

    if (!opt.use_arena) {
      // Disable CPU mem arena (rarely needed; keep true by default)
      impl_->sess_opt.DisableCpuMemArena();
    }

    impl_->session = std::make_unique<Ort::Session>(impl_->env, onnx_path.c_str(), impl_->sess_opt);

    // Single input/output expected
    const size_t n_in = impl_->session->GetInputCount();
    const size_t n_out = impl_->session->GetOutputCount();
    if (n_in < 1 || n_out < 1) {
      std::cerr << "[PolicyDriver] Model has no inputs/outputs.\n";
      impl_->loaded = false;
      return false;
    }

    // Input name
    {
      Ort::AllocatedStringPtr name = impl_->session->GetInputNameAllocated(0, impl_->allocator);
      impl_->in_name = name.get();
    }
    // Output name
    {
      Ort::AllocatedStringPtr name = impl_->session->GetOutputNameAllocated(0, impl_->allocator);
      impl_->out_name = name.get();
    }

    // Input dim
    {
      Ort::TypeInfo ti = impl_->session->GetInputTypeInfo(0);
      auto tensor_info = ti.GetTensorTypeAndShapeInfo();
      if (tensor_info.GetElementType() != ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT) {
        std::cerr << "[PolicyDriver] Input type is not float.\n";
        impl_->loaded = false;
        return false;
      }
      impl_->in_dim = Impl::read_single_dim(tensor_info);
    }

    // Output dim
    {
      Ort::TypeInfo ti = impl_->session->GetOutputTypeInfo(0);
      auto tensor_info = ti.GetTensorTypeAndShapeInfo();
      if (tensor_info.GetElementType() != ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT) {
        std::cerr << "[PolicyDriver] Output type is not float.\n";
        impl_->loaded = false;
        return false;
      }
      impl_->out_dim = Impl::read_single_dim(tensor_info);
    }

    if (impl_->in_dim <= 0 || impl_->out_dim <= 0) {
      std::cerr << "[PolicyDriver] Failed to infer input/output dims (dynamic shape not resolved).\n";
      std::cerr << "  in_dim=" << impl_->in_dim << " out_dim=" << impl_->out_dim << "\n";
      impl_->loaded = false;
      return false;
    }

    // Preallocate buffers
    impl_->in_buf.assign(static_cast<size_t>(impl_->in_dim), 0.0f);
    impl_->out_buf.assign(static_cast<size_t>(impl_->out_dim), 0.0f);

    impl_->loaded = true;
    return true;

  } catch (const Ort::Exception& e) {
    std::cerr << "[PolicyDriver] ORT exception: " << e.what() << "\n";
    impl_->loaded = false;
    return false;
  } catch (const std::exception& e) {
    std::cerr << "[PolicyDriver] Exception: " << e.what() << "\n";
    impl_->loaded = false;
    return false;
  }
}

int PolicyDriver::input_dim() const noexcept { return impl_->in_dim; }
int PolicyDriver::output_dim() const noexcept { return impl_->out_dim; }

const std::string& PolicyDriver::input_name() const noexcept { return impl_->in_name; }
const std::string& PolicyDriver::output_name() const noexcept { return impl_->out_name; }

bool PolicyDriver::is_loaded() const noexcept { return impl_->loaded; }

// bool PolicyDriver::run(const float* obs, int obs_dim, float* action, int action_dim) noexcept {
//   if (!impl_->loaded || !impl_->session) return false;
//   if (!obs || !action) return false;
//   if (obs_dim != impl_->in_dim) return false;
//   if (action_dim != impl_->out_dim) return false;

//   try {
//     // Copy obs to internal buffer (keeps IO memory stable)
//     std::memcpy(impl_->in_buf.data(), obs, sizeof(float) * static_cast<size_t>(impl_->in_dim));

//     // Create input tensor [1, in_dim]
//     std::array<int64_t, 2> in_shape{1, static_cast<int64_t>(impl_->in_dim)};
//     Ort::Value in_tensor = Ort::Value::CreateTensor<float>(
//         impl_->mem_info,
//         impl_->in_buf.data(),
//         impl_->in_buf.size(),
//         in_shape.data(),
//         in_shape.size());

//     // Prepare output tensor buffer [1, out_dim]
//     std::array<int64_t, 2> out_shape{1, static_cast<int64_t>(impl_->out_dim)};
//     Ort::Value out_tensor = Ort::Value::CreateTensor<float>(
//         impl_->mem_info,
//         impl_->out_buf.data(),
//         impl_->out_buf.size(),
//         out_shape.data(),
//         out_shape.size());

//     const char* in_names[] = {impl_->in_name.c_str()};
//     const char* out_names[] = {impl_->out_name.c_str()};

//     // Run (use preallocated output)
//     impl_->session->Run(
//         Ort::RunOptions{nullptr},
//         in_names, &in_tensor, 1,
//         out_names, &out_tensor, 1);

//     // Copy to caller
//     std::memcpy(action, impl_->out_buf.data(), sizeof(float) * static_cast<size_t>(impl_->out_dim));
//     return true;

//   } catch (...) {
//     // noexcept path: swallow and fail
//     return false;
//   }
// }

bool PolicyDriver::run(const float* obs, int obs_dim, float* action, int action_dim) noexcept {
  if (!impl_->loaded || !impl_->session) return false;
  if (!obs || !action) return false;
  if (obs_dim != impl_->in_dim) return false;
  if (action_dim != impl_->out_dim) return false;

  try {
    // Copy obs to internal buffer (stable memory)
    std::memcpy(impl_->in_buf.data(), obs, sizeof(float) * static_cast<size_t>(impl_->in_dim));

    // Create input tensor [1, in_dim]
    std::array<int64_t, 2> in_shape{1, static_cast<int64_t>(impl_->in_dim)};
    Ort::Value in_tensor = Ort::Value::CreateTensor<float>(
        impl_->mem_info,
        impl_->in_buf.data(),
        impl_->in_buf.size(),
        in_shape.data(),
        in_shape.size());

    const char* in_names[] = {impl_->in_name.c_str()};
    const char* out_names[] = {impl_->out_name.c_str()};

    // Run and fetch outputs from return value (robust across ORT versions)
    auto outputs = impl_->session->Run(
        Ort::RunOptions{nullptr},
        in_names, &in_tensor, 1,
        out_names, 1);

    if (outputs.size() != 1 || !outputs[0].IsTensor()) return false;

    auto out_info = outputs[0].GetTensorTypeAndShapeInfo();
    auto out_shape = out_info.GetShape();
    // expected [1, out_dim] or [out_dim]
    int out_dim = -1;
    if (out_shape.size() == 2) out_dim = static_cast<int>(out_shape[1]);
    else if (out_shape.size() == 1) out_dim = static_cast<int>(out_shape[0]);
    if (out_dim != impl_->out_dim) return false;

    const float* out_ptr = outputs[0].GetTensorData<float>();
    std::memcpy(action, out_ptr, sizeof(float) * static_cast<size_t>(impl_->out_dim));
    return true;

  } catch (...) {
    return false;
  }
}


bool PolicyDriver::run(const std::vector<float>& obs, std::vector<float>& action) {
  if (!impl_->loaded) return false;
  if (static_cast<int>(obs.size()) != impl_->in_dim) return false;
  action.resize(static_cast<size_t>(impl_->out_dim));
  return run(obs.data(), static_cast<int>(obs.size()), action.data(), static_cast<int>(action.size()));
}

}  // namespace roa::policy
