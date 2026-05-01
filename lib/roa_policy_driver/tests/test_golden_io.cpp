/*
# NOTE
구 버전 테스트 코드 > core 모듈에 대한 하드코딩 테스트
*/


#include "roa_policy_driver/policy_driver.hpp"

#include <cmath>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

static bool read_f32_bin(const std::string& path, std::vector<float>& out) {
  std::ifstream ifs(path, std::ios::binary);
  if (!ifs) return false;
  ifs.seekg(0, std::ios::end);
  const std::streamsize nbytes = ifs.tellg();
  ifs.seekg(0, std::ios::beg);
  if (nbytes % static_cast<std::streamsize>(sizeof(float)) != 0) return false;

  const size_t n = static_cast<size_t>(nbytes / sizeof(float));
  out.resize(n);
  if (!ifs.read(reinterpret_cast<char*>(out.data()), nbytes)) return false;
  return true;
}

static float max_abs_diff(const std::vector<float>& a, const std::vector<float>& b) {
  if (a.size() != b.size()) return INFINITY;
  float m = 0.0f;
  for (size_t i = 0; i < a.size(); ++i) {
    m = std::max(m, std::fabs(a[i] - b[i]));
  }
  return m;
}

int main(int argc, char** argv) {
  std::string model = "onnx/12dof/policy.onnx";
  std::string obs_path = "onnx/12dof/data/golden_obs.bin";
  std::string act_path = "onnx/12dof/data/golden_action.bin";

  if (argc >= 2) model = argv[1];
  if (argc >= 3) obs_path = argv[2];
  if (argc >= 4) act_path = argv[3];

  roa::policy::PolicyDriver drv;
  if (!drv.load(model)) {
    std::cerr << "FAILED: load model: " << model << "\n";
    return 1;
  }
  if (drv.input_dim() != 42 || drv.output_dim() != 12) {
    std::cerr << "FAILED: unexpected dims: in=" << drv.input_dim()
              << " out=" << drv.output_dim() << "\n";
    return 2;
  }

  std::vector<float> obs, golden;
  if (!read_f32_bin(obs_path, obs)) {
    std::cerr << "FAILED: read obs: " << obs_path << "\n";
    return 3;
  }
  if (!read_f32_bin(act_path, golden)) {
    std::cerr << "FAILED: read golden action: " << act_path << "\n";
    return 4;
  }

  if (static_cast<int>(obs.size()) != drv.input_dim()) {
    std::cerr << "FAILED: obs size mismatch. got " << obs.size()
              << " expected " << drv.input_dim() << "\n";
    return 5;
  }
  if (static_cast<int>(golden.size()) != drv.output_dim()) {
    std::cerr << "FAILED: golden action size mismatch. got " << golden.size()
              << " expected " << drv.output_dim() << "\n";
    return 6;
  }

  std::vector<float> out;
  if (!drv.run(obs, out)) {
    std::cerr << "FAILED: run\n";
    return 7;
  }

  const float diff = max_abs_diff(out, golden);
  std::cout << "max_abs_diff=" << diff << "\n";

  const float tol = 1e-5f;
  if (diff > tol) {
    std::cerr << "FAILED: diff > tol (" << tol << ")\n";
    return 8;
  }

  std::cout << "PASS\n";
  return 0;
}
