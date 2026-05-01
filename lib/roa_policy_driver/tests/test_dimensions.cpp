/*
# NOTE
구 버전 테스트 코드 > core 모듈에 대한 하드코딩 테스트
*/

#include "roa_policy_driver/policy_driver.hpp"
#include <cstdlib>
#include <iostream>
#include <string>

static std::string default_model_path() {
  // 현재 repo 트리 기준: ./onnx/12dof/policy.onnx
  return std::string("onnx/12dof/policy.onnx");
}

int main(int argc, char** argv) {
  std::string model = default_model_path();
  if (argc >= 2) model = argv[1];

  roa::policy::PolicyDriver drv;
  if (!drv.load(model)) {
    std::cerr << "FAILED: load model: " << model << "\n";
    return 1;
  }

  std::cout << "Loaded model: " << model << "\n";
  std::cout << "Input name: " << drv.input_name() << " dim=" << drv.input_dim() << "\n";
  std::cout << "Output name: " << drv.output_name() << " dim=" << drv.output_dim() << "\n";

  if (drv.input_dim() != 42) {
    std::cerr << "FAILED: expected input_dim=42 but got " << drv.input_dim() << "\n";
    return 2;
  }
  if (drv.output_dim() != 12) {
    std::cerr << "FAILED: expected output_dim=12 but got " << drv.output_dim() << "\n";
    return 3;
  }

  std::cout << "PASS\n";
  return 0;
}

