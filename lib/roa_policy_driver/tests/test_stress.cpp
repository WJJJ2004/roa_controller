/*
# NOTE
구 버전 테스트 코드 > core 모듈에 대한 하드코딩 테스트
*/


#include "roa_policy_driver/policy_driver.hpp"

#include <chrono>
#include <iostream>
#include <random>
#include <vector>

int main(int argc, char** argv) {
  std::string model = "onnx/12dof/policy.onnx";
  int iters = 200000;  // 기본 20만번
  if (argc >= 2) model = argv[1];
  if (argc >= 3) iters = std::max(1, std::atoi(argv[2]));

  roa::policy::PolicyDriver drv;
  if (!drv.load(model)) {
    std::cerr << "FAILED: load " << model << "\n";
    return 1;
  }

  const int in_dim = drv.input_dim();
  const int out_dim = drv.output_dim();
  if (in_dim <= 0 || out_dim <= 0) return 2;

  std::vector<float> obs(in_dim, 0.0f);
  std::vector<float> act(out_dim, 0.0f);

  // 랜덤 but 안정적인 값 (sim2sim 범위 비슷하게)
  std::mt19937 rng(0);
  std::uniform_real_distribution<float> cmdx(-1.0f, 1.0f);
  std::uniform_real_distribution<float> cmdy(-0.5f, 0.5f);
  std::uniform_real_distribution<float> cmdw(-1.0f, 1.0f);
  std::uniform_real_distribution<float> small(-1.0f, 1.0f);

  auto t0 = std::chrono::high_resolution_clock::now();

  for (int i = 0; i < iters; ++i) {
    // cmd
    obs[0] = cmdx(rng);
    obs[1] = cmdy(rng);
    obs[2] = cmdw(rng);
    // 나머지는 대충 작은 값 (실제는 q_rel/qd/imu/last_action)
    for (int k = 3; k < in_dim; ++k) obs[k] = 0.1f * small(rng);

    if (!drv.run(obs.data(), in_dim, act.data(), out_dim)) {
      std::cerr << "FAILED: run at i=" << i << "\n";
      return 3;
    }
  }

  auto t1 = std::chrono::high_resolution_clock::now();
  double ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
  std::cout << "iters=" << iters << " total_ms=" << ms
            << " avg_us=" << (ms * 1000.0 / iters) << "\n";

  std::cout << "PASS\n";
  return 0;
}
