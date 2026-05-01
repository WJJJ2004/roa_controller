/*


*/

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <numeric>
#include <string>
#include <vector>

#include "roa_policy_driver/policy_driver.hpp"

static bool read_bin_f32(const std::string& path, std::vector<float>& out) {
  std::ifstream ifs(path, std::ios::binary);
  if (!ifs) return false;
  ifs.seekg(0, std::ios::end);
  std::streamsize n = ifs.tellg();
  ifs.seekg(0, std::ios::beg);
  if (n <= 0 || (n % 4) != 0) return false;
  out.resize(static_cast<size_t>(n / 4));
  ifs.read(reinterpret_cast<char*>(out.data()), n);
  return static_cast<bool>(ifs);
}

static double percentile_sorted(const std::vector<double>& v_sorted, double p) {
  if (v_sorted.empty()) return 0.0;
  if (p <= 0.0) return v_sorted.front();
  if (p >= 100.0) return v_sorted.back();
  const double idx = (p / 100.0) * (static_cast<double>(v_sorted.size() - 1));
  const size_t i0 = static_cast<size_t>(idx);
  const size_t i1 = std::min(i0 + 1, v_sorted.size() - 1);
  const double t = idx - static_cast<double>(i0);
  return v_sorted[i0] * (1.0 - t) + v_sorted[i1] * t;
}

int main(int argc, char** argv) {
  if (argc < 2) {
    std::cerr
        << "usage:\n"
        << "  bench_inference <model.onnx> [obs.bin] [iters] [warmup]\n"
        << "\n"
        << "examples:\n"
        << "  bench_inference ../onnx/12dof/policy.onnx\n"
        << "  bench_inference ../onnx/12dof/policy.onnx ../onnx/12dof/data/golden_obs.bin 200000 2000\n";
    return 1;
  }

  const std::string model_path = argv[1];
  const std::string obs_path = (argc >= 3) ? argv[2] : "";
  const int iters = (argc >= 4) ? std::max(1, std::stoi(argv[3])) : 200000;
  const int warmup = (argc >= 5) ? std::max(0, std::stoi(argv[4])) : 2000;

  roa::policy::PolicyDriver driver;
  roa::policy::Options opt;

  if (!driver.load(model_path, opt)) {
    std::cerr << "FAILED: load model: " << model_path << "\n";
    return 2;
  }

  const int in_dim = driver.input_dim();
  const int out_dim = driver.output_dim();

  std::vector<float> obs(in_dim, 0.0f);
  if (!obs_path.empty()) {
    std::vector<float> tmp;
    if (!read_bin_f32(obs_path, tmp)) {
      std::cerr << "FAILED: read obs file: " << obs_path << "\n";
      return 3;
    }
    if (static_cast<int>(tmp.size()) != in_dim) {
      std::cerr << "FAILED: obs dim mismatch. file=" << tmp.size() << " expected=" << in_dim << "\n";
      return 4;
    }
    obs = std::move(tmp);
  }

  std::vector<float> action(out_dim, 0.0f);

  // Warmup
  for (int i = 0; i < warmup; ++i) {
    if (!driver.run(obs.data(), in_dim, action.data(), out_dim)) {
      std::cerr << "FAILED: run during warmup\n";
      return 5;
    }
    // 아주 작은 변화로 완전 동일 입력 반복 최적화/캐싱 같은 오해 방지(의미상 거의 영향 없음)
    obs[0] = obs[0] + 1e-9f;
  }

  // Benchmark
  std::vector<double> us;
  us.reserve(static_cast<size_t>(iters));

  for (int i = 0; i < iters; ++i) {
    const auto t0 = std::chrono::steady_clock::now();
    const bool ok = driver.run(obs.data(), in_dim, action.data(), out_dim);
    const auto t1 = std::chrono::steady_clock::now();
    if (!ok) {
      std::cerr << "FAILED: run during benchmark\n";
      return 6;
    }
    const double dt_us =
        std::chrono::duration_cast<std::chrono::duration<double, std::micro>>(t1 - t0).count();
    us.push_back(dt_us);

    // 입력을 아주 미세하게 흔들어줌
    obs[0] = obs[0] + 1e-9f;
  }

  // Stats
  std::vector<double> us_sorted = us;
  std::sort(us_sorted.begin(), us_sorted.end());

  const double sum = std::accumulate(us.begin(), us.end(), 0.0);
  const double mean = sum / static_cast<double>(us.size());
  const double p50 = percentile_sorted(us_sorted, 50.0);
  const double p90 = percentile_sorted(us_sorted, 90.0);
  const double p99 = percentile_sorted(us_sorted, 99.0);
  const double mx = us_sorted.back();
  const double mn = us_sorted.front();

  std::cout << "model: " << model_path << "\n";
  if (!obs_path.empty()) std::cout << "obs:   " << obs_path << "\n";
  std::cout << "in_dim=" << in_dim << " out_dim=" << out_dim << "\n";
  std::cout << "iters=" << iters << " warmup=" << warmup << "\n";
  std::cout << "latency_us: mean=" << mean
            << " p50=" << p50
            << " p90=" << p90
            << " p99=" << p99
            << " max=" << mx
            << " min=" << mn
            << "\n";

  // 참고용: 마지막 action 일부 출력
  std::cout << "action[0..5]: ";
  for (int i = 0; i < std::min(6, out_dim); ++i) std::cout << action[i] << " ";
  std::cout << "\n";

  return 0;
}
