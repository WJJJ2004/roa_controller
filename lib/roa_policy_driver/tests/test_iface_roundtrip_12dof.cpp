#include <cstdint>
#include <cstring>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include "roa_policy_driver/interfaces/policy_12dof_v1.hpp"

static bool read_f32_bin(const std::string& path, std::vector<float>& out) {
  std::ifstream ifs(path, std::ios::binary);
  if (!ifs) return false;

  ifs.seekg(0, std::ios::end);
  const std::streamsize nbytes = ifs.tellg();
  ifs.seekg(0, std::ios::beg);

  if (nbytes <= 0) return false;
  if (nbytes % static_cast<std::streamsize>(sizeof(float)) != 0) return false;

  const size_t n = static_cast<size_t>(nbytes / sizeof(float));
  out.resize(n);

  if (!ifs.read(reinterpret_cast<char*>(out.data()), nbytes)) return false;
  return true;
}

int main(int argc, char** argv) {
  using Spec = roa::policy::iface::Policy12DofV1;

  std::string obs_path = "onnx/12dof/data/golden_obs.bin";
  if (argc >= 2) obs_path = argv[1];

  std::vector<float> obs_packed;
  if (!read_f32_bin(obs_path, obs_packed)) {
    std::cerr << "FAILED: read obs: " << obs_path << "\n";
    return 1;
  }

  if (obs_packed.size() != static_cast<size_t>(Spec::kObsDim)) {
    std::cerr << "FAILED: obs size mismatch. got " << obs_packed.size()
              << " expected " << Spec::kObsDim << "\n";
    return 2;
  }

  // unpack -> pack roundtrip
  Spec::Obs obs_struct{};
  Spec::unpack_obs(obs_packed.data(), obs_struct);

  std::vector<float> repacked(static_cast<size_t>(Spec::kObsDim), 0.0f);
  Spec::pack_obs(obs_struct, repacked.data());

  const int cmp = std::memcmp(
      obs_packed.data(), repacked.data(),
      sizeof(float) * static_cast<size_t>(Spec::kObsDim));

  if (cmp != 0) {
    std::cerr << "FAILED: iface roundtrip mismatch (memcmp != 0)\n";
    // 디버그용: 첫 mismatch 인덱스 출력
    for (int i = 0; i < Spec::kObsDim; ++i) {
      if (obs_packed[static_cast<size_t>(i)] != repacked[static_cast<size_t>(i)]) {
        std::cerr << " first_diff i=" << i
                  << " orig=" << obs_packed[static_cast<size_t>(i)]
                  << " repacked=" << repacked[static_cast<size_t>(i)] << "\n";
        break;
      }
    }
    return 3;
  }

  std::cout << "PASS\n";
  return 0;
}