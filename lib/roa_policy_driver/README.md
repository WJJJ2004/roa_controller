# roa_policy_driver







## Requirements

* Ubuntu 22.04

* C++17

* ONNX Runtime 1.23.2 (prebuilt tgz, CPU-only)

  * 설치 경로: `/opt/onnxruntime/current`

  * 헤더: `/opt/onnxruntime/current/include/onnxruntime_cxx_api.h`

  * 라이브러리: `/opt/onnxruntime/current/lib/libonnxruntime.so`

## Repository Layout

* `onnx/12dof/policy.onnx` : 기본 테스트 대상 모델

* `include/roa_policy_driver/policy_driver.hpp` : public API

* `src/policy_driver.cpp` : 구현

* `tests/` : `dimension/golden/stress` 테스트

## How To Build

```bash
git clone https://github.com/WJJJ2004/roa_policy_driver
cd roa_policy_driver
rm -rf build
mkdir build && cd build
cmake .. -DROA_BUILD_TESTS=ON
cmake --build . -j
```

## Run Tests

```bash
cd build
ctest --output-on-failure
```

테스트 구성:

1. `test_dimensions`: 모델 로드 및 input/output dim 검증

2. `test_golden_io`: `golden_obs.bin`을 입력했을 때 `golden_action.bin`과 동일 출력인지 검증

3. `test_stress`: 반복 실행 안정성(메모리/세션) 검증

## Install & Use

```bash
cd build
sudo cmake --install .
sudo ldconfig
```

설치 후 외부 CMake 프로젝트에서 find_package(roa_policy_driver)로 사용할 수 있다.

```cmake
CMake Integration (Consumer)
find_package(roa_policy_driver REQUIRED)

add_executable(my_app main.cpp)
target_link_libraries(my_app PRIVATE roa::roa_policy_driver)
```
## quick setup

```bash
rm -rf build
mkdir build && cd build
cmake ..
cmake --build . -j
sudo cmake --install .
sudo ldconfig
```


## API Usage

헤더: `#include <roa_policy_driver/policy_driver.hpp>`

기본 사용 흐름:

1. `PolicyDriver` 생성

2. `load(model_path, options)`로 ONNX 로드

3. 실시간 루프에서 `run(obs, action)` 호출

### Example
```cpp
#include <array>
#include <iostream>
#include <roa_policy_driver/policy_driver.hpp>

int main() {
  roa::policy::PolicyDriver driver;
  roa::policy::Options opt;  // 기본 옵션

  const std::string model_path =
      "onnx/12dof/policy.onnx";

  if (!driver.load(model_path, opt)) {
    std::cerr << "Failed to load model: " << model_path << "\n";
    return 1;
  }

  const int in_dim = driver.input_dim();
  const int out_dim = driver.output_dim();

  std::vector<float> obs(in_dim, 0.0f);
  std::vector<float> action(out_dim, 0.0f);

  // obs 채우기 (사용자 로직)
  // obs = [cmd(3), q_rel(13), qd_rel(13), imu_omega(3), last_action(13)] (예시)

  if (!driver.run(obs.data(), in_dim, action.data(), out_dim)) {
    std::cerr << "Policy inference failed\n";
    return 2;
  }

  std::cout << "action[0]=" << action[0] << "\n";
  return 0;
}

```
### Example (with wrapper)

```cpp
#include <array>
#include <iostream>

#include <roa_policy_driver/policy_driver.hpp>
#include <roa_policy_driver/interfaces/policy_10dof_v1.hpp>

int main() {
  using Spec = roa::policy::iface::Policy10DofV1;

  roa::policy::PolicyDriver driver;
  roa::policy::Options opt;  // 기본 옵션

  const std::string model_path = "onnx/10dof/policy.onnx";

  if (!driver.load(model_path, opt)) {
    std::cerr << "Failed to load model: " << model_path << "\n";
    return 1;
  }

  // dimension safety check (권장)
  if (driver.input_dim() != Spec::kObsDim ||
      driver.output_dim() != Spec::kActDim) {
    std::cerr << "Model dimension mismatch with Spec\n";
    return 2;
  }

  // Structured observation 생성
  Spec::Obs obs{};
  obs.cmd = {0.3f, 0.0f, 0.1f};

  for (int i = 0; i < Spec::kDof; ++i) {
    obs.q_rel[i] = 0.0f;
    obs.qd_rel[i] = 0.0f;
    obs.last_action[i] = 0.0f;
  }

  obs.imu_omega_body = {0.0f, 0.0f, 0.0f};

  // Pack → flat float buffer
  float obs_buffer[Spec::kObsDim];
  Spec::pack_obs(obs, obs_buffer);

  // Inference
  float action_buffer[Spec::kActDim];
  if (!driver.run(obs_buffer, Spec::kObsDim,
                  action_buffer, Spec::kActDim)) {
    std::cerr << "Policy inference failed\n";
    return 3;
  }

  // Unpack action
  Spec::Act act{};
  Spec::unpack_act(action_buffer, act);

  std::cout << "action[0] = " << act.action[0] << "\n";

  return 0;
}
```

## Realtime Usage Notes

* `load()`는 초기화 단계에서 1회만 호출한다 (configure 단계)
  
* 제어 루프(update)에서는 `run()`만 호출한다

* 성공 경로에서 동적 할당이 없도록(obs/action 버퍼는 외부에서 고정 크기로 유지 권장)
  
* `cmd/last_action` 등의 입력은 `controller` 측에서 realtime-safe하게 관리한다

## Updating the Policy Model

새 모델로 교체할 때는 다음을 함께 갱신해야 한다.

* 모델 파일 경로

* golden 테스트 데이터:

  * `onnx/12dof/data/golden_obs.bin`

  * `onnx/12dof/data/golden_action.bin`

golden 데이터는 “해당 모델 + 해당 obs 정의” 조합에서 생성된 값이어야 한다.
