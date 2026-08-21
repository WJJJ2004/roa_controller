# roa_hw_joint_test

Feedback을 사용하지 않고 고정된 hip-pitch 파형을 재생합니다.

1. 모든 관절을 0으로 10초 유지
2. Hip pitch를 5초 동안 +5도로 이동
3. +5도에서 3초 유지
4. 5초 동안 0도로 복귀
5. 5초 동안 -5도로 이동
6. -5도에서 3초 유지
7. 5초 동안 0도로 복귀한 뒤 계속 0 유지

Hip pitch의 좌우 부호와 10/12, 11/13 결합축 변환은
`pace_symmetric_sample_generator_node`와 같습니다. Hip을 제외한 ID 9~17은
항상 0을 명령하고 `/rsu/target`의 네 발목 가상 관절도 항상 0입니다.

```bash
source install/setup.bash
ros2 launch roa_hw_joint_test joint_test.launch.py
```

RViz가 필요 없으면 다음처럼 실행합니다.

```bash
ros2 launch roa_hw_joint_test joint_test.launch.py visualize:=false
```

진폭만 바꾸려면 다음처럼 실행합니다.

```bash
ros2 launch roa_hw_joint_test joint_test.launch.py amplitude_deg:=3.0
```

`roa_main_controller`나 sample generator 등 다른
`/hardware_interface/command` publisher는 반드시 종료한 상태에서 실행해야
합니다.
