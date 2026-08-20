# ROA Controller

ROS 2-based humanoid control stack integrating reinforcement learning policy inference, RobStride motor control, RSU (Revolute–Spherical–Universal) ankle kinematics, IMU feedback, and lifecycle-based system management.

> [!CAUTION]
> This repository sends position, velocity, and gain commands to physical motors. Before running on hardware, secure the robot on a support frame and verify the CAN interfaces, motor IDs, initial pose, RSU seeds, and emergency-stop procedure. `roa_main_controller.launch.py` starts the controller in real-time control mode.

## Development Environment

| Component | Reference Environment |
| --- | --- |
| OS | Ubuntu 22.04 |
| ROS 2 | Humble Hawksbill |
| Python | 3.10 |
| C++ | C++17 |
| ONNX Runtime | 1.23.2, CPU |
| IMU | EBIMU |
| Motor | RobStride over SocketCAN |

These versions reflect the current development and validation environment. Other environments are not guaranteed to work.

## System Architecture

```text
 /ctrl/policy_cmd        /imu/data, /imu/gravity
          │                         │
          └──────────┬──────────────┘
                     ▼
          ┌─────────────────────┐
          │ roa_main_controller │◀────────── /rsu/state
          │  observation → ONNX │◀────────── /rsu/imp_solution
          └─────────┬───────────┘
                    │ /rsu/target
                    ▼
             ┌─────────────┐
             │ rsu_manager │
             └──────┬──────┘
                    │ actuator solution
                    ▼
          /hardware_interface/command
                    │
                    ▼
       ┌───────────────────────────────┐
       │ robstride_hardware_interface  │
       └──────────────┬────────────────┘
                      │
          /hardware_interface/state
                      │
                      └──────────► controller / RSU solver

 roa_system_manager
   ├─ configures, activates, and deactivates the main controller
   ├─ holds the initial motor pose during startup
   └─ manages RUN, SAFE_HOLD, and ESTOP states
```

The main controller and RobStride hardware interface are ROS 2 lifecycle nodes. The system manager controls only the **main controller lifecycle**. The hardware interface must be configured and activated separately.

## Repository Structure

| Path | Description | Document |
| --- | --- | --- |
| `lib/roa_policy_driver` | ONNX Runtime policy loader and typed policy interfaces | [README](./lib/roa_policy_driver/README.md) |
| `lib/roa_packet_manager` | Motor command packet builder and state decoder | [README](./lib/roa_packet_manager/README.md) |
| `lib/roa_common` | Shared constants and initial pose definitions | — |
| `msgs/roa_interfaces` | Motor, RSU, and system-status ROS messages | — |
| `ros2_pkg/roa_main_controller` | Observation construction, policy inference, and actuator command generation | [README](./ros2_pkg/roa_main_controller/README.md) |
| `ros2_pkg/roa_system_manager` | Controller lifecycle and safety-state management | [README](./ros2_pkg/roa_system_manager/README.md) |
| `ros2_pkg/robstride_hardware_interface` | SocketCAN-based RobStride motor interface | [README](./ros2_pkg/robstride_hardware_interface/README.md) |
| `ros2_pkg/rsu_manager` | RSU inverse kinematics, state estimation, impedance mapping, and test nodes | [README](./ros2_pkg/rsu_manager/README.md) |
| `ros2_pkg/imu_pkg` | EBIMU serial driver and ROS publisher | [README](./ros2_pkg/imu_pkg/README.md) |
| `ros2_pkg/roa_cmd_test` | Gamepad-based velocity command test node | — |
| `ros2_pkg/roa_visualizer` | RViz bridge for motor and RSU commands | [README](./ros2_pkg/roa_visualizer/README.md) |
| `shell` | ONNX Runtime, udev, and full-build scripts | — |
| `docs` | System diagrams and demonstration material | — |

`imu_pkg` and `robstride_hardware_interface` are Git submodules.

## Installation

### 1. Clone the Repository

```bash
mkdir -p ~/colcon_ws/src
cd ~/colcon_ws/src
git clone --recursive https://github.com/WJJJ2004/roa_controller.git
```

If the repository was cloned without `--recursive`, initialize the submodules separately:

```bash
cd ~/colcon_ws/src/roa_controller
git submodule update --init --recursive
```

### 2. Install ROS and Python Dependencies

Source ROS 2 Humble before installing dependencies:

```bash
source /opt/ros/humble/setup.bash
cd ~/colcon_ws
rosdep update
rosdep install --from-paths src --ignore-src -r -y
python3 -m pip install --user numpy pyserial PyYAML
```

### 3. Install ONNX Runtime

The policy driver expects ONNX Runtime under `/opt/onnxruntime/current`. The provided script installs ONNX Runtime 1.23.2 CPU and updates the system linker configuration.

```bash
cd ~/colcon_ws/src/roa_controller
./shell/install_onnxruntime.sh
```

The script modifies `apt`, `/opt`, and `/etc/ld.so.conf.d` and therefore requires `sudo` access.

### 4. Install the EBIMU udev Rule

```bash
cd ~/colcon_ws/src/roa_controller
./shell/install_udev_rules.sh
ls -l /dev/ttyUSB-EBIMU
```

The IMU may need to be disconnected and reconnected after installing the rule. The default serial configuration is defined in `ros2_pkg/imu_pkg/config/params.yaml`.

## Build and Test

The full-build script performs the following steps:

1. Builds `roa_policy_driver` in Release mode
2. Runs the policy driver CTest suite
3. Installs the policy driver into `install/roa_policy_driver`
4. Builds the remaining ROS 2 packages with colcon

```bash
source /opt/ros/humble/setup.bash
cd ~/colcon_ws/src/roa_controller
./shell/build_all.sh
source ~/colcon_ws/install/setup.bash
```

> [!NOTE]
> The script removes and recreates `lib/roa_policy_driver/build` and `install/roa_policy_driver`. The current policy-driver CTest configuration primarily runs the interface round-trip test. Some golden and stress tests described in the package README are disabled in CMake.

Verify the build:

```bash
colcon list --base-paths ~/colcon_ws/src/roa_controller
ctest --test-dir ~/colcon_ws/src/roa_controller/lib/roa_policy_driver/build \
  --output-on-failure
```

## Configuration

Review at least the following files before running on hardware:

| File | Main Settings |
| --- | --- |
| `ros2_pkg/robstride_hardware_interface/config/motor_setting.yaml` | CAN interface names, motor IDs and types, 150 Hz control rate |
| `ros2_pkg/roa_main_controller/config/params.yaml` | 100 Hz hardware command rate, 50 Hz policy rate, timeouts, and topics |
| `ros2_pkg/rsu_manager/config/rsu_imp_rt.yaml` | Non-neutral RSU seeds and impedance gains |
| `ros2_pkg/rsu_manager/config/rsu_imp_sample.yaml` | Neutral seeds for sample collection |
| `ros2_pkg/imu_pkg/config/params.yaml` | Serial port, unit conversion, and 50 Hz publish rate |
| `lib/roa_common/include/roa_common/constants.hpp` | Initial pose and policy reference joint angles |

The main controller uses `Policy12DofV2` with `lib/roa_policy_driver/onnx/12dof_guv_v2/policy.onnx`. The observation order, default joint angles, and action scale must remain consistent with the training environment.

## Run

### RViz Visualization

Use the visualizer to inspect the URDF and command mapping without connecting the motor hardware:

```bash
source ~/colcon_ws/install/setup.bash
ros2 launch roa_visualizer display.launch.py
```

The visualizer converts `/hardware_interface/command` and `/rsu/target` into `/joint_states`.

### IMU

```bash
ros2 launch imu_pkg imu.launch.py
ros2 topic hz /imu/data
ros2 topic echo /imu/gravity
```

Use `debug.launch.py` to start the IMU publisher with RViz.

### Main Controller in Debug Mode

The debug launch file creates the controller with `REALTIME_CONTROL_MODE=False`. Lifecycle transitions must be requested manually.

```bash
ros2 launch roa_main_controller debug.launch.py
```

In another terminal:

```bash
ros2 lifecycle set /roa_main_controller configure
ros2 lifecycle set /roa_main_controller activate
ros2 lifecycle get /roa_main_controller
```

The controller may report a fault when required input topics are missing or stale.

### RSU Solver

Run the standard impedance solver:

```bash
ros2 launch rsu_manager solver_node.launch.py
```

Run the symmetric chirp sample generator with neutral seeds:

```bash
ros2 launch rsu_manager sample_generate.launch.py
```

`hw_test.launch.py`, `imp_hw_test.launch.py`, and `init_pose_tunning.launch.py` are hardware test launch files that generate actuator commands. Do not run them without first validating the configuration and physical robot state.

## Hardware Startup Sequence

The following sequence reflects the current package architecture. Run each long-lived command in a separate terminal and source the workspace in every terminal.

### 1. Configure CAN

The USB-CAN devices must be available under the following interface names:

```text
can_right_leg
can_left_leg
can_right_arm
can_left_arm
```

```bash
cd ~/colcon_ws/src/roa_controller
./ros2_pkg/robstride_hardware_interface/scripts/can_setup.sh
ip -details link show type can
```

### 2. Start and Activate the Hardware Interface

```bash
ros2 launch robstride_hardware_interface robstride.launch.py
```

In another terminal, verify the state and perform the lifecycle transitions:

```bash
ros2 lifecycle set /hardware_interface_node configure
ros2 lifecycle set /hardware_interface_node activate
ros2 lifecycle get /hardware_interface_node
```

Do not continue if the configure transition reports a CAN-open or motor-configuration failure.

### 3. Start the Sensors and RSU Solver

```bash
ros2 launch imu_pkg imu.launch.py
```

```bash
ros2 launch rsu_manager solver_node.launch.py
```

Confirm that the initial seeds in `rsu_imp_rt.yaml` match the current mechanism pose before starting the solver.

### 4. Start the Main Controller

```bash
ros2 launch roa_main_controller roa_main_controller.launch.py
```

This launch file sets `REALTIME_CONTROL_MODE=True`, but the lifecycle node initially remains unconfigured. Do not activate it manually when using the system manager.

### 5. Start the System Manager

```bash
ros2 launch roa_system_manager system_manager.launch.py
```

The system manager follows this sequence:

```text
BOOT
  → configure controller
  → verify ready / healthy / rt_ok
  → hold the initial pose through /hardware_interface/command
  → wait for walk_initialized=True from the hardware interface
  → activate controller
  → RUN
  → recoverable fault: SAFE_HOLD
  → critical fault or recovery timeout: ESTOP
```

The `init_pos_timeout_sec` parameter is defined, but the initial-pose timeout handling is currently disabled in code. The manager may remain in BOOT indefinitely if `walk_initialized` is not received.

### 6. Publish Velocity Commands

```bash
ros2 run roa_cmd_test roa_cmd_test
```

Verify that commands are published on `/ctrl/policy_cmd`.

## Topics

| Topic | Type | Description |
| --- | --- | --- |
| `/ctrl/policy_cmd` | `geometry_msgs/msg/Twist` | User velocity command to the controller |
| `/imu/data` | `sensor_msgs/msg/Imu` | IMU measurement to the controller |
| `/imu/gravity` | `geometry_msgs/msg/Vector3Stamped` | Gravity vector to the controller |
| `/rsu/target` | `roa_interfaces/msg/RsuTarget` | Virtual ankle target from the controller to the RSU solver |
| `/rsu/imp_solution` | `roa_interfaces/msg/RsuImpSol` | RSU actuator solution to the controller |
| `/rsu/state` | `roa_interfaces/msg/RsuStateArray` | Virtual RSU joint state to the controller |
| `/hardware_interface/command` | `roa_interfaces/msg/MotorCommandArray` | Motor command from the controller or system manager |
| `/hardware_interface/state` | `roa_interfaces/msg/MotorStateArray` | Motor feedback to the controller and RSU solver |
| `/controller/status` | `roa_interfaces/msg/SystemStatus` | Controller health and state to the system manager |
| `/hardware_interface/etop` | `std_msgs/msg/Bool` | Hardware torque and emergency-stop input |
| `/walk_initialized` | `std_msgs/msg/Bool` | Initial-pose completion signal from the hardware interface |
| `/joint_states` | `sensor_msgs/msg/JointState` | Joint state output from the visualizer |

Review both the package YAML files and code defaults when changing topic names or timeouts. Some test nodes use fixed topic names in code.

## Diagnostics and Shutdown

```bash
ros2 node list
ros2 lifecycle nodes
ros2 lifecycle get /hardware_interface_node
ros2 lifecycle get /roa_main_controller
ros2 topic hz /hardware_interface/state
ros2 topic hz /controller/status
ros2 topic echo /controller/status
```

For a normal hardware shutdown, stop command input, deactivate the lifecycle nodes, and then stop the processes:

```bash
ros2 lifecycle set /roa_main_controller deactivate
ros2 lifecycle set /hardware_interface_node deactivate
```

In an unsafe condition, follow the physical emergency-stop and power-disconnection procedure before attempting a software shutdown.

## Known Limitations

- There is no single integrated launch file for the complete system.
- The system manager controls the main controller lifecycle but does not control the hardware-interface lifecycle.
- The initial-pose completion timeout is currently disabled.
- The `imu_ui` entry point registered by `imu_pkg` does not have a corresponding implementation file in the current submodule.
- Some package metadata and Python dependency declarations are incomplete, so dependencies may be required in addition to those installed by `rosdep`.
- Several package-level README files are incomplete or older than the current implementation. Use this document together with the actual launch and configuration files.
- Hardware-in-the-loop behavior and timing guarantees must be validated for each hardware and CAN configuration.

## License

The top-level repository is licensed under the [MIT License](./LICENSE). Submodules, individual packages, models, and external libraries may use different licenses. Review each component before redistribution.
