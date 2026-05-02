# ROA Controller

ROS2-based humanoid control stack integrating reinforcement learning policy inference, real-time motor control, and RSU ankle kinematics.

---

## Overview

This repository provides a complete control stack for a humanoid robot, designed for real-time execution on physical hardware while maintaining compatibility with simulation-based training pipelines.

The system integrates the following core components:

- Reinforcement learning policy inference using ONNX Runtime
- Real-time motor command generation via ROS2
- RSU (Revolute–Spherical–Universal) ankle inverse kinematics and state estimation
- Lifecycle-based system management and safety handling

The architecture is designed to support sim-to-real transfer, modular development, and robust hardware operation.

---

## System Architecture

The control pipeline is structured as follows:

```
        RL Policy (ONNX)
                ↓
     roa_policy_driver
                ↓
     roa_controller_node
        - observation build
        - policy inference
        - command generation
                ↓
   hardware_interface (actuator layer)
                ↓
             Robot

Additional pipeline:
rsu_manager → /rsu/state → controller
```

The controller aggregates sensor inputs, performs policy inference, and generates actuator commands in a real-time loop.

---

## Requirements

| Component       | Version            |
|----------------|-------------------|
| OS             | Ubuntu 22.04      |
| ROS2           | Humble            |
| Python         | 3.10              |
| ONNX Runtime   | 1.23 or higher    |

---

## Repository Structure

| Directory | Description | Document |
|----------|------------|----------|
| `lib/roa_policy_driver` | Reinforcement learning policy inference module (ONNX-based) | [README](./lib/roa_policy_driver/README.md) |
| `lib/roa_packet_manager` | Packet builder and decoder utilities for ROA motor interfaces. | [README](./lib/roa_packet_manager/README.md) |
| `msgs/roa_interfaces` | Custom ROS2 message definitions | [README](./msgs/roa_interfaces/README.md) |
| `ros2_pkg/roa_controller_node` | Main controller node handling inference and command generation | [README](./ros2_pkg/roa_controller_node/README.md) |
| `ros2_pkg/roa_system_manager` | Lifecycle-based system manager and state supervisor | [README](./ros2_pkg/roa_system_manager/README.md) |
| `ros2_pkg/rsu_manager` | RSU solver, estimator, and debugging tools | [README](./ros2_pkg/rsu_manager/README.md) |

---

## Control Pipeline

The control loop operates as follows:

1. Receive command input (e.g., velocity command)
2. Collect sensor data:
   - IMU
   - Motor states
   - RSU state
3. Construct observation vector for policy
4. Perform policy inference
5. Convert action output to joint targets
6. Publish motor commands to hardware interface

The system runs in a real-time loop with strict timing constraints.

---

## RSU Ankle System

The RSU ankle is modeled as a virtual 2-DOF joint and includes:

- Closed-form inverse kinematics solver
- State estimator based on damped Gauss-Newton optimization
- Velocity estimation via Jacobian-based pseudo-inverse
- Filtering and numerical stabilization for real-time operation

The RSU system enables accurate ankle control despite complex mechanical linkage constraints.

---

## Topics

| Topic | Type | Description |
|------|------|------------|
| `/hardware_interface/command` | MotorCommandArray | Actuator command output |
| `/hardware_interface/state` | MotorStateArray | Motor feedback input |
| `/rsu/state` | RsuStateArray | Estimated RSU joint state |
| `/rsu/solution` | RsuSolution | IK solver output |
| `/rsu/target` | RsuTarget | Desired RSU command |
| `/controller/status` | SystemStatus | Controller health and state |
| `/imu/data` | sensor_msgs/Imu | IMU measurements |

---

## Quick Start

### 1. Clone repository into ROS2 workspace

```bash
mkdir -p ~/colcon_ws/src
cd ~/colcon_ws/src

git clone https://github.com/WJJJ2004/roa_controller.git
```

---

### 2. Build all components

```bash
cd ~/colcon_ws

chmod +x src/roa_controller/shell/build_all.sh
./src/roa_controller/shell/build_all.sh
```

---

### 3. Source environment

```bash
source install/setup.bash
```

---

## Run

```bash
ros2 launch roa_controller_node roa_controller_node.launch.py
```

---

## Design Considerations

- Real-time performance prioritized through lightweight message passing and minimal latency pipelines
- Modular separation between policy inference, control logic, and hardware interface
- Robustness against sensor delays and communication dropouts
- Compatibility with both simulation and real hardware environments

---

## License

This project is licensed under the terms of the LICENSE file included in this repository.
