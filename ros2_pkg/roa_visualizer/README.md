# roa_visualizer

`roa_visualizer` is a ROS 2 Humble `ament_python` package that converts PACE command topics into `sensor_msgs/JointState` messages for RViz visualization.

## Topics

### Subscribed

* `/hardware_interface/command`
  Type: `roa_interfaces/msg/MotorCommandArray`
  Uses the `position` field of non-RSU motor commands.

* `/rsu/target`
  Type: `roa_interfaces/msg/RsuTarget`
  Uses the virtual ankle roll and pitch targets.

### Published

* `/joint_states`
  Type: `sensor_msgs/msg/JointState`

## Motor Mapping

| Motor ID | URDF Joint       |
| -------: | ---------------- |
|        9 | torso_yaw        |
|       10 | left_hip_pitch   |
|       11 | right_hip_pitch  |
|       12 | left_hip_roll    |
|       13 | right_hip_roll   |
|       14 | left_hip_yaw     |
|       15 | right_hip_yaw    |
|       16 | left_knee_pitch  |
|       17 | right_knee_pitch |

## Mesh Setup

Copy the STL files from the original description package into the `meshes` directory.

```bash
cp <description_package>/meshes/*.stl \
  <colcon_ws>/src/roa_visualizer/meshes/
```

The URDF mesh paths should use the package URI format:

```xml
<mesh filename="package://roa_visualizer/meshes/base_link.stl"/>
```

## Build

```bash
cd ~/colcon_ws

rosdep install --from-paths src --ignore-src -r -y

colcon build \
  --symlink-install \
  --packages-select roa_visualizer

source install/setup.bash
```

## Run

Launch RViz, `robot_state_publisher`, and the visualization node:

```bash
ros2 launch roa_visualizer pace_visualization.launch.py
```

Use an external URDF:

```bash
ros2 launch roa_visualizer pace_visualization.launch.py \
  urdf_path:=/absolute/path/to/roa_deploy.urdf
```

Run only the conversion node:

```bash
ros2 run roa_visualizer command_joint_state_publisher
```

## Parameters

```bash
ros2 run roa_visualizer command_joint_state_publisher --ros-args \
  -p publish_rate_hz:=100.0 \
  -p command_topic:=/hardware_interface/command \
  -p rsu_target_topic:=/rsu/target \
  -p joint_state_topic:=/joint_states
```

## Debugging

```bash
ros2 topic echo /joint_states
ros2 topic hz /joint_states
ros2 topic info /hardware_interface/command -v
ros2 topic info /rsu/target -v
```

## Notes

The current URDF defines `torso_yaw` as a fixed joint, so motor ID 9 will not move in RViz unless the joint type is changed to `revolute`.

RViz displays the robot configuration but does not automatically detect self-collisions.
