# rsu_manager_v2

C++ real-time-oriented replacement for the Python `rsu_manager` control path.
It intentionally preserves the existing external topics and message types:

- input `/rsu/target` (`roa_interfaces/msg/RsuTarget`)
- input `/hardware_interface/state` (`MotorStateArray`)
- output `/rsu/imp_solution` (`RsuImpSol`), fixed 300 Hz
- output `/rsu/state` (`RsuStateArray`), fixed 300 Hz

The node is one process with three parallel workers:

```text
ROS target callback -> atomic immutable latch
                         -> 300 Hz target worker, IK only for a new target
                         -> target snapshot (held between 50 Hz inference ticks)

ROS feedback callback -> bounded SPSC queue
                         -> 600 Hz state worker drains every received sample
                         -> LUT estimator -> Jacobian -> Kp/Kd
                         -> state/impedance snapshot

target snapshot + state snapshot
                         -> fixed 300 Hz command worker
                         -> /rsu/imp_solution and /rsu/state
```

The Jacobian and stateful impedance mappers are owned only by the state worker.
The command worker receives completed Kp/Kd snapshots, never a mutable Jacobian.

## Clipping and validation

The selected internal virtual workspace was certified by the supplied
`rsu_joint_limits.npz` sweep:

- roll `[-18.0, 17.5]` degrees
- pitch `[-36.0, 39.5]` degrees

The complete rectangle passes IK margin, Jacobian, impedance fit, coupling and
non-saturation criteria. Left external limits equal the internal limits. Right
external limits are mirrored: roll `[-17.5, 18.0]`, pitch `[-39.5, 36.0]`.

Targets are checked for finite values and clipped in external virtual-joint
space before mirroring and LUT IK. Actuator targets are clipped again to the
LUT-derived actuator extrema. Feedback is never clipped: a value outside those
extrema plus the configured two-degree measurement margin invalidates the
state, so a sensor/calibration fault is not hidden.

The v2 Kd upper bound is 4.9, deliberately below the existing main
controller's strict `kd < 5.0` acceptance check. The validated trajectory is
around 0.57--0.58, so this interface guard does not alter the measured mapping.

## Run

```bash
ros2 launch rsu_manager_v2 rsu_manager_v2.launch.py
```

Do not run the legacy `rt_imp_solver_node` or `rsu_lut_rt` concurrently because
all three publish the same external RSU topics.

The current implementation improves scheduling and consistency but does not
request `SCHED_FIFO`, lock process memory, or pin CPU cores. Those should only
be enabled after measuring the complete controller/hardware thread priorities.
