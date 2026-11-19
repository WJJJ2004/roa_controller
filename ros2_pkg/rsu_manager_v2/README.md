# rsu_manager_v2

C++ real-time-oriented replacement for the Python `rsu_manager` control path.
It intentionally preserves the existing external topics and message types:

- input `/rsu/target` (`roa_interfaces/msg/RsuTarget`)
- input `/hardware_interface/state` (`MotorStateArray`)
- output `/rsu/imp_solution` (`RsuImpSol`), fixed 200 Hz
- output `/rsu/state` (`RsuStateArray`), fixed 200 Hz

The node is one process with three parallel workers:

```text
ROS target callback -> atomic immutable latch
                         -> 200 Hz target worker, IK only for a new target
                         -> target snapshot (held between 50 Hz inference ticks)

ROS feedback callback -> bounded SPSC queue
                         -> 400 Hz state worker drains every received sample
                         -> LUT estimator -> Jacobian -> Kp/Kd
                         -> state/impedance snapshot

target snapshot + state snapshot
                         -> fixed 200 Hz command worker
                         -> /rsu/imp_solution and /rsu/state
```

The Jacobian and stateful impedance mappers are owned only by the state worker.
The command worker receives completed Kp/Kd snapshots, never a mutable Jacobian.

## Filter-free virtual velocity (October 7 deployment setup)

The LUT supplies motor angles and the local Jacobian, not stored velocity.
After reconstructing virtual q from measured motor positions, the estimator
publishes `(J^T J + 3e-7 I)^-1 J^T motor_qd` with the original
+/-5.235987756 rad/s observation clamp. Numerical damping and
finite/state-validity checks remain. Position-difference mixing and the 1.5 Hz
temporal LPF are removed.
Previous q is retained as the inverse-position solver seed, not for velocity
differentiation. Target workspace/actuator limits and impedance guards remain.

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

The default YAML requests twice the original virtual gains: pitch Kp/Kd
`50 / 2.4`, roll Kp/Kd `68.5 / 3.288` (roll/pitch ratio `1.37`). Fixed motor
bounds are Kp `[5, 45]` and Kd `[0.2, 2.5]`. They cover tuning from 1x to 3x
the original pitch gains `25 / 1.2` with that axis ratio unchanged. A 0.0625
degree grid sweep (687,921 poses per scale) found a maximum motor Kp/Kd of
`41.3854 / 1.9865` at 3x, without bound activation. This is an offline sampled
mapping check, not a hardware stability guarantee. The main controller accepts
RSU solutions only when `0 < kp < 50` and `0 < kd < 5`.

System-manager boot and main-controller initial/debug holds share
`roa_common/pose.hpp`: ankle actuator Kp/Kd are `31.5 / 5.0`, twice their prior
`15.75 / 2.5`. These fixed commands bypass the RSU-solution gain check; the
remaining joints retain their gains. The historical `chirp_replay.yaml`
explicitly retains its original gains and limits for baseline comparisons.

## Run

```bash
ros2 launch rsu_manager_v2 rsu_manager_v2.launch.py
```

Do not run the legacy `rt_imp_solver_node` or `rsu_lut_rt` concurrently because
all three publish the same external RSU topics.

The current implementation improves scheduling and consistency but does not
request `SCHED_FIFO`, lock process memory, or pin CPU cores. Those should only
be enabled after measuring the complete controller/hardware thread priorities.
