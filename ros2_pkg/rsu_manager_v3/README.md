# RSU manager v3 — 200 Hz direct motor torque

`/rsu/target` (`RsuTarget`) and `/rsu/state` (`RsuStateArray`) retain their
existing schemas, ordering, units, mirrored right-side convention and topic
names. The v2 estimator/LUT library is reused without numerical source changes.
Run **one** RSU manager in production. No legacy impedance solution is published
by v3.

```
walking policy -> /rsu/target -> v3 control -> /rsu/torque_command -> HW iface -> CAN
HW state -> unchanged v2 estimator -> /rsu/state -> walking policy
main controller -> existing full motor packet -> HW iface -> other motors
```

## Torque law and coordinates

At 200 Hz, with the latest valid measured state:

- `tau_v = diag(34.25,25)*(q_des-q) - diag(1.644,1.2)*qd` per foot.
- Existing targets contain positions only: desired velocity is zero. There is no
  implicit target differentiation or additional integral/friction compensation.
- LUT `A = d(alpha)/d(q)`. Solve `A^T tau_model = tau_v`; equivalently
  `tau_model = (dq/dalpha)^T tau_v`. The Jacobian is evaluated at measured q.
- Mirror both virtual and motor axes on the right. Their two signs cancel in
  the external-coordinate A. Model-to-protocol torque conversion is applied
  **once**, by the HW gate: negative sign for IDs 18,20,19,21, consistent with
  their existing position/velocity direction. Existing MotorCommand torque
  semantics are left intact; the direct topic explicitly uses model axes.
- Motor gains are zero in torque ownership. Motor current regulation/firmware
  remains active; “feedforward” refers to removal of motor position/velocity PD.

`RsuTorqueCommand` has a fixed ordering `[18,20,19,21]`, N m units, publication
header, original feedback/target headers, output sequence and target sequence,
virtual and pre-limit motor torque, applied torque, saturation and validity.
Fault bits: 1 target invalid/stale, 2 state invalid/stale, 4 mapping failure.
Invalid output has all four applied torques zero. No indefinite nonzero hold.

Initial software limits are 11.9 N m per virtual axis and motor, and 200 N m/s
motor slew; these are configurable starting envelopes, **not measured hardware
continuous ratings**. Saturation and slew can change transient demands and are
reported. A constant in-envelope demand does not decay. Ill-conditioned maps
are rejected, without regularized torque attenuation. State estimator damping
and velocity clipping remain as in v2.

## HW ownership and startup

Both main controller and HW iface take `rsu_control_mode: torque` (legacy default
is `impedance`). Main continues publishing existing target/state/full motor
interfaces but skips reliance on `/rsu/imp_solution`. No packet-manager torque
relay is used.

HW retains existing disabled-feedback collection and position initialization.
The fixed initialization pose is still an impedance command. At `RUNNING`, it
switches ankle ownership to the fresh v3 torque stream; the remaining motors
retain their original packet values. There is no automatic impedance fallback.
The gate is shared by the production HW code and socket-free test mock.
Ownership waits for the existing `/CTRL/RT_CONTROL` full-packet marker plus a
fresh torque stream, with a 100 ms readiness window. This preserves the existing
system-manager sequence (position initialization, then policy activation).
Applied HW torque also starts from zero with an independent 200 N m/s cap;
a manager that was already calculating torque cannot cause an immediate full
feedforward jump at handover. A new initialization-hold packet after torque
engagement requests the fault path, rather than switching back to impedance.

The gate checks publication/source ages (20 ms command, 20 ms state, 100 ms
target), receipt age on steady time, finite/bounded torque, strictly increasing
publication stamps, and exactly one entry for each ankle in the full packet.
HW also requires a fresh complete upper packet within 50 ms. Faults enter the
existing FROZEN/disable path; recover through lifecycle deactivate/activate,
which resets the gate. Before RUNNING invalid startup samples are discarded.
Emergency-stop and CAN fault paths retain precedence.

Source ROS timestamps require synchronized clocks. Replays need a matching ROS
clock or explicit retimestamping; receipt freshness alone does not make old
source data valid. Current MotorStateArray has only an aggregate timestamp;
per-motor CAN age remains a limitation inherited from the observation path.

## Build and walking integration

From the chosen source workspace, source Humble and build all consumers:

```bash
colcon build --packages-up-to rsu_manager_v3 robstride_hardware_interface roa_main_controller
colcon test --packages-select rsu_manager_v2 rsu_manager_v3 robstride_hardware_interface roa_packet_manager roa_main_controller
```

New message is additive; existing target/state and MotorCommand schemas are not
changed. Rebuild dependent packages and source exactly the intended overlay.
Root and `ros_ws/src/roa_controller` are separate source trees; do not mix binaries.

```bash
ros2 launch rsu_manager_v3 walking.launch.py
```

This starts v3 and the walking controller; it does not start physical HW by
default. The launch supports `start_hardware:=true`, an explicit hardware YAML,
and a shared `motor_torque_limit`. Hardware and main lifecycle transitions,
IMU/gravity, and existing walking system-manager/operator sequence are still
required. This launch does not activate hardware automatically. Torque limit
and timeout tuning must be reviewed for the actual supported robot.

## Local validation

The reproducible domain77 driver is `tools/test_domain77.py`. It refuses another
DDS domain, requires localhost-only discovery, audits an empty graph, and starts
only v2/v3 numerical managers and `rsu_torque_mock`. The mock uses the production
gate and actual CAN encoder with **no CAN socket or hardware lifecycle**.

Tests distinguish algebra/transport correctness from physical torque fidelity
and gait stability. 200 Hz mean cadence is not proof of hard real-time deadlines.
Physical torque attenuation, supported startup transfer and walking behavior
require separate physical validation; no hardware command is needed for these
local tests.

## Validated local overlay

This session built a separate overlay at
`/home/lwj/roa-sandbox/analysis/rsu_manager_v3_validation/install`.
From the sandbox root, source
`analysis/rsu_manager_v3_validation/env.sh` to select it explicitly.
`analysis/rsu_manager_v3_validation/build_local.py` records a dependency-order
CMake fallback for the local colcon subprocess-completion hang.

Legacy impedance-fit parameters in the copied observation worker retain its
existing validity behavior. The torque law is configured with `virtual_kp` and
`virtual_kd`; those are the v3 control gains.

The synthetic timing producer disables Python cyclic GC during the timed loop,
and the measurement-only torque subscriber uses depth2048 to capture epochs.
Production command QoS remains best-effort depth1 (latest-value semantics).
A low-depth diagnostic subscriber can miss samples even when the independent
HW consumer continues operating; report source sequence gaps separately.
