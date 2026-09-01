# rsu_lut_rt

Real-time-only RSU pipeline. It deliberately contains no sample generation,
RViz, gamepad, tuning, or hardware-test nodes.

External protocol compatibility:

- subscribes `/rsu/target` (`roa_interfaces/msg/RsuTarget`)
- subscribes `/hardware_interface/state` (`MotorStateArray`)
- publishes `/rsu/imp_solution` (`RsuImpSol`)
- publishes `/rsu/state` (`RsuStateArray`)

The state process owns feedback reconstruction, Jacobian, and impedance mapper
state. It sends an immutable `RsuImpSol` gain snapshot on the private
`/rsu_lut/gain_snapshot` topic. The solution process owns target IK and branch
continuity. There is no shared mutable Jacobian data between processes.

Both timers poll at 600 Hz so that an asynchronous 300 Hz source is not lost
through equal-frequency phase aliasing. Expensive work and publication happen
only for a new source timestamp (`MotorStateArray`) or sequence (`RsuTarget`),
so the external output rates follow the 300 Hz feedback and 50 Hz policy target
rather than the polling rate.

```bash
ros2 launch rsu_lut_rt rt.launch.py
```
