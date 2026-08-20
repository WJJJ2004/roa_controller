# roa_packet_manager

---

## Actuator-Space PD Gains

The following table summarizes the current PD gains used in the actuator coordinate space.

| Joint / Actuator | Motor ID | Kp | Kd |
|---|---:|---:|---:|
| Torso Yaw | 9 | 50.0 | 2.0 |
| Left Hip Pitch | 10 | 150.0 | 24.722 |
| Right Hip Pitch | 11 | 150.0 | 24.722 |
| Left Hip Roll | 12 | 200.0 | 26.387 |
| Right Hip Roll | 13 | 200.0 | 26.387 |
| Left Hip Yaw | 14 | 100.0 | 3.419 |
| Right Hip Yaw | 15 | 100.0 | 3.419 |
| Left Knee Pitch | 16 | 150.0 | 8.654 |
| Right Knee Pitch | 17 | 150.0 | 8.654 |
| Left RSU Upper Actuator | 18 | 40.0 | 0.99 |
| Right RSU Upper Actuator | 19 | 40.0 | 0.99 |
| Left RSU Lower Actuator | 20 | 40.0 | 0.99 |
| Right RSU Lower Actuator | 21 | 40.0 | 0.99 |

> **Note:**
> The hip and knee gains are directly defined in the corresponding actuator/joint coordinate space.  
> The RSU ankle mechanism consists of coupled upper and lower actuators; therefore, these actuator-space gains should not be directly compared with the ankle pitch/roll gains defined in the virtual joint space.