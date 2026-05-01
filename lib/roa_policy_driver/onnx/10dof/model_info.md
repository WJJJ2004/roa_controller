[INFO] Command Manager:  <CommandManager> contains 1 active terms.
+------------------------------------------------+
|              Active Command Terms              |
+-------+---------------+------------------------+
| Index | Name          |          Type          |
+-------+---------------+------------------------+
|   0   | base_velocity | UniformVelocityCommand |
+-------+---------------+------------------------+

[INFO] Event Manager:  <EventManager> contains 1 active terms.
+---------------------------------------+
|  Active Event Terms in Mode: 'reset'  |
+--------+------------------------------+
| Index  | Name                         |
+--------+------------------------------+
|   0    | physics_material             |
|   1    | reset_base                   |
|   2    | reset_robot_joints           |
|   3    | add_limb_masses              |
|   4    | randomize_actuator_gains     |
|   5    | randomize_joint_properties   |
|   6    | randomize_imu_mount          |
+--------+------------------------------+

[INFO] Recorder Manager:  <RecorderManager> contains 0 active terms.
+---------------------+
| Active Recorder Terms |
+-----------+---------+
|   Index   | Name    |
+-----------+---------+
+-----------+---------+

[INFO] Action Manager:  <ActionManager> contains 1 active terms.
+------------------------------------+
|  Active Action Terms (shape: 10)   |
+--------+-------------+-------------+
| Index  | Name        |   Dimension |
+--------+-------------+-------------+
|   0    | joint_pos   |          10 |
+--------+-------------+-------------+

[INFO] Observation Manager: <ObservationManager> contains 2 groups.
+------------------------------------------------------------+
| Active Observation Terms in Group: 'critic' (shape: (269,)) |
+----------+------------------------------------+------------+
|  Index   | Name                               |   Shape    |
+----------+------------------------------------+------------+
|    0     | base_lin_vel                       |    (3,)    |
|    1     | base_ang_vel                       |    (3,)    |
|    2     | projected_gravity                  |    (3,)    |
|    3     | velocity_commands                  |    (3,)    |
|    4     | actions                            |   (10,)    |
|    5     | height_scan                        |   (187,)   |
|    6     | imu_projected_gravity              |    (3,)    |
|    7     | imu_ang_vel                        |    (3,)    |
|    8     | imu_lin_acc                        |    (3,)    |
|    9     | joint_torques                      |   (10,)    |
|    10    | body_poses                         |   (14,)    |
|    11    | joint_pos_accurate                 |   (10,)    |
|    12    | joint_vel_accurate                 |   (10,)    |
|    13    | base_pos                           |    (1,)    |
|    14    | root_lin_vel_w                     |    (3,)    |
|    15    | root_ang_vel_w                     |    (3,)    |
+----------+------------------------------------+------------+
+---------------------------------------------------------+
| Active Observation Terms in Group: 'policy' (shape: (36,)) |
+-----------+---------------------------------+-----------+
|   Index   | Name                            |   Shape   |
+-----------+---------------------------------+-----------+
|     0     | velocity_commands               |    (3,)   |
|     1     | joint_pos                       |   (10,)   |
|     2     | joint_vel                       |   (10,)   |
|     3     | imu_ang_vel                     |    (3,)   |
|     4     | actions                         |   (10,)   |
+-----------+---------------------------------+-----------+

[INFO] Termination Manager:  <TerminationManager> contains 2 active terms.
+---------------------------------+
|     Active Termination Terms    |
+-------+--------------+----------+
| Index | Name         | Time Out |
+-------+--------------+----------+
|   0   | time_out     |   True   |
|   1   | base_contact |  False   |
+-------+--------------+----------+

[INFO] Reward Manager:  <RewardManager> contains 19 active terms.
+----------------------------------------------------+
|                Active Reward Terms                 |
+-------+--------------------------------+-----------+
| Index | Name                           |    Weight |
+-------+--------------------------------+-----------+
|   0   | track_lin_vel_xy_exp           |       2.0 |
|   1   | track_ang_vel_z_exp            |       1.0 |
|   2   | lin_vel_z_l2                   |       0.0 |
|   3   | ang_vel_xy_l2                  |     -0.05 |
|   4   | dof_torques_l2                 |  -1.5e-07 |
|   5   | dof_acc_l2                     | -1.25e-07 |
|   6   | action_rate_l2                 |     -0.05 |
|   7   | feet_air_time                  |      0.25 |
|   8   | flat_orientation_l2            |      -1.0 |
|   9   | dof_pos_limits                 |      -1.0 |
|   10  | track_lin_vel_x_exp            |       1.0 |
|   11  | track_lin_vel_y_exp            |       1.0 |
|   12  | backward_penalty               |      -1.0 |
|   13  | termination_penalty            |    -200.0 |
|   14  | feet_slide                     |      -0.1 |
|   15  | joint_deviation_hip            |      -0.5 |
|   16  | joint_deviation_hip_pitch_knee |      -0.1 |
|   17  | joint_deviation_ankles         |      -0.5 |
|   18  | foot_impact_penalty            |   -0.0015 |
+-------+--------------------------------+-----------+

[INFO] Curriculum Manager:  <CurriculumManager> contains 1 active terms.
+---------------------------+
|  Active Curriculum Terms  |
+--------+------------------+
| Index  | Name             |
+--------+------------------+
|   0    | terrain_levels   |
+--------+------------------+

[INFO]: Completed setting up the environment...
[INFO] Recording videos during training.
        video_folder: /workspace/isaaclab/logs/rsl_rl/adult_rough/2026-02-18_09-28-52/videos/play
        step_trigger: lambda step: step == 0
        video_length: 1000
        disable_logger: True
/workspace/isaaclab/source/isaaclab/isaaclab/envs/mdp/events.py:1960: UserWarning: To copy construct from a tensor, it is recommended to use sourceTensor.clone().detach() or sourceTensor.clone().detach().requires_grad_(True), rather than torch.tensor(sourceTensor).
  actuator_joint_indices = torch.tensor(actuator.joint_indices, device=asset.device)
[INFO]: Loading model checkpoint from: /workspace/isaaclab/logs/rsl_rl/adult_rough/2026-02-18_09-28-52/model_42350.pt
Actor MLP: Sequential(
  (0): Linear(in_features=36, out_features=512, bias=True)
  (1): ELU(alpha=1.0)
  (2): Linear(in_features=512, out_features=256, bias=True)
  (3): ELU(alpha=1.0)
  (4): Linear(in_features=256, out_features=128, bias=True)
  (5): ELU(alpha=1.0)
  (6): Linear(in_features=128, out_features=10, bias=True)
)
Critic MLP: Sequential(
  (0): Linear(in_features=269, out_features=512, bias=True)
  (1): ELU(alpha=1.0)
  (2): Linear(in_features=512, out_features=256, bias=True)
  (3): ELU(alpha=1.0)
  (4): Linear(in_features=256, out_features=128, bias=True)
  (5): ELU(alpha=1.0)
  (6): Linear(in_features=128, out_features=1, bias=True)
)

========== ACTION INDEX -> JOINT NAME MAPPING ==========
num_actions (from env): N/A

--- ActionTerm: joint_pos (JointPositionAction) ---
  (term-local) idx[00] -> left_hip_pitch
  (term-local) idx[01] -> right_hip_pitch
  (term-local) idx[02] -> left_hip_roll
  (term-local) idx[03] -> right_hip_roll
  (term-local) idx[04] -> left_hip_yaw
  (term-local) idx[05] -> right_hip_yaw
  (term-local) idx[06] -> left_knee_pitch
  (term-local) idx[07] -> right_knee_pitch
  (term-local) idx[08] -> left_ankle_pitch
  (term-local) idx[09] -> right_ankle_pitch
========================================================
