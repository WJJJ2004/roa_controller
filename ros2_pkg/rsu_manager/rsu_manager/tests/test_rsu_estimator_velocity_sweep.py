"""
scripts.tests.test_rsu_estimator_velocity_sweep의 Docstring


최적 조합
jac_lambda = 3e-7
jac_h = 1e-4
beta_jac = 0.95
vel_lpf_tau = 0.0
motor_vel_lpf_tau = 0.0


lwj@lwj:~/colcon_ws/src/roa_controller/ros2_pkg/rsu_manager$ PYTHONPATH=$PWD python3 rsu_manager/tests/test_rsu_estimator_velocity_sweep.py
==== RSU VELOCITY SWEEP TEST ====
total combos: 75
trajectory samples: 16500
feasible samples: 16500

progress: 20/75
progress: 40/75
progress: 60/75
progress: 75/75

==== TOP 15 PARAMETER SETS ====

[Rank 1] jac_lambda=3.0e-07, jac_h=1.0e-04, beta=0.95, vel_tau=0.000, motor_tau=0.000 | q_mean=0.0000 deg, q_max=0.0037 deg | qd_mean=2.4029 deg/s, qd_max=7578.5023 deg/s, qd_rms=115.3576 deg/s | res_mean=2.105e-08, cond_max=1.627
[Rank 2] jac_lambda=3.0e-07, jac_h=7.0e-05, beta=0.95, vel_tau=0.000, motor_tau=0.000 | q_mean=0.0000 deg, q_max=0.0037 deg | qd_mean=2.4029 deg/s, qd_max=7578.5023 deg/s, qd_rms=115.3576 deg/s | res_mean=2.105e-08, cond_max=1.627
[Rank 3] jac_lambda=3.0e-07, jac_h=5.0e-05, beta=0.95, vel_tau=0.000, motor_tau=0.000 | q_mean=0.0000 deg, q_max=0.0037 deg | qd_mean=2.4029 deg/s, qd_max=7578.5023 deg/s, qd_rms=115.3576 deg/s | res_mean=2.105e-08, cond_max=1.627
[Rank 4] jac_lambda=3.0e-07, jac_h=3.0e-05, beta=0.95, vel_tau=0.000, motor_tau=0.000 | q_mean=0.0000 deg, q_max=0.0037 deg | qd_mean=2.4029 deg/s, qd_max=7578.5023 deg/s, qd_rms=115.3576 deg/s | res_mean=2.105e-08, cond_max=1.627
[Rank 5] jac_lambda=3.0e-07, jac_h=2.0e-05, beta=0.95, vel_tau=0.000, motor_tau=0.000 | q_mean=0.0000 deg, q_max=0.0037 deg | qd_mean=2.4029 deg/s, qd_max=7578.5023 deg/s, qd_rms=115.3576 deg/s | res_mean=2.105e-08, cond_max=1.627
[Rank 6] jac_lambda=5.0e-07, jac_h=1.0e-04, beta=0.95, vel_tau=0.000, motor_tau=0.000 | q_mean=0.0000 deg, q_max=0.0037 deg | qd_mean=2.4029 deg/s, qd_max=7578.5016 deg/s, qd_rms=115.3576 deg/s | res_mean=2.105e-08, cond_max=1.627
[Rank 7] jac_lambda=5.0e-07, jac_h=7.0e-05, beta=0.95, vel_tau=0.000, motor_tau=0.000 | q_mean=0.0000 deg, q_max=0.0037 deg | qd_mean=2.4029 deg/s, qd_max=7578.5016 deg/s, qd_rms=115.3576 deg/s | res_mean=2.105e-08, cond_max=1.627
[Rank 8] jac_lambda=5.0e-07, jac_h=5.0e-05, beta=0.95, vel_tau=0.000, motor_tau=0.000 | q_mean=0.0000 deg, q_max=0.0037 deg | qd_mean=2.4029 deg/s, qd_max=7578.5016 deg/s, qd_rms=115.3576 deg/s | res_mean=2.105e-08, cond_max=1.627
[Rank 9] jac_lambda=5.0e-07, jac_h=3.0e-05, beta=0.95, vel_tau=0.000, motor_tau=0.000 | q_mean=0.0000 deg, q_max=0.0037 deg | qd_mean=2.4029 deg/s, qd_max=7578.5016 deg/s, qd_rms=115.3576 deg/s | res_mean=2.105e-08, cond_max=1.627
[Rank 10] jac_lambda=5.0e-07, jac_h=2.0e-05, beta=0.95, vel_tau=0.000, motor_tau=0.000 | q_mean=0.0000 deg, q_max=0.0037 deg | qd_mean=2.4029 deg/s, qd_max=7578.5016 deg/s, qd_rms=115.3576 deg/s | res_mean=2.105e-08, cond_max=1.627
[Rank 11] jac_lambda=1.0e-06, jac_h=1.0e-04, beta=0.95, vel_tau=0.000, motor_tau=0.000 | q_mean=0.0000 deg, q_max=0.0037 deg | qd_mean=2.4029 deg/s, qd_max=7578.4999 deg/s, qd_rms=115.3576 deg/s | res_mean=2.105e-08, cond_max=1.627
[Rank 12] jac_lambda=1.0e-06, jac_h=7.0e-05, beta=0.95, vel_tau=0.000, motor_tau=0.000 | q_mean=0.0000 deg, q_max=0.0037 deg | qd_mean=2.4029 deg/s, qd_max=7578.4999 deg/s, qd_rms=115.3576 deg/s | res_mean=2.105e-08, cond_max=1.627
[Rank 13] jac_lambda=1.0e-06, jac_h=5.0e-05, beta=0.95, vel_tau=0.000, motor_tau=0.000 | q_mean=0.0000 deg, q_max=0.0037 deg | qd_mean=2.4029 deg/s, qd_max=7578.4999 deg/s, qd_rms=115.3576 deg/s | res_mean=2.105e-08, cond_max=1.627
[Rank 14] jac_lambda=1.0e-06, jac_h=3.0e-05, beta=0.95, vel_tau=0.000, motor_tau=0.000 | q_mean=0.0000 deg, q_max=0.0037 deg | qd_mean=2.4029 deg/s, qd_max=7578.4999 deg/s, qd_rms=115.3576 deg/s | res_mean=2.105e-08, cond_max=1.627
[Rank 15] jac_lambda=1.0e-06, jac_h=2.0e-05, beta=0.95, vel_tau=0.000, motor_tau=0.000 | q_mean=0.0000 deg, q_max=0.0037 deg | qd_mean=2.4029 deg/s, qd_max=7578.4999 deg/s, qd_rms=115.3576 deg/s | res_mean=2.105e-08, cond_max=1.627

==== BEST SET DETAIL ====
success                 : True
jac_lambda              : 3.00000000e-07
jac_h                   : 1.00000000e-04
beta_jac                : 9.50000000e-01
vel_lpf_tau             : 0.00000000e+00
motor_vel_lpf_tau       : 0.00000000e+00
samples_total           : 16500
samples_feasible        : 16500
samples_valid           : 9242
samples_eval            : 9242
degraded_count          : 7258
q_err_mean_deg          : 8.34716461e-07
q_err_max_deg           : 3.73975691e-03
qd_err_mean_deg_s       : 2.40291913e+00
qd_err_max_deg_s        : 7.57850233e+03
qd_rms_roll_deg_s       : 3.92877478e+01
qd_rms_pitch_deg_s      : 1.08461270e+02
qd_rms_norm_deg_s       : 1.15357593e+02
residual_mean           : 2.10490652e-08
residual_max            : 9.45142093e-05
condJ_mean              : 1.23672480e+00
condJ_max               : 1.62736897e+00
sigma_min_mean          : 1.42300006e+00
sigma_min_min           : 1.38625579e+00

==== BEST VERIFY-MODE SET (beta=1, no LPF) ====
jac_lambda=3.0e-07, jac_h=2.0e-05, beta=1.00, vel_tau=0.000, motor_tau=0.000 | q_mean=0.0000 deg, q_max=0.0037 deg | qd_mean=2.4298 deg/s, qd_max=7977.3485 deg/s, qd_rms=121.3389 deg/s | res_mean=2.105e-08, cond_max=1.627

==== BEST RUNTIME-MODE SET ====
jac_lambda=3.0e-07, jac_h=1.0e-04, beta=0.95, vel_tau=0.000, motor_tau=0.000 | q_mean=0.0000 deg, q_max=0.0037 deg | qd_mean=2.4029 deg/s, qd_max=7578.5023 deg/s, qd_rms=115.3576 deg/s | res_mean=2.105e-08, cond_max=1.627

"""



# tests/test_rsu_estimator_velocity_sweep.py
import os
import sys
import math
import itertools
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "util"))

from rsu_manager.util.rsu_solver import RSUParams, RSUSolver
from rsu_manager.util.rsu_state_estimator import RSUStateEstimator, RSUStateEstimatorConfig


def rad2deg(x):
    return np.asarray(x) * 180.0 / math.pi


def make_solver():
    W_a = np.array([[0.0,  36.0, 169.5],
                    [0.0, -36.0,  81.0]])
    F_b = np.array([[-30.0,  36.0, 0.0],
                    [-30.0, -36.0, 0.0]])
    c = np.array([30.0, -30.0])
    r = np.array([169.5, 81.0])
    psi = np.array([1.5707963267948966, -1.5707963267948966])
    # =========================

    params = RSUParams(a_W=W_a, b_F=F_b, c=c, r=r, psi=psi, eps=1e-9)
    return RSUSolver(params)


def make_estimator(solver, jac_lambda, jac_h, beta_jac, vel_lpf_tau, motor_vel_lpf_tau):
    cfg = RSUStateEstimatorConfig(
        max_iter=5,
        lambda_pos=1e-6,
        jac_h=jac_h,
        jac_lambda=jac_lambda,
        residual_thresh=1e-4,
        beta_jac=beta_jac,
        vel_lpf_tau=vel_lpf_tau,
        motor_vel_lpf_tau=motor_vel_lpf_tau,
        cond_warn=50.0,
        cond_fail=200.0,
        sigma_min_thresh=1e-5,
        q_init=np.array([0.0, 0.0]),
        hold_last_on_invalid=False,
        zero_vel_on_invalid=True,
    )
    return RSUStateEstimator(solver=solver, cfg=cfg)


def build_trajectory():
    dt = 0.002

    roll_min = np.deg2rad(-15.0)
    roll_max = np.deg2rad(15.0)
    pitch_min = np.deg2rad(-45.0)
    pitch_max = np.deg2rad(30.0)

    roll_mid = 0.5 * (roll_max + roll_min)
    roll_amp = 0.48 * (roll_max - roll_min)

    pitch_mid = 0.5 * (pitch_max + pitch_min)
    pitch_amp = 0.48 * (pitch_max - pitch_min)

    segments_q = []
    segments_qd = []

    def add_segment(T, roll_fn, pitch_fn, roll_dot_fn, pitch_dot_fn):
        t = np.arange(0.0, T, dt)
        roll = roll_fn(t)
        pitch = pitch_fn(t)
        roll_dot = roll_dot_fn(t)
        pitch_dot = pitch_dot_fn(t)

        q = np.column_stack([roll, pitch])
        qd = np.column_stack([roll_dot, pitch_dot])

        q[:, 0] = np.clip(q[:, 0], roll_min, roll_max)
        q[:, 1] = np.clip(q[:, 1], pitch_min, pitch_max)

        segments_q.append(q)
        segments_qd.append(qd)

    # 1) 기존보다 큰 Lissajous sweep
    add_segment(
        T=8.0,
        roll_fn=lambda t: roll_mid + roll_amp * np.sin(2.0 * math.pi * 0.37 * t),
        pitch_fn=lambda t: pitch_mid + pitch_amp * np.sin(2.0 * math.pi * 0.23 * t + 0.7),
        roll_dot_fn=lambda t: roll_amp * (2.0 * math.pi * 0.37) * np.cos(2.0 * math.pi * 0.37 * t),
        pitch_dot_fn=lambda t: pitch_amp * (2.0 * math.pi * 0.23) * np.cos(2.0 * math.pi * 0.23 * t + 0.7),
    )

    # 2) 빠른 중심부 sweep
    add_segment(
        T=5.0,
        roll_fn=lambda t: roll_mid + 0.55 * roll_amp * np.sin(2.0 * math.pi * 1.5 * t),
        pitch_fn=lambda t: pitch_mid + 0.55 * pitch_amp * np.sin(2.0 * math.pi * 1.2 * t + 0.4),
        roll_dot_fn=lambda t: 0.55 * roll_amp * (2.0 * math.pi * 1.5) * np.cos(2.0 * math.pi * 1.5 * t),
        pitch_dot_fn=lambda t: 0.55 * pitch_amp * (2.0 * math.pi * 1.2) * np.cos(2.0 * math.pi * 1.2 * t + 0.4),
    )

    # 3) 경계 근처 sweep: pitch 깊은 dorsiflexion/plantarflexion 쪽 확인
    add_segment(
        T=6.0,
        roll_fn=lambda t: roll_mid + 0.85 * roll_amp * np.sin(2.0 * math.pi * 0.31 * t),
        pitch_fn=lambda t: pitch_mid + 0.95 * pitch_amp * np.sin(2.0 * math.pi * 0.19 * t + 1.2),
        roll_dot_fn=lambda t: 0.85 * roll_amp * (2.0 * math.pi * 0.31) * np.cos(2.0 * math.pi * 0.31 * t),
        pitch_dot_fn=lambda t: 0.95 * pitch_amp * (2.0 * math.pi * 0.19) * np.cos(2.0 * math.pi * 0.19 * t + 1.2),
    )

    # 4) 복합 주파수: 실제 policy 출력처럼 부드럽지만 단일 사인파가 아닌 입력
    add_segment(
        T=6.0,
        roll_fn=lambda t: roll_mid + roll_amp * (
            0.55 * np.sin(2.0 * math.pi * 0.45 * t)
            + 0.25 * np.sin(2.0 * math.pi * 1.35 * t + 0.3)
        ),
        pitch_fn=lambda t: pitch_mid + pitch_amp * (
            0.50 * np.sin(2.0 * math.pi * 0.32 * t + 0.5)
            + 0.30 * np.sin(2.0 * math.pi * 1.10 * t + 1.1)
        ),
        roll_dot_fn=lambda t: roll_amp * (
            0.55 * (2.0 * math.pi * 0.45) * np.cos(2.0 * math.pi * 0.45 * t)
            + 0.25 * (2.0 * math.pi * 1.35) * np.cos(2.0 * math.pi * 1.35 * t + 0.3)
        ),
        pitch_dot_fn=lambda t: pitch_amp * (
            0.50 * (2.0 * math.pi * 0.32) * np.cos(2.0 * math.pi * 0.32 * t + 0.5)
            + 0.30 * (2.0 * math.pi * 1.10) * np.cos(2.0 * math.pi * 1.10 * t + 1.1)
        ),
    )

    # 5) 사각형 경계 경로: workspace 네 모서리 근처를 천천히 연결
    corner_deg = [
        (-15.0, -45.0),
        (15.0, -45.0),
        (15.0, 30.0),
        (-15.0, 30.0),
        (-15.0, -45.0),
    ]

    for (r0_deg, p0_deg), (r1_deg, p1_deg) in zip(corner_deg[:-1], corner_deg[1:]):
        T = 2.0
        t = np.arange(0.0, T, dt)
        s = t / T
        # cubic smoothstep
        u = 3.0 * s**2 - 2.0 * s**3
        du = (6.0 * s - 6.0 * s**2) / T

        r0 = np.deg2rad(r0_deg)
        r1 = np.deg2rad(r1_deg)
        p0 = np.deg2rad(p0_deg)
        p1 = np.deg2rad(p1_deg)

        roll = r0 + (r1 - r0) * u
        pitch = p0 + (p1 - p0) * u
        roll_dot = (r1 - r0) * du
        pitch_dot = (p1 - p0) * du

        segments_q.append(np.column_stack([roll, pitch]))
        segments_qd.append(np.column_stack([roll_dot, pitch_dot]))

    q_true = np.vstack(segments_q)
    qd_true = np.vstack(segments_qd)

    t = np.arange(0.0, len(q_true) * dt, dt)

    return t, dt, q_true, qd_true

# def build_trajectory():
#     dt = 0.002
#     T = 8.0
#     t = np.arange(0.0, T, dt)

#     roll = np.deg2rad(6.0) * np.sin(2.0 * math.pi * 0.5 * t)
#     pitch = np.deg2rad(8.0) * np.sin(2.0 * math.pi * 0.35 * t + 0.4)

#     roll_dot = np.deg2rad(6.0) * (2.0 * math.pi * 0.5) * np.cos(2.0 * math.pi * 0.5 * t)
#     pitch_dot = np.deg2rad(8.0) * (2.0 * math.pi * 0.35) * np.cos(2.0 * math.pi * 0.35 * t + 0.4)

#     q_true = np.column_stack([roll, pitch])
#     qd_true = np.column_stack([roll_dot, pitch_dot])
#     return t, dt, q_true, qd_true


def generate_motor_data(solver, t, q_true, dt):
    alpha = np.zeros((len(t), 2), dtype=float)
    feasible_mask = np.zeros(len(t), dtype=bool)

    prev_alpha = np.array([0.0, 0.0], dtype=float)

    for k in range(len(t)):
        sol = solver.solve(float(q_true[k, 0]), float(q_true[k, 1]), prev_alpha=prev_alpha)
        if sol.feasible:
            alpha[k, :] = sol.alpha
            feasible_mask[k] = True
            prev_alpha = sol.alpha.copy()
        else:
            alpha[k, :] = np.nan
            feasible_mask[k] = False
            prev_alpha = None

    alpha_dot = np.zeros_like(alpha)
    for k in range(1, len(t) - 1):
        if feasible_mask[k - 1] and feasible_mask[k + 1]:
            alpha_dot[k, :] = (alpha[k + 1, :] - alpha[k - 1, :]) / (2.0 * dt)
        else:
            alpha_dot[k, :] = 0.0

    alpha_dot[0, :] = alpha_dot[1, :]
    alpha_dot[-1, :] = alpha_dot[-2, :]

    return alpha, alpha_dot, feasible_mask


def run_one_case(
    solver,
    q_true,
    qd_true,
    alpha,
    alpha_dot,
    feasible_mask,
    dt,
    jac_lambda,
    jac_h,
    beta_jac,
    vel_lpf_tau,
    motor_vel_lpf_tau,
    warmup_sec=0.3,
):
    estimator = make_estimator(
        solver=solver,
        jac_lambda=jac_lambda,
        jac_h=jac_h,
        beta_jac=beta_jac,
        vel_lpf_tau=vel_lpf_tau,
        motor_vel_lpf_tau=motor_vel_lpf_tau,
    )

    N = len(q_true)

    q_hat = np.full_like(q_true, np.nan, dtype=float)
    qd_hat = np.full_like(qd_true, np.nan, dtype=float)
    residual_norm = np.full(N, np.nan, dtype=float)
    condJ = np.full(N, np.nan, dtype=float)
    sigma_min = np.full(N, np.nan, dtype=float)
    valid_mask = np.zeros(N, dtype=bool)
    degraded_mask = np.zeros(N, dtype=bool)

    for k in range(N):
        if not feasible_mask[k]:
            continue

        state = estimator.update(alpha[k, :], alpha_dot[k, :], dt)

        q_hat[k, :] = state.q_rel
        qd_hat[k, :] = state.qd_rel
        residual_norm[k] = state.residual_norm
        condJ[k] = state.condJ
        sigma_min[k] = state.sigma_min
        valid_mask[k] = state.valid
        degraded_mask[k] = state.degraded

    # print("\n[DEBUG ONE CASE]")
    # print("jac_lambda:", jac_lambda, "jac_h:", jac_h, "beta:", beta_jac)
    # print("valid_count:", int(np.sum(valid_mask)))
    # print("degraded_count:", int(np.sum(degraded_mask)))
    # print("residual finite:", np.sum(np.isfinite(residual_norm)))
    # print("residual min/max:", np.nanmin(residual_norm), np.nanmax(residual_norm))
    # print("condJ min/max:", np.nanmin(condJ), np.nanmax(condJ))
    # print("sigma_min min/max:", np.nanmin(sigma_min), np.nanmax(sigma_min))
    # print("first q_hat:", q_hat[0])
    # print("first q_true:", q_true[0])
    

    warmup_n = int(round(warmup_sec / dt))
    eval_mask = feasible_mask & valid_mask
    if warmup_n < N:
        eval_mask[:warmup_n] = False
    if np.sum(eval_mask) == 0:
        return {
            "success": False,
            "reason": "no valid evaluation samples",
            "jac_lambda": jac_lambda,
            "jac_h": jac_h,
            "beta_jac": beta_jac,
            "vel_lpf_tau": vel_lpf_tau,
            "motor_vel_lpf_tau": motor_vel_lpf_tau,
        }

    q_err = q_hat[eval_mask] - q_true[eval_mask]
    qd_err = qd_hat[eval_mask] - qd_true[eval_mask]

    q_err_norm = np.linalg.norm(q_err, axis=1)
    qd_err_norm = np.linalg.norm(qd_err, axis=1)

    qd_rms_each = np.sqrt(np.mean(qd_err ** 2, axis=0))
    qd_rms_norm = math.sqrt(np.mean(np.sum(qd_err ** 2, axis=1)))

    result = {
        "success": True,
        "jac_lambda": jac_lambda,
        "jac_h": jac_h,
        "beta_jac": beta_jac,
        "vel_lpf_tau": vel_lpf_tau,
        "motor_vel_lpf_tau": motor_vel_lpf_tau,

        "samples_total": N,
        "samples_feasible": int(np.sum(feasible_mask)),
        "samples_valid": int(np.sum(valid_mask)),
        "samples_eval": int(np.sum(eval_mask)),
        "degraded_count": int(np.sum(degraded_mask)),

        "q_err_mean_deg": float(np.mean(rad2deg(q_err_norm))),
        "q_err_max_deg": float(np.max(rad2deg(q_err_norm))),

        "qd_err_mean_deg_s": float(np.mean(rad2deg(qd_err_norm))),
        "qd_err_max_deg_s": float(np.max(rad2deg(qd_err_norm))),
        "qd_rms_roll_deg_s": float(rad2deg(qd_rms_each[0])),
        "qd_rms_pitch_deg_s": float(rad2deg(qd_rms_each[1])),
        "qd_rms_norm_deg_s": float(rad2deg(qd_rms_norm)),

        "residual_mean": float(np.nanmean(residual_norm[eval_mask])),
        "residual_max": float(np.nanmax(residual_norm[eval_mask])),

        "condJ_mean": float(np.nanmean(condJ[eval_mask])),
        "condJ_max": float(np.nanmax(condJ[eval_mask])),

        "sigma_min_mean": float(np.nanmean(sigma_min[eval_mask])),
        "sigma_min_min": float(np.nanmin(sigma_min[eval_mask])),
    }
    return result


def print_result(res, rank=None):
    head = f"[Rank {rank}] " if rank is not None else ""
    if not res["success"]:
        print(head + f"FAILED | "
              f"jac_lambda={res['jac_lambda']:.1e}, "
              f"jac_h={res['jac_h']:.1e}, "
              f"beta={res['beta_jac']:.2f}, "
              f"vel_tau={res['vel_lpf_tau']:.3f}, "
              f"motor_tau={res['motor_vel_lpf_tau']:.3f} | "
              f"reason={res['reason']}")
        return

    print(
        head +
        f"jac_lambda={res['jac_lambda']:.1e}, "
        f"jac_h={res['jac_h']:.1e}, "
        f"beta={res['beta_jac']:.2f}, "
        f"vel_tau={res['vel_lpf_tau']:.3f}, "
        f"motor_tau={res['motor_vel_lpf_tau']:.3f} | "
        f"q_mean={res['q_err_mean_deg']:.4f} deg, "
        f"q_max={res['q_err_max_deg']:.4f} deg | "
        f"qd_mean={res['qd_err_mean_deg_s']:.4f} deg/s, "
        f"qd_max={res['qd_err_max_deg_s']:.4f} deg/s, "
        f"qd_rms={res['qd_rms_norm_deg_s']:.4f} deg/s | "
        f"res_mean={res['residual_mean']:.3e}, "
        f"cond_max={res['condJ_max']:.3f}"
    )


def main():
    solver = make_solver()
    t, dt, q_true, qd_true = build_trajectory()
    alpha, alpha_dot, feasible_mask = generate_motor_data(solver, t, q_true, dt)

    if np.sum(feasible_mask) == 0:
        print("No feasible motor trajectory samples.")
        return

    # ======================
    # Sweep candidates
    # ======================

    jac_lambda_list = [3e-7, 5e-7, 1e-6, 2e-6, 3e-6]
    jac_h_list = [2e-5, 3e-5, 5e-5, 7e-5, 1e-4]
    beta_jac_list = [1.0, 0.98, 0.95]

    # LPF fixed
    vel_lpf_tau_list = [0.0]
    motor_vel_lpf_tau_list = [0.0]
    
    # jac_lambda_list = [1e-6]
    # jac_h_list = [5e-5]
    # beta_jac_list = [1.0]
    # vel_lpf_tau_list = [0.0]
    # motor_vel_lpf_tau_list = [0.0]
    # jac_lambda_list = [1e-7, 1e-6, 1e-5, 1e-4]
    # jac_h_list = [1e-6, 1e-5, 5e-5]
    # beta_jac_list = [1.0, 0.95, 0.9, 0.8]
    # vel_lpf_tau_list = [0.0, 0.003, 0.005, 0.01]
    # motor_vel_lpf_tau_list = [0.0, 0.002, 0.005]

    combos = list(itertools.product(
        jac_lambda_list,
        jac_h_list,
        beta_jac_list,
        vel_lpf_tau_list,
        motor_vel_lpf_tau_list,
    ))

    print("==== RSU VELOCITY SWEEP TEST ====")
    print(f"total combos: {len(combos)}")
    print(f"trajectory samples: {len(t)}")
    print(f"feasible samples: {np.sum(feasible_mask)}")
    print("")

    results = []

    for idx, (jac_lambda, jac_h, beta_jac, vel_tau, motor_tau) in enumerate(combos, start=1):
        res = run_one_case(
            solver=solver,
            q_true=q_true,
            qd_true=qd_true,
            alpha=alpha,
            alpha_dot=alpha_dot,
            feasible_mask=feasible_mask,
            dt=dt,
            jac_lambda=jac_lambda,
            jac_h=jac_h,
            beta_jac=beta_jac,
            vel_lpf_tau=vel_tau,
            motor_vel_lpf_tau=motor_tau,
            warmup_sec=0.3,
        )
        results.append(res)

        if idx % 20 == 0 or idx == len(combos):
            print(f"progress: {idx}/{len(combos)}")

    success_results = [r for r in results if r["success"]]
    if len(success_results) == 0:
        print("No successful parameter set.")
        return

    # 정렬 기준:
    # 1) qd mean error
    # 2) qd RMS
    # 3) qd max error
    # 4) q mean error
    ranked = sorted(
        success_results,
        key=lambda r: (
            r["qd_err_mean_deg_s"],
            r["qd_rms_norm_deg_s"],
            r["qd_err_max_deg_s"],
            r["q_err_mean_deg"],
        )
    )

    print("\n==== TOP 15 PARAMETER SETS ====\n")
    for i, r in enumerate(ranked[:15], start=1):
        print_result(r, rank=i)

    best = ranked[0]
    print("\n==== BEST SET DETAIL ====")
    for k, v in best.items():
        if isinstance(v, float):
            print(f"{k:24s}: {v:.8e}")
        else:
            print(f"{k:24s}: {v}")

    # 참고용: 검증모드 후보만 따로 보기
    verify_mode = [
        r for r in success_results
        if abs(r["beta_jac"] - 1.0) < 1e-12
        and abs(r["vel_lpf_tau"]) < 1e-12
        and abs(r["motor_vel_lpf_tau"]) < 1e-12
    ]
    if len(verify_mode) > 0:
        verify_best = sorted(
            verify_mode,
            key=lambda r: (
                r["qd_err_mean_deg_s"],
                r["qd_rms_norm_deg_s"],
                r["qd_err_max_deg_s"],
            )
        )[0]
        print("\n==== BEST VERIFY-MODE SET (beta=1, no LPF) ====")
        print_result(verify_best)

    # 참고용: 실사용 모드 후보만 따로 보기
    runtime_mode = [
        r for r in success_results
        if r["beta_jac"] <= 0.95
    ]
    if len(runtime_mode) > 0:
        runtime_best = sorted(
            runtime_mode,
            key=lambda r: (
                r["qd_err_mean_deg_s"],
                r["qd_rms_norm_deg_s"],
                r["qd_err_max_deg_s"],
            )
        )[0]
        print("\n==== BEST RUNTIME-MODE SET ====")
        print_result(runtime_best)


if __name__ == "__main__":
    main()