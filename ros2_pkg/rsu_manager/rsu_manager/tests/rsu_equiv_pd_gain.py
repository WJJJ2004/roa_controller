#!/usr/bin/env python3
"""
ros2_pkg.rsu_manager.rsu_manager.tests.rsu_equiv_pd_gain의 Docstring
python3 rsu_equiv_pd_gain.py \
  --roll_steps 61 \
  --pitch_steps 61 \
  --csv rsu_equiv_pd_sweep_left_61x61.csv

"""
import math
import argparse
import csv
from dataclasses import dataclass
from typing import Optional

import numpy as np

from rsu_manager.util.rsu_solver import RSUParams, RSUSolver


# ============================================================
# Basic utils
# ============================================================

def deg2rad(deg: float) -> float:
    return deg * math.pi / 180.0


def rad2deg(rad: float) -> float:
    return rad * 180.0 / math.pi


def wrap_to_pi(x: float) -> float:
    two_pi = 2.0 * math.pi
    x = math.fmod(x + math.pi, two_pi)
    if x < 0.0:
        x += two_pi
    return x - math.pi


def wrap_angle_diff(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    diff = np.asarray(a, dtype=np.float64) - np.asarray(b, dtype=np.float64)
    return np.array([wrap_to_pi(float(x)) for x in diff], dtype=np.float64)


def safe_float(x) -> float:
    try:
        return float(x)
    except Exception:
        return float("nan")


# ============================================================
# RSU solver setup
# ============================================================

def create_left_rsu_solver() -> RSUSolver:
    """
    사용자 제공 RSU parameter 기반 왼발 RSU solver 생성.
    virtual q = [roll, pitch]
    actuator q = [alpha_0, alpha_1]
    """

    a_W_flat = [
        0.0, 36.0, 169.5,
        0.0, -36.0, 81.0,
    ]

    b_F_flat = [
        -30.0, 36.0, 0.0,
        -30.0, -36.0, 0.0,
    ]

    c_list = [30.0, -30.0]
    r_list = [169.5, 81.0]
    psi_list = [deg2rad(90.0), deg2rad(-90.0)]

    a_W = np.array(a_W_flat, dtype=np.float64).reshape(2, 3)
    b_F = np.array(b_F_flat, dtype=np.float64).reshape(2, 3)
    c = np.array(c_list, dtype=np.float64).reshape(2,)
    r = np.array(r_list, dtype=np.float64).reshape(2,)
    psi = np.array(psi_list, dtype=np.float64).reshape(2,)

    params = RSUParams(
        a_W=a_W,
        b_F=b_F,
        c=c,
        r=r,
        psi=psi,
    )

    return RSUSolver(params)


def solve_alpha(
    solver: RSUSolver,
    roll: float,
    pitch: float,
    prev_alpha: Optional[np.ndarray] = None,
) -> np.ndarray:
    result = solver.solve(
        roll=roll,
        pitch=pitch,
        prev_alpha=prev_alpha,
    )

    if not result.feasible:
        raise RuntimeError(
            f"RSU solver infeasible. "
            f"roll={rad2deg(roll):.3f} deg, "
            f"pitch={rad2deg(pitch):.3f} deg, "
            f"branch={result.branch}, residual={result.residual}"
        )

    return np.asarray(result.alpha, dtype=np.float64).reshape(2,)


# ============================================================
# Jacobian / equivalent gain
# ============================================================

def compute_numerical_jacobian(
    solver: RSUSolver,
    qv: np.ndarray,
    eps: float,
) -> tuple[np.ndarray, np.ndarray]:
    """
    J = dqa / dqv

    qv = [roll, pitch]
    qa = [alpha_0, alpha_1]

    J shape = (2 actuator, 2 virtual)
    """

    qv = np.asarray(qv, dtype=np.float64).reshape(2,)

    roll = float(qv[0])
    pitch = float(qv[1])

    alpha0 = solve_alpha(
        solver=solver,
        roll=roll,
        pitch=pitch,
        prev_alpha=None,
    )

    J = np.zeros((2, 2), dtype=np.float64)

    for i in range(2):
        qv_plus = qv.copy()
        qv_minus = qv.copy()

        qv_plus[i] += eps
        qv_minus[i] -= eps

        alpha_plus = solve_alpha(
            solver=solver,
            roll=float(qv_plus[0]),
            pitch=float(qv_plus[1]),
            prev_alpha=alpha0,
        )

        alpha_minus = solve_alpha(
            solver=solver,
            roll=float(qv_minus[0]),
            pitch=float(qv_minus[1]),
            prev_alpha=alpha0,
        )

        d_alpha = wrap_angle_diff(alpha_plus, alpha_minus)
        J[:, i] = d_alpha / (2.0 * eps)

    return J, alpha0


def compute_effective_virtual_pd(
    J: np.ndarray,
    actuator_kp: np.ndarray,
    actuator_kd: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    """
    Kv_eff = J.T @ Ka @ J
    Dv_eff = J.T @ Da @ J
    """

    actuator_kp = np.asarray(actuator_kp, dtype=np.float64).reshape(2,)
    actuator_kd = np.asarray(actuator_kd, dtype=np.float64).reshape(2,)

    Ka = np.diag(actuator_kp)
    Da = np.diag(actuator_kd)

    Kv_eff = J.T @ Ka @ J
    Dv_eff = J.T @ Da @ J

    return Kv_eff, Dv_eff


# ============================================================
# Sweep data structure
# ============================================================

@dataclass
class SweepSample:
    roll_deg: float
    pitch_deg: float

    alpha0_deg: float
    alpha1_deg: float

    j00: float
    j01: float
    j10: float
    j11: float

    cond: float
    sigma_min: float
    sigma_max: float

    kv_roll: float
    kv_pitch: float
    kv_coupling: float

    dv_roll: float
    dv_pitch: float
    dv_coupling: float

    feasible: bool
    error: str = ""


def analyze_one_pose(
    solver: RSUSolver,
    roll: float,
    pitch: float,
    actuator_kp: np.ndarray,
    actuator_kd: np.ndarray,
    eps: float,
) -> SweepSample:
    roll_deg = rad2deg(roll)
    pitch_deg = rad2deg(pitch)

    try:
        qv = np.array([roll, pitch], dtype=np.float64)

        J, alpha = compute_numerical_jacobian(
            solver=solver,
            qv=qv,
            eps=eps,
        )

        Kv_eff, Dv_eff = compute_effective_virtual_pd(
            J=J,
            actuator_kp=actuator_kp,
            actuator_kd=actuator_kd,
        )

        singular_values = np.linalg.svd(J, compute_uv=False)
        sigma_max = float(np.max(singular_values))
        sigma_min = float(np.min(singular_values))
        cond = float(np.linalg.cond(J))

        return SweepSample(
            roll_deg=roll_deg,
            pitch_deg=pitch_deg,

            alpha0_deg=rad2deg(float(alpha[0])),
            alpha1_deg=rad2deg(float(alpha[1])),

            j00=float(J[0, 0]),
            j01=float(J[0, 1]),
            j10=float(J[1, 0]),
            j11=float(J[1, 1]),

            cond=cond,
            sigma_min=sigma_min,
            sigma_max=sigma_max,

            kv_roll=float(Kv_eff[0, 0]),
            kv_pitch=float(Kv_eff[1, 1]),
            kv_coupling=float(Kv_eff[0, 1]),

            dv_roll=float(Dv_eff[0, 0]),
            dv_pitch=float(Dv_eff[1, 1]),
            dv_coupling=float(Dv_eff[0, 1]),

            feasible=True,
            error="",
        )

    except Exception as e:
        return SweepSample(
            roll_deg=roll_deg,
            pitch_deg=pitch_deg,

            alpha0_deg=float("nan"),
            alpha1_deg=float("nan"),

            j00=float("nan"),
            j01=float("nan"),
            j10=float("nan"),
            j11=float("nan"),

            cond=float("nan"),
            sigma_min=float("nan"),
            sigma_max=float("nan"),

            kv_roll=float("nan"),
            kv_pitch=float("nan"),
            kv_coupling=float("nan"),

            dv_roll=float("nan"),
            dv_pitch=float("nan"),
            dv_coupling=float("nan"),

            feasible=False,
            error=str(e),
        )


# ============================================================
# Sweep methods
# ============================================================

def make_grid_samples(
    roll_min: float,
    roll_max: float,
    pitch_min: float,
    pitch_max: float,
    roll_steps: int,
    pitch_steps: int,
) -> list[tuple[float, float]]:
    rolls = np.linspace(roll_min, roll_max, roll_steps)
    pitches = np.linspace(pitch_min, pitch_max, pitch_steps)

    samples = []
    for roll in rolls:
        for pitch in pitches:
            samples.append((float(roll), float(pitch)))

    return samples


def make_random_samples(
    roll_min: float,
    roll_max: float,
    pitch_min: float,
    pitch_max: float,
    num_samples: int,
    seed: int,
) -> list[tuple[float, float]]:
    rng = np.random.default_rng(seed)

    rolls = rng.uniform(roll_min, roll_max, size=num_samples)
    pitches = rng.uniform(pitch_min, pitch_max, size=num_samples)

    return [(float(r), float(p)) for r, p in zip(rolls, pitches)]


# ============================================================
# Statistics / reporting
# ============================================================

def array_from_samples(samples: list[SweepSample], attr: str) -> np.ndarray:
    values = []
    for s in samples:
        if s.feasible:
            values.append(safe_float(getattr(s, attr)))
    return np.asarray(values, dtype=np.float64)


def print_stat(name: str, values: np.ndarray) -> None:
    if values.size == 0:
        print(f"{name:20s}: no valid data")
        return

    print(
        f"{name:20s}: "
        f"mean={np.mean(values): .8f}, "
        f"std={np.std(values): .8f}, "
        f"min={np.min(values): .8f}, "
        f"max={np.max(values): .8f}"
    )


def find_max_sample(samples: list[SweepSample], attr: str) -> Optional[SweepSample]:
    valid = [s for s in samples if s.feasible and np.isfinite(safe_float(getattr(s, attr)))]
    if not valid:
        return None
    return max(valid, key=lambda s: abs(safe_float(getattr(s, attr))))


def print_extreme_pose(samples: list[SweepSample], attr: str, label: str) -> None:
    s = find_max_sample(samples, attr)
    if s is None:
        print(f"{label}: no valid sample")
        return

    value = safe_float(getattr(s, attr))
    print(
        f"{label}: {attr}={value:.8f} at "
        f"roll={s.roll_deg:.3f} deg, pitch={s.pitch_deg:.3f} deg, "
        f"cond={s.cond:.6f}"
    )


def print_summary(samples: list[SweepSample]) -> None:
    total = len(samples)
    feasible_samples = [s for s in samples if s.feasible]
    failed_samples = [s for s in samples if not s.feasible]

    print("\n========== RSU Equivalent PD Sweep Summary ==========")
    print(f"total samples    = {total}")
    print(f"feasible samples = {len(feasible_samples)}")
    print(f"failed samples   = {len(failed_samples)}")

    if total > 0:
        print(f"success ratio    = {len(feasible_samples) / total * 100.0:.2f} %")

    print("\n---------- Kv_eff statistics ----------")
    print_stat("Kv roll", array_from_samples(samples, "kv_roll"))
    print_stat("Kv pitch", array_from_samples(samples, "kv_pitch"))
    print_stat("Kv coupling", array_from_samples(samples, "kv_coupling"))

    print("\n---------- Dv_eff statistics ----------")
    print_stat("Dv roll", array_from_samples(samples, "dv_roll"))
    print_stat("Dv pitch", array_from_samples(samples, "dv_pitch"))
    print_stat("Dv coupling", array_from_samples(samples, "dv_coupling"))

    print("\n---------- Jacobian quality statistics ----------")
    print_stat("condition", array_from_samples(samples, "cond"))
    print_stat("sigma min", array_from_samples(samples, "sigma_min"))
    print_stat("sigma max", array_from_samples(samples, "sigma_max"))

    print("\n---------- Extreme poses ----------")
    print_extreme_pose(samples, "kv_roll", "Max abs Kv roll")
    print_extreme_pose(samples, "kv_pitch", "Max abs Kv pitch")
    print_extreme_pose(samples, "kv_coupling", "Max abs Kv coupling")
    print_extreme_pose(samples, "cond", "Max condition")

    if failed_samples:
        print("\n---------- First failed samples ----------")
        for s in failed_samples[:5]:
            print(
                f"failed at roll={s.roll_deg:.3f} deg, "
                f"pitch={s.pitch_deg:.3f} deg, error={s.error}"
            )

    print("\n=====================================================\n")


def write_csv(path: str, samples: list[SweepSample]) -> None:
    fieldnames = list(SweepSample.__dataclass_fields__.keys())

    with open(path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()

        for s in samples:
            writer.writerow({
                key: getattr(s, key)
                for key in fieldnames
            })


# ============================================================
# Main
# ============================================================

def main():
    parser = argparse.ArgumentParser(
        description="Sweep RSU Jacobian-based equivalent virtual PD gain."
    )

    parser.add_argument("--roll_min_deg", type=float, default=-25.0)
    parser.add_argument("--roll_max_deg", type=float, default=25.0)
    parser.add_argument("--pitch_min_deg", type=float, default=-25.0)
    parser.add_argument("--pitch_max_deg", type=float, default=25.0)

    parser.add_argument(
        "--mode",
        type=str,
        default="grid",
        choices=["grid", "random"],
        help="Sampling mode.",
    )

    parser.add_argument(
        "--roll_steps",
        type=int,
        default=31,
        help="Grid roll sample count.",
    )

    parser.add_argument(
        "--pitch_steps",
        type=int,
        default=31,
        help="Grid pitch sample count.",
    )

    parser.add_argument(
        "--num_samples",
        type=int,
        default=1000,
        help="Random sample count.",
    )

    parser.add_argument(
        "--seed",
        type=int,
        default=0,
        help="Random sampling seed.",
    )

    parser.add_argument(
        "--kp",
        type=float,
        default=10.0,
        help="Initial actuator Kp for both RSU actuators.",
    )

    parser.add_argument(
        "--kd",
        type=float,
        default=0.25,
        help="Initial actuator Kd for both RSU actuators.",
    )

    parser.add_argument(
        "--kp0",
        type=float,
        default=None,
        help="Optional actuator 0 Kp. If omitted, --kp is used.",
    )

    parser.add_argument(
        "--kp1",
        type=float,
        default=None,
        help="Optional actuator 1 Kp. If omitted, --kp is used.",
    )

    parser.add_argument(
        "--kd0",
        type=float,
        default=None,
        help="Optional actuator 0 Kd. If omitted, --kd is used.",
    )

    parser.add_argument(
        "--kd1",
        type=float,
        default=None,
        help="Optional actuator 1 Kd. If omitted, --kd is used.",
    )

    parser.add_argument(
        "--eps",
        type=float,
        default=1e-5,
        help="Numerical differentiation epsilon in rad.",
    )

    parser.add_argument(
        "--csv",
        type=str,
        default="",
        help="Optional CSV output path.",
    )

    args = parser.parse_args()

    roll_min = deg2rad(args.roll_min_deg)
    roll_max = deg2rad(args.roll_max_deg)
    pitch_min = deg2rad(args.pitch_min_deg)
    pitch_max = deg2rad(args.pitch_max_deg)

    kp0 = args.kp if args.kp0 is None else args.kp0
    kp1 = args.kp if args.kp1 is None else args.kp1
    kd0 = args.kd if args.kd0 is None else args.kd0
    kd1 = args.kd if args.kd1 is None else args.kd1

    actuator_kp = np.array([kp0, kp1], dtype=np.float64)
    actuator_kd = np.array([kd0, kd1], dtype=np.float64)

    solver = create_left_rsu_solver()

    if args.mode == "grid":
        pose_list = make_grid_samples(
            roll_min=roll_min,
            roll_max=roll_max,
            pitch_min=pitch_min,
            pitch_max=pitch_max,
            roll_steps=args.roll_steps,
            pitch_steps=args.pitch_steps,
        )
    else:
        pose_list = make_random_samples(
            roll_min=roll_min,
            roll_max=roll_max,
            pitch_min=pitch_min,
            pitch_max=pitch_max,
            num_samples=args.num_samples,
            seed=args.seed,
        )

    print("\n========== RSU Equivalent PD Sweep Config ==========")
    print(f"mode       = {args.mode}")
    print(f"roll range = [{args.roll_min_deg:.3f}, {args.roll_max_deg:.3f}] deg")
    print(f"pitch range= [{args.pitch_min_deg:.3f}, {args.pitch_max_deg:.3f}] deg")
    print(f"eps        = {args.eps:.3e} rad")
    print(f"actuator Kp= {actuator_kp}")
    print(f"actuator Kd= {actuator_kd}")

    if args.mode == "grid":
        print(f"roll_steps = {args.roll_steps}")
        print(f"pitch_steps= {args.pitch_steps}")
        print(f"total grid = {args.roll_steps * args.pitch_steps}")
    else:
        print(f"num_samples= {args.num_samples}")
        print(f"seed       = {args.seed}")

    print("====================================================\n")

    samples: list[SweepSample] = []

    for idx, (roll, pitch) in enumerate(pose_list):
        sample = analyze_one_pose(
            solver=solver,
            roll=roll,
            pitch=pitch,
            actuator_kp=actuator_kp,
            actuator_kd=actuator_kd,
            eps=args.eps,
        )
        samples.append(sample)

        # 너무 출력이 많아지지 않게 진행 상황만 간단히 표시
        if (idx + 1) % 200 == 0 or (idx + 1) == len(pose_list):
            print(f"processed {idx + 1}/{len(pose_list)} samples")

    print_summary(samples)

    if args.csv:
        write_csv(args.csv, samples)
        print(f"CSV saved: {args.csv}")


if __name__ == "__main__":
    main()