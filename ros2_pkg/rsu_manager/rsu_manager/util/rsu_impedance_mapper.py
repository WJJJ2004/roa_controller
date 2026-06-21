#!/usr/bin/env python3
# rsu_impedance_mapper.py
#
# Maps a desired virtual RSU ankle impedance
#   q_v = [roll, pitch]
# into independent actuator MIT gains
#   Ka = diag(kp1, kp2), Da = diag(kd1, kd2)
#
# Jacobian convention:
#   alpha_dot = J_qx @ q_dot
#   J_qx = d(alpha) / d([roll, pitch])
#
# Equivalent virtual impedance:
#   Kv_eff = J_qx.T @ Ka @ J_qx
#   Dv_eff = J_qx.T @ Da @ J_qx
#
# Because the hardware exposes only independent scalar Kp/Kd values,
# the requested 2x2 virtual impedance is approximated using bounded
# diagonal-gain least squares.

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Optional, Tuple
import math
import numpy as np


def _vec2(value, name: str) -> np.ndarray:
    arr = np.asarray(value, dtype=np.float64).reshape(2,)
    if not np.all(np.isfinite(arr)):
        raise ValueError(f"{name} contains NaN/Inf: {arr}")
    return arr


def _mat2(value, name: str) -> np.ndarray:
    arr = np.asarray(value, dtype=np.float64).reshape(2, 2)
    if not np.all(np.isfinite(arr)):
        raise ValueError(f"{name} contains NaN/Inf:\n{arr}")
    return arr


def _relative_frobenius_error(
    achieved: np.ndarray,
    desired: np.ndarray,
    eps: float = 1e-12,
) -> float:
    denom = max(float(np.linalg.norm(desired, ord="fro")), eps)
    return float(np.linalg.norm(achieved - desired, ord="fro") / denom)


@dataclass
class RSUImpedanceMapperConfig:
    # ------------------------------------------------------------------
    # Desired virtual ankle impedance
    # Axis order: [roll, pitch]
    # ------------------------------------------------------------------
    virtual_pitch_kp: float = 20.0
    virtual_pitch_kd: float = 5.0
    virtual_roll_scale: float = 1.37

    # ------------------------------------------------------------------
    # Actuator gain limits
    # Scalar values are applied to both actuators unless per-actuator
    # arrays are explicitly passed after construction.
    # ------------------------------------------------------------------
    actuator_kp_min: np.ndarray = field(
        default_factory=lambda: np.array([5.0, 5.0], dtype=np.float64)
    )
    actuator_kp_max: np.ndarray = field(
        default_factory=lambda: np.array([25.0, 25.0], dtype=np.float64)
    )
    actuator_kd_min: np.ndarray = field(
        default_factory=lambda: np.array([0.2, 0.2], dtype=np.float64)
    )
    actuator_kd_max: np.ndarray = field(
        default_factory=lambda: np.array([6.0, 6.0], dtype=np.float64)
    )

    # Initial fallback values before the first valid mapping
    default_kp: np.ndarray = field(
        default_factory=lambda: np.array([9.0, 9.0], dtype=np.float64)
    )
    default_kd: np.ndarray = field(
        default_factory=lambda: np.array([2.25, 2.25], dtype=np.float64)
    )

    # ------------------------------------------------------------------
    # Jacobian validity
    # The measured workspace showed cond(J) <= about 2.17.
    # ------------------------------------------------------------------
    cond_warn: float = 10.0
    cond_fail: float = 50.0
    sigma_min_thresh: float = 1e-4

    # ------------------------------------------------------------------
    # Fitting quality
    # Relative Frobenius error:
    # ||G_achieved - G_desired||_F / ||G_desired||_F
    # ------------------------------------------------------------------
    relative_fit_error_warn: float = 0.20
    relative_fit_error_fail: float = 0.50

    # ------------------------------------------------------------------
    # Temporal stabilization
    # Set cutoff <= 0 to bypass LPF.
    # Set slew rate <= 0 to bypass the corresponding rate limiter.
    # ------------------------------------------------------------------
    gain_lpf_cutoff_hz: float = 5.0
    kp_slew_rate: float = 100.0   # gain units / second
    kd_slew_rate: float = 30.0    # gain units / second

    # Timing validity
    dt_min: float = 1e-5
    dt_max: float = 0.2

    # Fallback behavior
    hold_last_on_invalid: bool = True

    def __post_init__(self):
        self.actuator_kp_min = _vec2(self.actuator_kp_min, "actuator_kp_min")
        self.actuator_kp_max = _vec2(self.actuator_kp_max, "actuator_kp_max")
        self.actuator_kd_min = _vec2(self.actuator_kd_min, "actuator_kd_min")
        self.actuator_kd_max = _vec2(self.actuator_kd_max, "actuator_kd_max")
        self.default_kp = _vec2(self.default_kp, "default_kp")
        self.default_kd = _vec2(self.default_kd, "default_kd")

        if np.any(self.actuator_kp_min > self.actuator_kp_max):
            raise ValueError("actuator_kp_min must be <= actuator_kp_max")
        if np.any(self.actuator_kd_min > self.actuator_kd_max):
            raise ValueError("actuator_kd_min must be <= actuator_kd_max")

        self.default_kp = np.clip(
            self.default_kp, self.actuator_kp_min, self.actuator_kp_max
        )
        self.default_kd = np.clip(
            self.default_kd, self.actuator_kd_min, self.actuator_kd_max
        )


@dataclass
class DiagonalGainFit:
    gain_raw: np.ndarray
    gain_bounded: np.ndarray
    achieved_virtual: np.ndarray
    relative_error: float
    saturated: bool
    valid: bool


@dataclass
class RSUImpedanceResult:
    valid: bool
    degraded: bool
    fallback_used: bool
    saturated: bool

    kp: np.ndarray
    kd: np.ndarray

    J_qx: np.ndarray
    condJ: float
    sigma_min: float
    sigma_max: float

    requested_Kv: np.ndarray
    requested_Dv: np.ndarray
    achieved_Kv: np.ndarray
    achieved_Dv: np.ndarray

    stiffness_error: float
    damping_error: float

    debug_msg: str = ""


class RSUImpedanceMapper:
    """
    Current-state Jacobian based RSU impedance mapper.

    Input:
        J_qx = d(alpha) / d([roll, pitch])
        dt

    Output:
        independent actuator gains kp[2], kd[2]

    Notes:
    - q_target is not generated here. The existing RSU IK solver remains
      responsible for actuator target angles.
    - This class is stateful because gain filtering, slew-rate limiting,
      and hold-last fallback require previous outputs.
    """

    def __init__(self, cfg: Optional[RSUImpedanceMapperConfig] = None):
        self.cfg = cfg if cfg is not None else RSUImpedanceMapperConfig()

        roll_kp = (
            float(self.cfg.virtual_pitch_kp)
            * float(self.cfg.virtual_roll_scale)
        )
        roll_kd = (
            float(self.cfg.virtual_pitch_kd)
            * float(self.cfg.virtual_roll_scale)
        )

        self.Kv_desired = np.diag([
            roll_kp,
            float(self.cfg.virtual_pitch_kp),
        ]).astype(np.float64)

        self.Dv_desired = np.diag([
            roll_kd,
            float(self.cfg.virtual_pitch_kd),
        ]).astype(np.float64)

        self.kp_prev_ = self.cfg.default_kp.copy()
        self.kd_prev_ = self.cfg.default_kd.copy()
        self.has_valid_output_ = False

    def reset(
        self,
        kp_init: Optional[np.ndarray] = None,
        kd_init: Optional[np.ndarray] = None,
        initialized: bool = False,
    ) -> None:
        if kp_init is None:
            kp = self.cfg.default_kp.copy()
        else:
            kp = _vec2(kp_init, "kp_init")

        if kd_init is None:
            kd = self.cfg.default_kd.copy()
        else:
            kd = _vec2(kd_init, "kd_init")

        self.kp_prev_ = np.clip(
            kp, self.cfg.actuator_kp_min, self.cfg.actuator_kp_max
        )
        self.kd_prev_ = np.clip(
            kd, self.cfg.actuator_kd_min, self.cfg.actuator_kd_max
        )
        self.has_valid_output_ = bool(initialized)

    @staticmethod
    def equivalent_virtual_gain(
        J_qx: np.ndarray,
        actuator_gain: np.ndarray,
    ) -> np.ndarray:
        J = _mat2(J_qx, "J_qx")
        gain = _vec2(actuator_gain, "actuator_gain")
        return J.T @ np.diag(gain) @ J

    @staticmethod
    def _fit_matrix(J_qx: np.ndarray) -> np.ndarray:
        """
        Build the weighted linear system corresponding to the symmetric
        Frobenius norm.

        For J = [[a,b],[c,d]] and G_a = diag(g1,g2):

            J.T G_a J =
            [[g1*a^2 + g2*c^2, g1*a*b + g2*c*d],
             [same,                 g1*b^2 + g2*d^2]]

        The off-diagonal residual is multiplied by sqrt(2), because it
        appears twice in the full Frobenius norm.
        """
        J = _mat2(J_qx, "J_qx")
        a, b = float(J[0, 0]), float(J[0, 1])
        c, d = float(J[1, 0]), float(J[1, 1])
        root2 = math.sqrt(2.0)

        return np.array([
            [a * a, c * c],
            [root2 * a * b, root2 * c * d],
            [b * b, d * d],
        ], dtype=np.float64)

    @staticmethod
    def _fit_target(G_virtual: np.ndarray) -> np.ndarray:
        G = _mat2(G_virtual, "G_virtual")
        G_sym = 0.5 * (G + G.T)
        root2 = math.sqrt(2.0)
        return np.array([
            G_sym[0, 0],
            root2 * G_sym[0, 1],
            G_sym[1, 1],
        ], dtype=np.float64)

    @classmethod
    def fit_diagonal_gain(
        cls,
        J_qx: np.ndarray,
        desired_virtual: np.ndarray,
        gain_min: np.ndarray,
        gain_max: np.ndarray,
    ) -> DiagonalGainFit:
        """
        Solve the two-variable box-constrained least-squares problem.

        Since only two actuator gains exist, the exact constrained optimum
        can be found without SciPy by evaluating:
          1) the unconstrained least-squares solution,
          2) each of the four box edges,
          3) all four box corners.

        This is superior to simply solving and clipping, because clipping an
        unconstrained solution is not always the constrained least-squares
        optimum.
        """
        J = _mat2(J_qx, "J_qx")
        desired = _mat2(desired_virtual, "desired_virtual")
        lower = _vec2(gain_min, "gain_min")
        upper = _vec2(gain_max, "gain_max")

        if np.any(lower > upper):
            raise ValueError("gain_min must be <= gain_max")

        A = cls._fit_matrix(J)
        b = cls._fit_target(desired)

        candidates = []

        def add_candidate(x):
            x = np.asarray(x, dtype=np.float64).reshape(2,)
            if not np.all(np.isfinite(x)):
                return
            if np.any(x < lower - 1e-12) or np.any(x > upper + 1e-12):
                return
            residual = A @ x - b
            cost = float(residual @ residual)
            candidates.append((cost, x.copy()))

        # Unconstrained solution
        try:
            x_unconstrained, _, _, _ = np.linalg.lstsq(A, b, rcond=None)
            add_candidate(x_unconstrained)
        except np.linalg.LinAlgError:
            x_unconstrained = np.full(2, np.nan, dtype=np.float64)

        # Optimize x1 while x0 is fixed to each bound.
        denom_col1 = float(A[:, 1] @ A[:, 1])
        for fixed_x0 in (lower[0], upper[0]):
            if denom_col1 > 1e-15:
                x1 = float(A[:, 1] @ (b - A[:, 0] * fixed_x0)) / denom_col1
                x1 = float(np.clip(x1, lower[1], upper[1]))
                add_candidate([fixed_x0, x1])

        # Optimize x0 while x1 is fixed to each bound.
        denom_col0 = float(A[:, 0] @ A[:, 0])
        for fixed_x1 in (lower[1], upper[1]):
            if denom_col0 > 1e-15:
                x0 = float(A[:, 0] @ (b - A[:, 1] * fixed_x1)) / denom_col0
                x0 = float(np.clip(x0, lower[0], upper[0]))
                add_candidate([x0, fixed_x1])

        # Corners guarantee at least one candidate.
        for x0 in (lower[0], upper[0]):
            for x1 in (lower[1], upper[1]):
                add_candidate([x0, x1])

        if not candidates:
            zeros = np.zeros(2, dtype=np.float64)
            return DiagonalGainFit(
                gain_raw=zeros.copy(),
                gain_bounded=zeros.copy(),
                achieved_virtual=np.zeros((2, 2), dtype=np.float64),
                relative_error=float("inf"),
                saturated=True,
                valid=False,
            )

        candidates.sort(key=lambda item: item[0])
        gain_bounded = candidates[0][1]

        achieved = cls.equivalent_virtual_gain(J, gain_bounded)
        relative_error = _relative_frobenius_error(achieved, desired)

        unconstrained_valid = np.all(np.isfinite(x_unconstrained))
        saturated = (
            not unconstrained_valid
            or np.any(x_unconstrained < lower - 1e-9)
            or np.any(x_unconstrained > upper + 1e-9)
            or np.linalg.norm(gain_bounded - x_unconstrained) > 1e-8
        )

        return DiagonalGainFit(
            gain_raw=x_unconstrained.copy(),
            gain_bounded=gain_bounded.copy(),
            achieved_virtual=achieved.copy(),
            relative_error=float(relative_error),
            saturated=bool(saturated),
            valid=bool(
                np.all(np.isfinite(gain_bounded))
                and np.all(np.isfinite(achieved))
                and np.isfinite(relative_error)
            ),
        )

    def _filter_gain(
        self,
        raw: np.ndarray,
        prev: np.ndarray,
        dt: float,
        slew_rate: float,
        gain_min: np.ndarray,
        gain_max: np.ndarray,
    ) -> np.ndarray:
        raw = _vec2(raw, "raw_gain")
        prev = _vec2(prev, "prev_gain")

        # First-order LPF
        cutoff = float(self.cfg.gain_lpf_cutoff_hz)
        if np.isfinite(cutoff) and cutoff > 0.0:
            tau = 1.0 / (2.0 * math.pi * cutoff)
            gamma = dt / (tau + dt)
            gamma = float(np.clip(gamma, 0.0, 1.0))
            filtered = prev + gamma * (raw - prev)
        else:
            filtered = raw.copy()

        # Slew-rate limiter
        if np.isfinite(slew_rate) and slew_rate > 0.0:
            max_delta = float(slew_rate) * dt
            delta = np.clip(filtered - prev, -max_delta, max_delta)
            filtered = prev + delta

        return np.clip(filtered, gain_min, gain_max)

    def _fallback(
        self,
        J_qx: Optional[np.ndarray],
        condJ: float,
        sigma_min: float,
        sigma_max: float,
        debug_msg: str,
    ) -> RSUImpedanceResult:
        if self.cfg.hold_last_on_invalid and self.has_valid_output_:
            kp = self.kp_prev_.copy()
            kd = self.kd_prev_.copy()
        else:
            kp = self.cfg.default_kp.copy()
            kd = self.cfg.default_kd.copy()

        if J_qx is not None:
            try:
                J = _mat2(J_qx, "J_qx")
                achieved_Kv = self.equivalent_virtual_gain(J, kp)
                achieved_Dv = self.equivalent_virtual_gain(J, kd)
                stiffness_error = _relative_frobenius_error(
                    achieved_Kv, self.Kv_desired
                )
                damping_error = _relative_frobenius_error(
                    achieved_Dv, self.Dv_desired
                )
            except Exception:
                J = np.zeros((2, 2), dtype=np.float64)
                achieved_Kv = np.zeros((2, 2), dtype=np.float64)
                achieved_Dv = np.zeros((2, 2), dtype=np.float64)
                stiffness_error = float("inf")
                damping_error = float("inf")
        else:
            J = np.zeros((2, 2), dtype=np.float64)
            achieved_Kv = np.zeros((2, 2), dtype=np.float64)
            achieved_Dv = np.zeros((2, 2), dtype=np.float64)
            stiffness_error = float("inf")
            damping_error = float("inf")

        return RSUImpedanceResult(
            valid=False,
            degraded=True,
            fallback_used=True,
            saturated=False,
            kp=kp,
            kd=kd,
            J_qx=J,
            condJ=float(condJ),
            sigma_min=float(sigma_min),
            sigma_max=float(sigma_max),
            requested_Kv=self.Kv_desired.copy(),
            requested_Dv=self.Dv_desired.copy(),
            achieved_Kv=achieved_Kv,
            achieved_Dv=achieved_Dv,
            stiffness_error=float(stiffness_error),
            damping_error=float(damping_error),
            debug_msg=debug_msg,
        )

    def compute(
        self,
        J_qx: np.ndarray,
        dt: float,
        estimator_valid: bool = True,
        estimator_degraded: bool = False,
    ) -> RSUImpedanceResult:
        """
        Compute actuator Kp/Kd from the Jacobian at the current estimated pose.

        Parameters
        ----------
        J_qx:
            Current-state Jacobian d(alpha)/d([roll,pitch]), shape (2,2).
            The estimator's AnkleState.J_qx can be passed directly.
        dt:
            Time since the previous impedance update.
        estimator_valid:
            Pass AnkleState.valid.
        estimator_degraded:
            Pass AnkleState.degraded.
        """
        if not estimator_valid:
            return self._fallback(
                J_qx=J_qx,
                condJ=float("inf"),
                sigma_min=0.0,
                sigma_max=0.0,
                debug_msg="state estimator is invalid",
            )

        if not np.isfinite(dt) or not (self.cfg.dt_min <= dt <= self.cfg.dt_max):
            return self._fallback(
                J_qx=J_qx,
                condJ=float("inf"),
                sigma_min=0.0,
                sigma_max=0.0,
                debug_msg=f"invalid dt: {dt}",
            )

        try:
            J = _mat2(J_qx, "J_qx")
        except Exception as exc:
            return self._fallback(
                J_qx=None,
                condJ=float("inf"),
                sigma_min=0.0,
                sigma_max=0.0,
                debug_msg=f"invalid Jacobian: {exc}",
            )

        try:
            singular_values = np.linalg.svd(J, compute_uv=False)
        except np.linalg.LinAlgError as exc:
            return self._fallback(
                J_qx=J,
                condJ=float("inf"),
                sigma_min=0.0,
                sigma_max=0.0,
                debug_msg=f"SVD failed: {exc}",
            )

        sigma_min = float(np.min(singular_values))
        sigma_max = float(np.max(singular_values))
        condJ = (
            float("inf")
            if sigma_min <= 1e-15
            else float(sigma_max / sigma_min)
        )

        if (
            not np.isfinite(condJ)
            or condJ > self.cfg.cond_fail
            or sigma_min < self.cfg.sigma_min_thresh
        ):
            return self._fallback(
                J_qx=J,
                condJ=condJ,
                sigma_min=sigma_min,
                sigma_max=sigma_max,
                debug_msg=(
                    "Jacobian invalid/near-singular: "
                    f"condJ={condJ:.6g}, sigma_min={sigma_min:.6g}"
                ),
            )

        try:
            kp_fit = self.fit_diagonal_gain(
                J_qx=J,
                desired_virtual=self.Kv_desired,
                gain_min=self.cfg.actuator_kp_min,
                gain_max=self.cfg.actuator_kp_max,
            )
            kd_fit = self.fit_diagonal_gain(
                J_qx=J,
                desired_virtual=self.Dv_desired,
                gain_min=self.cfg.actuator_kd_min,
                gain_max=self.cfg.actuator_kd_max,
            )
        except Exception as exc:
            return self._fallback(
                J_qx=J,
                condJ=condJ,
                sigma_min=sigma_min,
                sigma_max=sigma_max,
                debug_msg=f"gain fitting failed: {exc}",
            )

        if not kp_fit.valid or not kd_fit.valid:
            return self._fallback(
                J_qx=J,
                condJ=condJ,
                sigma_min=sigma_min,
                sigma_max=sigma_max,
                debug_msg="gain fitting produced invalid output",
            )

        # A fit error larger than the fail threshold means the independent
        # actuator gains cannot represent the requested virtual impedance
        # sufficiently well at this pose.
        if (
            kp_fit.relative_error > self.cfg.relative_fit_error_fail
            or kd_fit.relative_error > self.cfg.relative_fit_error_fail
        ):
            return self._fallback(
                J_qx=J,
                condJ=condJ,
                sigma_min=sigma_min,
                sigma_max=sigma_max,
                debug_msg=(
                    "fit error exceeds fail threshold: "
                    f"K={kp_fit.relative_error:.3f}, "
                    f"D={kd_fit.relative_error:.3f}"
                ),
            )

        kp_cmd = self._filter_gain(
            raw=kp_fit.gain_bounded,
            prev=self.kp_prev_,
            dt=dt,
            slew_rate=float(self.cfg.kp_slew_rate),
            gain_min=self.cfg.actuator_kp_min,
            gain_max=self.cfg.actuator_kp_max,
        )
        kd_cmd = self._filter_gain(
            raw=kd_fit.gain_bounded,
            prev=self.kd_prev_,
            dt=dt,
            slew_rate=float(self.cfg.kd_slew_rate),
            gain_min=self.cfg.actuator_kd_min,
            gain_max=self.cfg.actuator_kd_max,
        )

        achieved_Kv = self.equivalent_virtual_gain(J, kp_cmd)
        achieved_Dv = self.equivalent_virtual_gain(J, kd_cmd)

        stiffness_error = _relative_frobenius_error(
            achieved_Kv, self.Kv_desired
        )
        damping_error = _relative_frobenius_error(
            achieved_Dv, self.Dv_desired
        )

        degraded_reasons = []
        if estimator_degraded:
            degraded_reasons.append("state estimator degraded")
        if condJ > self.cfg.cond_warn:
            degraded_reasons.append(
                f"condJ={condJ:.3f} > warn={self.cfg.cond_warn:.3f}"
            )
        if kp_fit.saturated:
            degraded_reasons.append("Kp constrained by actuator limits")
        if kd_fit.saturated:
            degraded_reasons.append("Kd constrained by actuator limits")
        if stiffness_error > self.cfg.relative_fit_error_warn:
            degraded_reasons.append(
                f"stiffness error={stiffness_error:.3f}"
            )
        if damping_error > self.cfg.relative_fit_error_warn:
            degraded_reasons.append(
                f"damping error={damping_error:.3f}"
            )

        degraded = bool(degraded_reasons)
        saturated = bool(kp_fit.saturated or kd_fit.saturated)

        self.kp_prev_ = kp_cmd.copy()
        self.kd_prev_ = kd_cmd.copy()
        self.has_valid_output_ = True

        return RSUImpedanceResult(
            valid=True,
            degraded=degraded,
            fallback_used=False,
            saturated=saturated,
            kp=kp_cmd.copy(),
            kd=kd_cmd.copy(),
            J_qx=J.copy(),
            condJ=condJ,
            sigma_min=sigma_min,
            sigma_max=sigma_max,
            requested_Kv=self.Kv_desired.copy(),
            requested_Dv=self.Dv_desired.copy(),
            achieved_Kv=achieved_Kv.copy(),
            achieved_Dv=achieved_Dv.copy(),
            stiffness_error=stiffness_error,
            damping_error=damping_error,
            debug_msg="; ".join(degraded_reasons),
        )


if __name__ == "__main__":
    # Representative neutral-pose Jacobian from the previous RSU analysis.
    J_example = np.array([
        [1.2, 1.2],
        [-1.2, 1.2],
    ], dtype=np.float64)

    cfg = RSUImpedanceMapperConfig(
        actuator_kp_min=np.array([5.0, 5.0]),
        actuator_kp_max=np.array([25.0, 25.0]),
        actuator_kd_min=np.array([0.2, 0.2]),
        actuator_kd_max=np.array([6.0, 6.0]),
        gain_lpf_cutoff_hz=0.0,  # bypass filtering for this example
        kp_slew_rate=0.0,
        kd_slew_rate=0.0,
    )

    mapper = RSUImpedanceMapper(cfg)
    result = mapper.compute(
        J_qx=J_example,
        dt=0.01,
        estimator_valid=True,
        estimator_degraded=False,
    )

    print("valid              :", result.valid)
    print("degraded           :", result.degraded)
    print("fallback_used      :", result.fallback_used)
    print("kp                 :", result.kp)
    print("kd                 :", result.kd)
    print("condJ              :", result.condJ)
    print("sigma_min          :", result.sigma_min)
    print("requested Kv       :\n", result.requested_Kv)
    print("achieved Kv        :\n", result.achieved_Kv)
    print("requested Dv       :\n", result.requested_Dv)
    print("achieved Dv        :\n", result.achieved_Dv)
    print("stiffness error    :", result.stiffness_error)
    print("damping error      :", result.damping_error)
    print("debug              :", result.debug_msg)