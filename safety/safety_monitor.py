"""
Safety monitor for the Kinova Gen3 collaborative safety layer.

An INDEPENDENT observer: it reads the robot model and a trajectory and produces a
safety decision (OK / reduced speed / safe stop). It never modifies the robot
model, the controller, or the Simulink file — following the functional-safety
principle of freedom from interference between the safety and functional channels.

Implements the requirements in SAF-SRS-001. Each check maps to a requirement ID.

Scope: concept demonstration. Thresholds are illustrative ISO/TS 15066 / ISO 10218
defaults, not a certified biomechanical assessment.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum

import numpy as np


class SafetyState(Enum):
    NORMAL = "Normal"
    REDUCED = "Reduced speed"
    SAFE_STOP = "Safe stop"


@dataclass
class SafetyParams:
    """Illustrative safety parameters (SAF-SRS-001 §2), with reasoned defaults."""

    v_safe: float = 0.25            # m/s  safe collaborative TCP speed (ISO/TS 15066)
    tau_fraction: float = 0.50      # collaborative torque = 50% of rated TAU_MAX
    v_human: float = 1.6            # m/s  human approach speed (ISO/TS 15066)
    reaction_time: float = 0.10     # s    detection-to-response latency
    stop_time: float = 0.20         # s    robot stopping time
    s_min: float = 0.30             # m    minimum protective distance
    margin: float = 0.10            # m    combined uncertainty (C + Z_d + Z_r)
    # a_R (deceleration) derived from a representative TCP speed and stop_time:
    #   a_R = v_representative / stop_time. Reasoned default below.
    decel: float = 1.25             # m/s^2  (=0.25 m/s / 0.20 s)
    # Manipulability warning threshold. For a 7-DOF arm, w = sqrt(det(J*J^T));
    # a small positive floor flags near-singular configurations. Reasoned default.
    w_min: float = 0.02


@dataclass
class SafetyResult:
    """Per-timestep evaluation result."""

    state: SafetyState = SafetyState.NORMAL
    speed_scale: float = 1.0
    separation: float = float("inf")
    protective_distance: float = 0.0
    violations: list = field(default_factory=list)   # requirement IDs violated
    warnings: list = field(default_factory=list)


class SafetyMonitor:
    """Evaluates the safety functions defined in SAF-SRS-001."""

    def __init__(self, robot, params: SafetyParams | None = None):
        # `robot` is an existing KinovaGen3 instance — read only.
        self.robot = robot
        self.p = params or SafetyParams()
        self.tau_col = self.p.tau_fraction * robot.TAU_MAX
        self._latched = False   # SR-9.2: safe stop latches until reset

    def reset(self):
        self._latched = False

    # ---- plan-level checks (pre-flight) --------------------------------
    def validate_plan(self, t, q_ref, qd_ref):
        """SG8: reject invalid plans before execution. Returns (ok, issues)."""
        issues = []
        arrs = {"t": np.asarray(t), "q_ref": np.asarray(q_ref), "qd_ref": np.asarray(qd_ref)}
        for name, a in arrs.items():
            if not np.all(np.isfinite(a)):
                issues.append(f"SR-8.1: {name} contains NaN/Inf")
        tt = np.asarray(t).ravel()
        if tt.size >= 2 and not np.all(np.diff(tt) > 0):
            issues.append("SR-8.2: time vector not strictly increasing")
        # SR-4.2 / SR-5.1: planned range and speed
        q = np.asarray(q_ref)
        qd = np.asarray(qd_ref)
        if q.ndim == 2:
            if np.any(q < robot_lo(self.robot)) or np.any(q > robot_hi(self.robot)):
                issues.append("SR-4.2: planned joint position out of range")
        if qd.ndim == 2 and np.any(np.abs(qd) > self.robot.QD_MAX + 1e-9):
            issues.append("SR-5.1: planned joint velocity exceeds QD_MAX")
        return (len(issues) == 0, issues)

    # ---- per-timestep evaluation ---------------------------------------
    def evaluate(self, q, qd, tau, t=0.0, human_pos=None, human_vel_toward=0.0) -> SafetyResult:
        """Evaluate all safety functions at one instant. Returns a SafetyResult."""
        q = np.asarray(q, dtype=float)
        qd = np.asarray(qd, dtype=float)
        res = SafetyResult()

        # SG4 / SR-4.1 — joint position limits (hard)
        if np.any(q < robot_lo(self.robot) - 1e-6) or np.any(q > robot_hi(self.robot) + 1e-6):
            res.violations.append("SR-4.1")

        # SG5 / SR-5.1 — joint velocity limits (hard)
        if np.any(np.abs(qd) > self.robot.QD_MAX + 1e-6):
            res.violations.append("SR-5.1")

        # SG1 / SG6 — torque limits
        if tau is not None:
            tau = np.asarray(tau, dtype=float)
            if np.any(np.abs(tau) > self.robot.TAU_MAX + 1e-6):     # SR-1.1 / SR-6.1 hard
                res.violations.append("SR-1.1")
            elif np.any(np.abs(tau) > self.tau_col + 1e-6):          # SR-1.2 collaborative soft
                res.warnings.append("SR-1.2")
            if np.any(np.abs(np.abs(tau) - self.robot.TAU_MAX) < 1e-3):  # SR-1.3 saturation
                res.warnings.append("SR-1.3")

        # TCP speed via Jacobian
        J = self.robot.jacobian(q)[:3, :]        # linear part
        v_tcp_vec = J @ qd
        v_tcp = float(np.linalg.norm(v_tcp_vec))

        # SG2 / SR-2.1 — Cartesian collaborative speed (soft: triggers reduce)
        speed_scale = 1.0
        if v_tcp > self.p.v_safe:
            res.warnings.append("SR-2.1")
            speed_scale = min(speed_scale, self.p.v_safe / max(v_tcp, 1e-9))

        # SG7 / SR-7.1 — manipulability / singularity warning
        JJ = J @ J.T
        w = float(np.sqrt(max(np.linalg.det(JJ), 0.0)))
        if w < self.p.w_min:
            res.warnings.append("SR-7.1")

        # SG3 — Speed & Separation Monitoring (the showpiece)
        if human_pos is not None:
            tcp = self.robot.ee_position(q)
            d = float(np.linalg.norm(np.asarray(human_pos, dtype=float) - tcp))
            res.separation = d
            # SR-3.2 — dynamic protective distance
            v_R = max(v_tcp, 0.0)
            s_p = (
                self.p.v_human * (self.p.reaction_time + self.p.stop_time)
                + v_R * self.p.reaction_time
                + (v_R ** 2) / (2.0 * self.p.decel)
                + self.p.margin
            )
            res.protective_distance = s_p
            if d <= self.p.s_min:                       # SR-3.5 — stop
                res.violations.append("SR-3.5")
                speed_scale = 0.0
            elif d < s_p:                               # SR-3.4 — proportional reduce
                res.warnings.append("SR-3.4")
                frac = (d - self.p.s_min) / max(s_p - self.p.s_min, 1e-9)
                speed_scale = min(speed_scale, float(np.clip(frac, 0.0, 1.0)))
            # else SR-3.3 — full speed permitted

        # SG9 — aggregate into a state and apply the safe-stop latch
        if res.violations or self._latched:             # SR-9.1 / SR-9.2
            self._latched = True
            res.state = SafetyState.SAFE_STOP
            res.speed_scale = 0.0
        elif speed_scale < 1.0 or res.warnings:
            res.state = SafetyState.REDUCED
            res.speed_scale = speed_scale
        else:
            res.state = SafetyState.NORMAL
            res.speed_scale = 1.0
        return res


# ---- helpers to read limits robustly -----------------------------------
def robot_lo(robot):
    return robot.Q_LIM_LO


def robot_hi(robot):
    return robot.Q_LIM_HI
