"""
Safety analysis runner.

Ties together the existing robot model + trajectory (read only) with the new
human model and safety monitor, and produces a per-timestep safety analysis.

This is the independent safety layer: it imports the existing project as a
library and runs its own analysis. It modifies nothing.

Can be used two ways:
  * `analyse_trajectory(...)` — pure function, given arrays, returns results.
  * `run_scenario(...)` — generates a trajectory via the existing generator,
    then analyses it (needs roboticstoolbox + the project on the path).
"""

from __future__ import annotations

import os
import sys

import numpy as np

# Allow importing the existing project modules that live one level up.
_REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _REPO_ROOT not in sys.path:
    sys.path.insert(0, _REPO_ROOT)

from safety.human_model import SimulatedHuman
from safety.safety_monitor import SafetyMonitor, SafetyParams


def analyse_trajectory(robot, t, q_traj, qd_traj, tau_traj=None,
                       human: SimulatedHuman | None = None,
                       params: SafetyParams | None = None):
    """Run the safety monitor across a trajectory. Returns a results dict of arrays.

    t        : (N,) time
    q_traj   : (N,7) joint positions
    qd_traj  : (N,7) joint velocities
    tau_traj : (N,7) joint torques (optional)
    human    : SimulatedHuman (optional; enables SSM)
    """
    monitor = SafetyMonitor(robot, params)
    t = np.asarray(t, dtype=float).ravel()
    N = len(t)

    states, scales, seps, sps = [], [], [], []
    all_viol, all_warn = [], []
    human_path = []

    # Pre-flight plan validation (SG8).
    plan_ok, plan_issues = monitor.validate_plan(t, q_traj, qd_traj)

    for k in range(N):
        q = q_traj[k]
        qd = qd_traj[k]
        tau = tau_traj[k] if tau_traj is not None else None

        human_pos = None
        if human is not None:
            human_pos = human.position_at(t[k])
            human_path.append(human_pos)

        res = monitor.evaluate(q, qd, tau, t=t[k], human_pos=human_pos)
        states.append(res.state.value)
        scales.append(res.speed_scale)
        seps.append(res.separation)
        sps.append(res.protective_distance)
        all_viol.append(res.violations)
        all_warn.append(res.warnings)

    return {
        "t": t,
        "state": states,
        "speed_scale": np.array(scales),
        "separation": np.array(seps),
        "protective_distance": np.array(sps),
        "violations": all_viol,
        "warnings": all_warn,
        "human_path": np.array(human_path) if human_path else None,
        "plan_ok": plan_ok,
        "plan_issues": plan_issues,
        "stopped": any(s == "Safe stop" for s in states),
    }


def run_scenario(target_xyz=(0.4, 0.2, 0.3), tool_speed=0.20, curve_type="cubic",
                 human_start=(2.0, 0.0, 0.4), human_toward=(0.3, 0.0, 0.4),
                 params: SafetyParams | None = None):
    """Generate a trajectory with the existing generator, then analyse it.

    Requires the full project environment (roboticstoolbox etc.).
    """
    from generate_trajectory import generate_trajectory
    from robot_model import KinovaGen3

    robot = KinovaGen3()
    traj = generate_trajectory(robot, list(target_xyz), tool_speed=tool_speed,
                               curve_type=curve_type)
    q = traj["q_ref_traj"]
    qd = traj["qd_ref_traj"]
    t = q[:, 0]
    q7 = q[:, 1:8]
    qd7 = qd[:, 1:8]

    human = SimulatedHuman(start_xyz=human_start, toward_xyz=human_toward,
                           approach_speed=(params or SafetyParams()).v_human)
    return robot, analyse_trajectory(robot, t, q7, qd7, human=human, params=params)


def summarise(results):
    """Human-readable summary of a results dict."""
    n = len(results["t"])
    n_stop = sum(1 for s in results["state"] if s == "Safe stop")
    n_reduced = sum(1 for s in results["state"] if s == "Reduced speed")
    return {
        "samples": n,
        "plan_ok": results["plan_ok"],
        "reduced_speed_samples": n_reduced,
        "safe_stop_samples": n_stop,
        "min_separation": float(np.nanmin(results["separation"])) if n else None,
        "triggered_safe_stop": results["stopped"],
    }
