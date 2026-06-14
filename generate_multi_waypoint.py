"""
generate_multi_waypoint.py
Multi-waypoint cubic Hermite trajectory with junction velocity blending.
Python equivalent of generateMultiWaypoint.m
"""

import numpy as np
from robot_model import KinovaGen3


def generate_multi_waypoint(robot: KinovaGen3,
                             waypoints: np.ndarray,
                             orientations: list,
                             speeds: np.ndarray) -> dict:
    """
    Build a multi-waypoint cubic Hermite trajectory.

    Parameters
    ----------
    robot        : KinovaGen3 instance
    waypoints    : (N x 3) array of [x, y, z] targets in metres
    orientations : list of N strings ('Down','Up','Horiz','Tilt45')
    speeds       : (N,) array of tool speeds in m/s per segment

    Returns
    -------
    dict with keys:
        q_ref_traj    : (N x 8) [time, q1..q7]
        qd_ref_traj   : (N x 8)
        tau_ref_traj  : (N x 8) accelerations rad/s²
        T_total       : float
        seg_T         : list of segment durations
        ik_err_mm     : float (max across waypoints)
    """
    dt     = 0.001
    n_wps  = len(waypoints)

    # ── IK for each waypoint ──────────────────────────────────────────────────
    print(f'  [MW] Solving IK for {n_wps} waypoints...')
    q_wps    = np.zeros((7, n_wps + 1))   # +1 for home start
    q_wps[:, 0] = robot.home_config()
    max_ik_err = 0.0

    q_prev = robot.home_config()
    for w in range(n_wps):
        ori = orientations[w] if orientations[w] else 'Down'
        T_target = robot.target_transform(waypoints[w], ori.lower())
        q_sol, ok, err = robot.ikine(T_target, q_prev)
        if not ok:
            raise ValueError(f'IK failed at waypoint {w+1}: err={err:.1f}mm')
        q_wps[:, w+1] = q_sol
        max_ik_err = max(max_ik_err, err)
        q_prev = q_sol
        print(f'    WP{w+1}: IK={err:.3f}mm  q[1]={np.rad2deg(q_sol[1]):.1f}°')

    # ── Segment timings ───────────────────────────────────────────────────────
    seg_T = []
    for w in range(n_wps):
        qs = q_wps[:, w]
        qe = q_wps[:, w+1]
        dist = np.linalg.norm(qs - qe)
        spd  = speeds[w] if w < len(speeds) else 0.20
        seg_T.append(max(round(dist / spd, 1), 1.0))

    print(f'  [MW] Segment times: {[round(t,1) for t in seg_T]} s')

    # ── Junction velocity blending ────────────────────────────────────────────
    # Catmull-Rom style: average of adjacent segment velocities
    n_segs = n_wps
    jv = np.zeros((7, n_segs + 1))  # junction velocities (zero at start/end)

    for w in range(1, n_segs):
        di = (q_wps[:, w]   - q_wps[:, w-1]) / seg_T[w-1]
        do = (q_wps[:, w+1] - q_wps[:, w])   / seg_T[w]
        vj = (seg_T[w] * di + seg_T[w-1] * do) / (seg_T[w-1] + seg_T[w])
        # Zero velocity at direction reversals
        for j in range(7):
            if np.sign(di[j]) != np.sign(do[j]):
                vj[j] = 0.0
        jv[:, w] = vj

    # ── Build cubic Hermite segments ──────────────────────────────────────────
    all_t   = []
    all_pos = []
    all_vel = []
    all_acc = []
    t_offset = 0.0

    for w in range(n_segs):
        qs  = q_wps[:, w]
        qe  = q_wps[:, w+1]
        qds = jv[:, w]
        qde = jv[:, w+1]
        T_  = seg_T[w]

        ts = np.arange(0, T_ + dt, dt)
        Ns = len(ts)
        p_ = np.zeros((7, Ns))
        v_ = np.zeros((7, Ns))
        a_ = np.zeros((7, Ns))

        for j in range(7):
            a0 = qs[j];  a1 = qds[j]
            a2 = (3*(qe[j]-qs[j])/T_**2) - (2*qds[j]/T_) - (qde[j]/T_)
            a3 = (-2*(qe[j]-qs[j])/T_**3) + ((qds[j]+qde[j])/T_**2)
            p_[j] = a0 + a1*ts + a2*ts**2 + a3*ts**3
            v_[j] = a1 + 2*a2*ts + 3*a3*ts**2
            a_[j] = 2*a2 + 6*a3*ts

        # Remove duplicate junction point (except last segment)
        if w < n_segs - 1:
            p_ = p_[:, :-1]; v_ = v_[:, :-1]; a_ = a_[:, :-1]
            ts = ts[:-1]

        all_t.append(ts + t_offset)
        all_pos.append(p_)
        all_vel.append(v_)
        all_acc.append(a_)
        t_offset += ts[-1] + dt

    # ── Concatenate ───────────────────────────────────────────────────────────
    t_full   = np.concatenate(all_t)
    pos_full = np.concatenate(all_pos, axis=1)
    vel_full = np.concatenate(all_vel, axis=1)
    acc_full = np.concatenate(all_acc, axis=1)
    T_total  = t_full[-1]

    time_col = t_full.reshape(-1, 1)
    Kp_base = np.array([100,100,80,60,40,40,20], dtype=float)
    Kd_base = np.array([ 20, 20,16,12, 8, 8, 4], dtype=float)
    return {
        'q_ref_traj':   np.hstack([time_col, pos_full.T]),
        'qd_ref_traj':  np.hstack([time_col, vel_full.T]),
        'tau_ref_traj': np.hstack([time_col, acc_full.T]),
        'T_total':      T_total,
        'seg_T':        seg_T,
        'ik_err_mm':    max_ik_err,
        'Kp':           Kp_base * 3.0,
        'Kd':           Kd_base * 3.0,
    }


# ── Quick test ────────────────────────────────────────────────────────────────
if __name__ == '__main__':
    print("Loading robot...")
    robot = KinovaGen3()

    waypoints = np.array([
        [0.5,  0.0, 0.3],
        [0.4,  0.3, 0.4],
        [0.3, -0.2, 0.35],
    ])
    orientations = ['Down', 'Down', 'Down']
    speeds       = np.array([0.20, 0.15, 0.20])

    print("\nGenerating 3-waypoint trajectory...")
    res = generate_multi_waypoint(robot, waypoints, orientations, speeds)

    print(f"\nResults:")
    print(f"  T_total:      {res['T_total']:.2f} s")
    print(f"  Seg times:    {[round(t,1) for t in res['seg_T']]} s")
    print(f"  Max IK error: {res['ik_err_mm']:.3f} mm")
    print(f"  N steps:      {res['q_ref_traj'].shape[0]}")
    print(f"  Shape:        {res['q_ref_traj'].shape}")
    print(f"  q_start:      {np.round(np.rad2deg(res['q_ref_traj'][0,1:]),2)} deg")
    print(f"  q_end:        {np.round(np.rad2deg(res['q_ref_traj'][-1,1:]),2)} deg")
    print(f"  vel_start:    {np.round(res['qd_ref_traj'][0,1:],4)}")
    print(f"  vel_end:      {np.round(res['qd_ref_traj'][-1,1:],4)}")

    print("\ngenerate_multi_waypoint.py — OK")