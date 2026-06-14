"""
generate_trajectory.py
Single-target joint-space trajectory generator.
Python equivalent of generateTrajectory.m
5 profiles: cubic, quintic, lspb, hermite, bangbang
"""

import numpy as np
from spatialmath import SE3
from robot_model import KinovaGen3


def generate_trajectory(robot: KinovaGen3,
                        target_xyz: list,
                        tool_speed: float = 0.20,
                        gain_scale: float = 3.0,
                        curve_type: str = 'cubic',
                        orientation: str = 'down') -> dict:
    """
    Build a joint-space trajectory to a single Cartesian target.

    Parameters
    ----------
    robot       : KinovaGen3 instance
    target_xyz  : [x, y, z] in metres
    tool_speed  : EE speed in m/s
    gain_scale  : CTC gain multiplier
    curve_type  : 'cubic' | 'quintic' | 'lspb' | 'hermite' | 'bangbang'

    Returns
    -------
    dict with keys:
        q_ref_traj    : (N x 8) [time, q1..q7]
        qd_ref_traj   : (N x 8)
        tau_ref_traj  : (N x 8) accelerations rad/s²
        T_total       : float
        ik_err_mm     : float
        Kp            : (7,) array
        Kd            : (7,) array
    """
    dt = 0.001
    tx, ty, tz = target_xyz

    # ── IK ────────────────────────────────────────────────────────────────────
    q0 = robot.home_config()
    T0 = robot.fkine(q0)
    T_target = robot.target_transform([tx, ty, tz], orientation)
    q_target, ok, ik_err_mm = robot.ikine(T_target, q0)

    if not ok or ik_err_mm > 5.0:
        raise ValueError(f'IK failed — error {ik_err_mm:.1f} mm > 5 mm. '
                         f'Target [{tx:.3f}, {ty:.3f}, {tz:.3f}] unreachable.')

    # ── Timing ────────────────────────────────────────────────────────────────
    distance = np.linalg.norm(T0.t - np.array([tx, ty, tz]))
    T_total  = max(round(distance / tool_speed, 1), 2.0)
    timestamp = np.arange(0, T_total + dt, dt)
    N = len(timestamp)

    # ── Trajectory profiles ───────────────────────────────────────────────────
    pos = np.zeros((7, N))
    vel = np.zeros((7, N))
    acc = np.zeros((7, N))
    t   = timestamp

    for j in range(7):
        qs = q0[j]
        qe = q_target[j]
        T  = T_total

        ct = curve_type.lower()

        if ct == 'quintic':
            a3 =  10*(qe-qs)/T**3
            a4 = -15*(qe-qs)/T**4
            a5 =   6*(qe-qs)/T**5
            pos[j] = qs + a3*t**3 + a4*t**4 + a5*t**5
            vel[j] = 3*a3*t**2 + 4*a4*t**3 + 5*a5*t**4
            acc[j] = 6*a3*t   +12*a4*t**2 +20*a5*t**3

        elif ct == 'lspb':
            tb = T/4
            av = (qe-qs) / (tb*(T-tb))
            for k, tk in enumerate(t):
                if tk < tb:
                    pos[j,k] = qs + 0.5*av*tk**2
                    vel[j,k] = av*tk
                    acc[j,k] = av
                elif tk < T-tb:
                    pos[j,k] = qs + av*tb*(tk - tb/2)
                    vel[j,k] = av*tb
                    acc[j,k] = 0.0
                else:
                    pos[j,k] = qe - 0.5*av*(T-tk)**2
                    vel[j,k] = av*(T-tk)
                    acc[j,k] = -av

        elif ct == 'bangbang':
            tm  = T/2
            av  = 4*(qe-qs)/T**2
            for k, tk in enumerate(t):
                if tk <= tm:
                    acc[j,k] = av
                    vel[j,k] = av*tk
                    pos[j,k] = qs + 0.5*av*tk**2
                else:
                    acc[j,k] = -av
                    vel[j,k] = av*(T-tk)
                    pos[j,k] = qe - 0.5*av*(T-tk)**2

        else:  # cubic (default) and hermite (same for single target)
            a2 =  3*(qe-qs)/T**2
            a3 = -2*(qe-qs)/T**3
            pos[j] = qs + a2*t**2 + a3*t**3
            vel[j] = 2*a2*t + 3*a3*t**2
            acc[j] = 2*a2   + 6*a3*t

    # ── Gains ─────────────────────────────────────────────────────────────────
    Kp_base = np.array([100, 100, 80, 60, 40, 40, 20], dtype=float)
    Kd_base = np.array([ 20,  20, 16, 12,  8,  8,  4], dtype=float)

    # ── Package result ────────────────────────────────────────────────────────
    time_col = timestamp.reshape(-1, 1)
    return {
        'q_ref_traj':   np.hstack([time_col, pos.T]),   # N x 8
        'qd_ref_traj':  np.hstack([time_col, vel.T]),
        'tau_ref_traj': np.hstack([time_col, acc.T]),
        'T_total':      T_total,
        'ik_err_mm':    ik_err_mm,
        'Kp':           Kp_base * gain_scale,
        'Kd':           Kd_base * gain_scale,
    }


# ── Quick test ────────────────────────────────────────────────────────────────
if __name__ == '__main__':
    print("Loading robot...")
    robot = KinovaGen3()

    print("Testing Cubic Spline trajectory to [0.5, 0.0, 0.3]...")
    res = generate_trajectory(robot, [0.5, 0.0, 0.3],
                              tool_speed=0.20,
                              gain_scale=3.0,
                              curve_type='cubic')

    print(f"  T_total:    {res['T_total']:.1f} s")
    print(f"  IK error:   {res['ik_err_mm']:.3f} mm")
    print(f"  N steps:    {res['q_ref_traj'].shape[0]}")
    print(f"  q_ref shape:{res['q_ref_traj'].shape}")
    print(f"  Kp:         {res['Kp']}")
    print(f"  q_start:    {np.round(np.rad2deg(res['q_ref_traj'][0,1:]), 2)} deg")
    print(f"  q_end:      {np.round(np.rad2deg(res['q_ref_traj'][-1,1:]), 2)} deg")

    # Verify vel=0 at start and end (cubic property)
    print(f"  vel_start:  {np.round(res['qd_ref_traj'][0,1:], 4)}")
    print(f"  vel_end:    {np.round(res['qd_ref_traj'][-1,1:], 4)}")

    print("\nTesting all 5 profiles...")
    for profile in ['cubic','quintic','lspb','hermite','bangbang']:
        r = generate_trajectory(robot, [0.4, 0.2, 0.35],
                                tool_speed=0.15,
                                curve_type=profile)
        q_end = np.rad2deg(r['q_ref_traj'][-1, 1:])
        print(f"  {profile:10s}: T={r['T_total']:.1f}s  "
              f"q_end[1]={q_end[1]:.2f}°  IK={r['ik_err_mm']:.3f}mm")

    print("\ngenerate_trajectory.py — OK")