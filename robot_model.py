"""
robot_model.py
Kinova Gen3 7-DOF robot model wrapper using roboticstoolbox-python.
Provides FK, IK, Jacobian, mass matrix, Coriolis, gravity torque.
Equivalent to MATLAB loadrobot('kinovaGen3','DataFormat','column')
"""

import numpy as np
import roboticstoolbox as rtb
from scipy.optimize import approx_fprime
from spatialmath import SE3


class KinovaGen3:
    """
    Wrapper around roboticstoolbox KinovaGen3 model.
    Exposes only the 7 arm joints, ignoring the gripper.
    """

    N_JOINTS   = 7
    EE_LINK    = 'end_effector_link'
    GRAVITY    = np.array([0, 0, -9.80665])

    # Hardware torque limits [Nm]
    TAU_MAX = np.array([187, 187, 187, 52, 52, 52, 52], dtype=float)

    # Joint limits [rad]
    Q_LIM_LO = np.deg2rad([-138.1, -138.1, -152.4, -127.8,
                            -119.7, -119.7, -119.7])
    Q_LIM_HI = np.deg2rad([ 138.1,  138.1,  152.4,  127.8,
                             119.7,  119.7,  119.7])

    # Joint velocity limits [rad/s]
    QD_MAX = np.array([1.396, 1.396, 1.396, 1.745, 1.745, 1.745, 1.745])

    # Arm links used for dynamics computation
    _ARM_LINKS = ['shoulder_link','half_arm_1_link','half_arm_2_link',
                  'forearm_link','spherical_wrist_1_link',
                  'spherical_wrist_2_link','bracelet_link','end_effector_link']

    def __init__(self):
        self._robot = rtb.models.KinovaGen3()
        self._robot.gravity = self.GRAVITY
        # Extract 7-DOF arm ETS chain for FK and IK
        self._arm_ets = self._robot.ets(end=self.EE_LINK)
        # Fix None joint limits in ETS (gripper joints have no qlim)
        _limits = [[-2.41,2.41],[-2.41,2.41],[-2.66,2.66],
                   [-2.23,2.23],[-2.09,2.09],[-2.09,2.09],[-2.09,2.09]]
        j = 0
        for e in self._arm_ets:
            if e.isjoint:
                if e.qlim[0] is None or e.qlim[1] is None:
                    e.qlim = np.array(_limits[j])
                j += 1
        self._q_home_full = np.zeros(self._robot.n)

    @property
    def n(self):
        return self.N_JOINTS

    def home_config(self):
        """Return 7-DOF home configuration (all zeros)."""
        return np.zeros(self.N_JOINTS)

    def _full_q(self, q7):
        """Extend 7-DOF joint vector to full 13-DOF (gripper stays at 0)."""
        q_full = np.zeros(self._robot.n)
        q_full[:self.N_JOINTS] = q7
        return q_full

    def fkine(self, q):
        """
        Forward kinematics.
        Returns SE3 transform of end-effector.
        q: (7,) joint angles in radians
        """
        T = self._robot.fkine(self._full_q(q), end=self.EE_LINK)
        return T

    def ee_position(self, q):
        """Return EE position [x, y, z] in metres."""
        return self.fkine(q).t

    def ikine(self, T_target, q0=None):
        """
        Inverse kinematics.
        T_target: SE3 target transform
        q0: (7,) initial guess (default: home)
        Returns: (q, success, error_mm)
        """
        if q0 is None:
            q0 = self.home_config()

        sol = self._arm_ets.ikine_LM(
            T_target,
            q0=q0[:self.N_JOINTS],
            mask=[1, 1, 1, 1, 1, 1],
            tol=1e-6,
            joint_limits=True
        )

        if sol.success:
            q_sol = sol.q[:self.N_JOINTS]
            # Wrap to [-pi, pi]
            q_sol = np.arctan2(np.sin(q_sol), np.cos(q_sol))
            # Verify
            T_check = self.fkine(q_sol)
            err_mm = np.linalg.norm(T_check.t - T_target.t) * 1000
            return q_sol, True, err_mm
        else:
            return q0, False, 999.0

    def jacobian(self, q):
        """
        Geometric Jacobian (6x7) at configuration q.
        q: (7,) joint angles
        """
        J_full = self._robot.jacobe(self._full_q(q), end=self.EE_LINK)
        return J_full[:, :self.N_JOINTS]

    def _link_jacobians(self, q):
        """Linear velocity Jacobians for each arm link (padded to 7 cols)."""
        q_f = self._full_q(q)
        Js, ms = [], []
        for link in self._robot.links:
            if link.name in self._ARM_LINKS and link.m and link.m > 0:
                J_full = self._robot.jacob0(q_f, end=link.name)
                Jv = np.zeros((3, self.N_JOINTS))
                cols = min(J_full.shape[1], self.N_JOINTS)
                Jv[:, :cols] = J_full[:3, :cols]
                Js.append(Jv); ms.append(link.m)
        return Js, ms

    def mass_matrix(self, q):
        """7x7 joint-space mass matrix M(q) via Jacobian sum."""
        Js, ms = self._link_jacobians(q)
        M = np.zeros((self.N_JOINTS, self.N_JOINTS))
        for Jv, m in zip(Js, ms):
            M += m * (Jv.T @ Jv)
        return M

    def coriolis(self, q, qd):
        """Coriolis/centrifugal torque vector C(q,qd)*qd — (7,)."""
        eps = 1e-5
        n = self.N_JOINTS
        # dM/dqi * qd summed over i, minus 0.5 * grad_q(M*qd)
        C_qd = np.zeros(n)
        for i in range(n):
            q_p = q.copy(); q_p[i] += eps
            q_m = q.copy(); q_m[i] -= eps
            dM = (self.mass_matrix(q_p) - self.mass_matrix(q_m)) / (2*eps)
            C_qd += dM @ qd
        grad_Mqd = approx_fprime(q, lambda qq: self.mass_matrix(qq) @ qd, eps)
        return C_qd - 0.5 * grad_Mqd

    def gravity_torque(self, q):
        """Gravity torque vector G(q) — (7,) via potential energy gradient."""
        def potential(qq):
            q_f = self._full_q(qq)
            E = 0.0
            for link in self._robot.links:
                if link.name in self._ARM_LINKS and link.m and link.m > 0:
                    T = self._robot.fkine(q_f, end=link.name)
                    E += link.m * 9.80665 * T.t[2]
            return E
        return approx_fprime(q, potential, 1e-6)

    def ctc_torque(self, q, qd, qdd_ff, Kp, Kd, q_ref, qd_ref):
        """
        Compute CTC torque:
        tau = M(q) * [qdd_ff + Kp*(q_ref-q) + Kd*(qd_ref-qd)] + C + G
        All inputs: (7,) arrays
        Returns: (7,) clamped torque
        """
        e   = q_ref  - q
        ed  = qd_ref - qd
        M   = self.mass_matrix(q)
        C   = self.coriolis(q, qd)
        G   = self.gravity_torque(q)
        tau = M @ (qdd_ff + Kp * e + Kd * ed) + C + G
        return np.clip(tau, -self.TAU_MAX, self.TAU_MAX)

    def target_transform(self, xyz, orientation='down'):
        """
        Build SE3 target transform for IK.
        xyz: [x, y, z] in metres
        orientation: 'down' (-Z), 'up' (+Z), 'horiz', 'tilt45'
        """
        t = np.array(xyz)
        if orientation == 'down':
            # Rotate 180 deg around Y axis — EE pointing down
            R = SE3.Ry(np.pi).R
        elif orientation == 'up':
            R = np.eye(3)
        elif orientation == 'horiz':
            R = SE3.Ry(np.pi / 2).R
        elif orientation == 'tilt45':
            R = SE3.Ry(3 * np.pi / 4).R
        else:
            R = SE3.Ry(np.pi).R
        return SE3.Rt(R, t)


# ── Quick test ────────────────────────────────────────────────────────────────
if __name__ == '__main__':
    print("Loading Kinova Gen3 model...")
    robot = KinovaGen3()
    q0 = robot.home_config()

    print(f"DOF: {robot.n}")
    print(f"Home EE position: {robot.ee_position(q0)}")

    # Test FK
    T = robot.fkine(q0)
    print(f"Home FK:\n  pos={T.t}")

    # Test IK
    target = robot.target_transform([0.5, 0.0, 0.3], 'down')
    q_sol, ok, err = robot.ikine(target, q0)
    print(f"\nIK to [0.5, 0.0, 0.3]:")
    print(f"  Success: {ok}")
    print(f"  Error:   {err:.3f} mm")
    print(f"  q_sol:   {np.round(np.rad2deg(q_sol), 2)} deg")

    # Test dynamics
    print(f"\nGravity torque at home: {np.round(robot.gravity_torque(q0), 3)} Nm")
    print(f"Mass matrix [0,0]: {robot.mass_matrix(q0)[0,0]:.4f} kg*m^2")

    print("\nrobot_model.py — OK")