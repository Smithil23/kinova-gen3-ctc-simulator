"""
simulink_bridge.py
Connects Python to MATLAB Engine and runs KinovaCollisionFree.slx.
Python equivalent of the sim() call inside KinovaApp.m
"""

import numpy as np
import matlab
import matlab.engine
import os


class SimulinkBridge:
    """
    Manages a MATLAB Engine session and runs KinovaCollisionFree.slx.
    """

    MDL = 'KinovaCollisionFree'

    def __init__(self, slx_folder: str = None):
        """
        Parameters
        ----------
        slx_folder : path to folder containing KinovaCollisionFree.slx
                     If None, uses current working directory.
        """
        self._eng    = None
        self._folder = slx_folder or os.getcwd()

    def start(self):
        """Start MATLAB engine session."""
        if self._eng is not None:
            return
        print('[Bridge] Starting MATLAB engine...')
        self._eng = matlab.engine.start_matlab()
        # Add the slx folder to MATLAB path
        self._eng.addpath(self._folder, nargout=0)
        print('[Bridge] MATLAB engine ready.')

    def stop(self):
        """Stop MATLAB engine session."""
        if self._eng:
            self._eng.quit()
            self._eng = None
            print('[Bridge] MATLAB engine stopped.')

    def run(self,
            q_ref_traj:   np.ndarray,
            qd_ref_traj:  np.ndarray,
            tau_ref_traj: np.ndarray,
            Kp:           np.ndarray,
            Kd:           np.ndarray,
            T_total:      float) -> dict:
        """
        Push trajectory to MATLAB workspace, run Simulink, return Q_out.

        Parameters
        ----------
        q_ref_traj   : (N x 8) [time, q1..q7]
        qd_ref_traj  : (N x 8)
        tau_ref_traj : (N x 8) accelerations
        Kp           : (7,) proportional gains
        Kd           : (7,) derivative gains
        T_total      : simulation stop time in seconds

        Returns
        -------
        dict with:
            t_s   : (N,) time vector
            Q_out : (N x 7) actual joint angles from CTC simulation
        """
        if self._eng is None:
            self.start()

        eng = self._eng

        print('[Bridge] Pushing trajectory to MATLAB workspace...')
        # Convert numpy arrays to MATLAB double
        eng.workspace['q_ref_traj']   = matlab.double(q_ref_traj.tolist())
        eng.workspace['qd_ref_traj']  = matlab.double(qd_ref_traj.tolist())
        eng.workspace['tau_ref_traj'] = matlab.double(tau_ref_traj.tolist())
        # Load Kinova_DOF7 robot object required by Simulink dynamics blocks
        eng.eval("Kinova_DOF7 = loadrobot('kinovaGen3','DataFormat','column');"
                 "Kinova_DOF7.Gravity = [0 0 -9.80665];", nargout=0)
        print('[Bridge] Kinova_DOF7 loaded into MATLAB workspace.')

        print('[Bridge] Configuring Simulink model...')
        mdl = self.MDL
        eng.load_system(mdl, nargout=0)
        eng.set_param(mdl, 'StopTime', str(T_total), nargout=0)

        # Configure trajectory input blocks
        for blk in ['q_ref', 'qd_ref', 'tau_ref']:
            eng.set_param(f'{mdl}/{blk}',
                          'SampleTime', '0.001',
                          'Interpolate', 'on',
                          'OutputAfterFinalValue', 'Holding final value',
                          'ZeroCross', 'off',
                          nargout=0)

        eng.set_param(f'{mdl}/Unit Delay', 'SampleTime', '0.001', nargout=0)

        # Set CTC gains
        eng.set_param(f'{mdl}/Kp', 'Gain',
                      eng.mat2str(matlab.double(Kp.tolist())), nargout=0)
        eng.set_param(f'{mdl}/Kd', 'Gain',
                      eng.mat2str(matlab.double(Kd.tolist())), nargout=0)

        # Enable Simscape logging for Mechanics Explorer replay
        eng.set_param(mdl, 'SimscapeLogType',       'all',  nargout=0)
        eng.set_param(mdl, 'SimscapeLogOpenViewer', 'on',   nargout=0)
        eng.set_param(mdl, 'SimscapeLogToSDI',      'on',   nargout=0)


        print(f'[Bridge] Running KinovaCollisionFree.slx (T={T_total:.1f}s)...')
        out = eng.sim(mdl)

        print('[Bridge] Simulation complete. Extracting results...')
        # Store out in MATLAB workspace first, then extract via eval
        eng.workspace['out'] = out
        eng.eval("last_t_s  = out.logsout{1}.Values.Time;",  nargout=0)
        eng.eval("last_Qout = out.logsout{1}.Values.Data;",  nargout=0)
        t_s   = np.array(eng.workspace['last_t_s']).flatten()
        Q_out = np.array(eng.workspace['last_Qout'])
        # Keep in workspace for validateKinovaResults
        eng.eval("q_ref_traj_saved   = q_ref_traj;",   nargout=0)
        eng.eval("qd_ref_traj_saved  = qd_ref_traj;",  nargout=0)
        eng.eval("tau_ref_traj_saved = tau_ref_traj;",  nargout=0)

        print(f'[Bridge] Q_out shape: {Q_out.shape}  '
              f't: {t_s[0]:.3f}→{t_s[-1]:.3f}s')

        return {'t_s': t_s, 'Q_out': Q_out}

    def __enter__(self):
        self.start()
        return self

    def __exit__(self, *args):
        self.stop()


# ── Quick connection test (no sim) ────────────────────────────────────────────
if __name__ == '__main__':
    print("Testing MATLAB engine connection...")
    bridge = SimulinkBridge()
    bridge.start()

    eng = bridge._eng
    result = eng.eval('version', nargout=1)
    print(f"  MATLAB version: {result}")

    # Check .slx is accessible
    slx_exists = eng.eval(
        f"exist('KinovaCollisionFree.slx','file')", nargout=1)
    if slx_exists:
        print("  KinovaCollisionFree.slx found on MATLAB path")
    else:
        print("  WARNING: KinovaCollisionFree.slx not found")
        print(f"  Make sure it is in: {bridge._folder}")

    bridge.stop()
    print("\nsimulinkbridge.py — connection OK")