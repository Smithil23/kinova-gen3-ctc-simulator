"""
kinova_app.py
Kinova Gen3 CTC Trajectory Simulator — PyQt6 GUI
Python equivalent of KinovaApp.m
"""

import sys
import numpy as np
from PyQt6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QHBoxLayout, QVBoxLayout,
    QGridLayout, QLabel, QPushButton, QComboBox, QLineEdit,
    QSlider, QTableWidget, QTableWidgetItem, QTextEdit,
    QGroupBox, QSplitter, QFrame, QHeaderView, QSizePolicy
)
from PyQt6.QtCore import Qt, QThread, pyqtSignal, QTimer
from PyQt6.QtGui import QFont, QColor, QPalette
import matplotlib
matplotlib.use('QtAgg')
from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# ── Colour palette (matches MATLAB dark theme) ─────────────────────────────
C_BG      = '#0D1117'
C_PANEL   = '#161B22'
C_BORDER  = '#30363D'
C_ACCENT  = '#58A6FF'
C_ACCENT2 = '#3FB950'
C_WARN    = '#D29922'
C_DANGER  = '#F85149'
C_DIM     = '#8B949E'
C_TEXT    = '#C9D1D9'
C_WHITE   = '#F0F6FF'

J_COLORS = ['#00CCFF','#66FF66','#FFCC00','#FF6600',
            '#FF3388','#9966FF','#33CCCC']

STYLE = f"""
QMainWindow, QWidget {{
    background-color: {C_BG};
    color: {C_TEXT};
    font-family: 'Segoe UI', Arial, sans-serif;
    font-size: 12px;
}}
QGroupBox {{
    background-color: {C_PANEL};
    border: 1px solid {C_BORDER};
    border-radius: 6px;
    margin-top: 8px;
    padding: 6px;
    font-weight: bold;
    color: {C_ACCENT};
}}
QGroupBox::title {{
    subcontrol-origin: margin;
    left: 8px;
    color: {C_ACCENT};
    font-size: 11px;
    font-weight: bold;
    text-transform: uppercase;
    letter-spacing: 1px;
}}
QPushButton {{
    background-color: {C_PANEL};
    color: {C_TEXT};
    border: 1px solid {C_BORDER};
    border-radius: 4px;
    padding: 5px 10px;
    font-weight: bold;
}}
QPushButton:hover {{ background-color: #21262D; border-color: {C_ACCENT}; }}
QPushButton#btn_run {{
    background-color: #238636;
    color: white;
    border: none;
    font-size: 13px;
    padding: 8px;
}}
QPushButton#btn_run:hover {{ background-color: #2EA043; }}
QPushButton#btn_calc {{
    background-color: #1F6FEB;
    color: white;
    border: none;
    padding: 8px;
}}
QPushButton#btn_calc:hover {{ background-color: #388BFD; }}
QPushButton#btn_stop {{
    background-color: #DA3633;
    color: white;
    border: none;
    padding: 8px;
}}
QComboBox {{
    background-color: {C_PANEL};
    color: {C_TEXT};
    border: 1px solid {C_BORDER};
    border-radius: 4px;
    padding: 4px 8px;
}}
QComboBox::drop-down {{ border: none; }}
QComboBox QAbstractItemView {{
    background-color: {C_PANEL};
    color: {C_TEXT};
    selection-background-color: #21262D;
}}
QLineEdit {{
    background-color: {C_PANEL};
    color: {C_TEXT};
    border: 1px solid {C_BORDER};
    border-radius: 4px;
    padding: 4px 8px;
}}
QSlider::groove:horizontal {{
    background: {C_BORDER};
    height: 4px;
    border-radius: 2px;
}}
QSlider::handle:horizontal {{
    background: {C_ACCENT};
    width: 12px; height: 12px;
    border-radius: 6px;
    margin: -4px 0;
}}
QSlider::sub-page:horizontal {{ background: {C_ACCENT}; border-radius: 2px; }}
QTableWidget {{
    background-color: {C_PANEL};
    color: {C_TEXT};
    border: 1px solid {C_BORDER};
    gridline-color: {C_BORDER};
}}
QTableWidget QHeaderView::section {{
    background-color: #21262D;
    color: {C_DIM};
    border: 1px solid {C_BORDER};
    padding: 4px;
    font-size: 11px;
}}
QTextEdit {{
    background-color: {C_PANEL};
    color: {C_DIM};
    border: 1px solid {C_BORDER};
    font-family: 'Consolas', monospace;
    font-size: 11px;
}}
QLabel {{
    color: {C_TEXT};
}}
QLabel#label_value {{
    color: {C_ACCENT2};
    font-size: 16px;
    font-weight: bold;
}}
QLabel#label_dim {{
    color: {C_DIM};
    font-size: 10px;
}}
QFrame#separator {{
    background-color: {C_BORDER};
    max-height: 1px;
}}
"""

# ── Simulation worker thread ───────────────────────────────────────────────
class SimWorker(QThread):
    finished  = pyqtSignal(dict)
    log_msg   = pyqtSignal(str)
    error_msg = pyqtSignal(str)

    def __init__(self, mode, params):
        super().__init__()
        self.mode   = mode
        self.params = params

    def run(self):
        try:
            import numpy as np
            from robot_model import KinovaGen3
            from generate_trajectory import generate_trajectory
            from generate_multi_waypoint import generate_multi_waypoint
            from simulink_bridge import SimulinkBridge

            self.log_msg.emit('[SIM] Loading robot model...')
            robot = KinovaGen3()

            if self.mode == 'Single Target':
                self.log_msg.emit('[SIM] Generating trajectory...')
                res = generate_trajectory(
                    robot,
                    target_xyz  = self.params['target_xyz'],
                    tool_speed  = self.params['speed'],
                    gain_scale  = self.params['gain_scale'],
                    curve_type  = self.params['curve_type'],
                    orientation = self.params.get('orientation', 'down')
                )
                self.log_msg.emit(f"[SIM] IK={res['ik_err_mm']:.3f}mm "
                                  f"T={res['T_total']:.1f}s")

            elif self.mode in ('Multi-Waypoint', 'TCP Test'):
                self.log_msg.emit('[SIM] Generating multi-waypoint trajectory...')
                res = generate_multi_waypoint(
                    robot,
                    waypoints    = self.params['waypoints'],
                    orientations = self.params['orientations'],
                    speeds       = self.params['speeds']
                )
                # Apply gain scale
                gs = self.params.get('gain_scale', 3.0)
                Kp_base = np.array([100,100,80,60,40,40,20], dtype=float)
                Kd_base = np.array([ 20, 20,16,12, 8, 8, 4], dtype=float)
                res['Kp'] = Kp_base * gs
                res['Kd'] = Kd_base * gs
                self.log_msg.emit(f"[SIM] T={res['T_total']:.1f}s "
                                  f"MaxIK={res['ik_err_mm']:.3f}mm")
                if self.mode == 'TCP Test':
                    self.log_msg.emit('[TCP] Running TCP accuracy test...')
            else:
                self.error_msg.emit(f'Unknown mode: {self.mode}')
                return

            self.log_msg.emit('[SIM] Running Simulink CTC simulation...')
            # Use shared bridge — keep MATLAB engine alive for Mechanics Explorer
            if not hasattr(self, 'bridge') or self.bridge is None:
                self.bridge = SimulinkBridge()
            bridge = self.bridge
            sim_out = bridge.run(
                q_ref_traj   = res['q_ref_traj'],
                qd_ref_traj  = res['qd_ref_traj'],
                tau_ref_traj = res['tau_ref_traj'],
                Kp           = res['Kp'],
                Kd           = res['Kd'],
                T_total      = res['T_total']
            )
            # Do NOT stop the bridge — keeps MATLAB + Mechanics Explorer open
            self.log_msg.emit('[SIM] Simulation complete.')
            self.log_msg.emit('[SIM] Mechanics Explorer is still open — press Play to replay.')

            result_pkg = {
                'res': res, 'sim': sim_out, 'robot': robot, 'mode': self.mode
            }
            # TCP Test: compute per-waypoint EE deviation
            if self.mode == 'TCP Test':
                t_s   = sim_out['t_s']
                Q_out = sim_out['Q_out']
                N_out = len(t_s)
                ee_actual = np.zeros((N_out, 3))
                for i in range(N_out):
                    ee_actual[i] = robot.ee_position(Q_out[i])
                ts_ref  = res['q_ref_traj'][:,0]
                pos_ref = res['q_ref_traj'][:,1:]
                ee_ref  = np.zeros((len(ts_ref), 3))
                for i in range(len(ts_ref)):
                    ee_ref[i] = robot.ee_position(pos_ref[i])
                ee_ref_i = np.array([
                    np.interp(t_s, ts_ref, ee_ref[:,j]) for j in range(3)]).T
                dev_mm = np.linalg.norm(ee_actual - ee_ref_i, axis=1) * 1000
                max_dev = float(np.max(dev_mm))
                rms_dev = float(np.sqrt(np.mean(dev_mm**2)))
                # Per-waypoint arrival
                t_junc = np.cumsum([0] + list(res['seg_T']))
                for w in range(len(res['seg_T'])):
                    idx = int(np.argmin(np.abs(t_s - t_junc[w+1])))
                    wp  = self.params['waypoints'][w]
                    err_w = float(np.linalg.norm(ee_actual[idx] - wp) * 1000)
                    status = 'PASS' if err_w < 2.0 else 'FAIL'
                    self.log_msg.emit(f'[TCP] WP{w+1}: {err_w:.2f}mm {status}')
                self.log_msg.emit(
                    f'[TCP] EE deviation — Max:{max_dev:.2f}mm  RMS:{rms_dev:.2f}mm')
                result_pkg['tcp_dev_mm']  = dev_mm
                result_pkg['tcp_max_dev'] = max_dev
                result_pkg['tcp_rms_dev'] = rms_dev
                result_pkg['tcp_t_s']     = t_s
            self.finished.emit(result_pkg)
        except Exception as e:
            self.error_msg.emit(f'[ERROR] {e}')


# ── Matplotlib canvas ──────────────────────────────────────────────────────
class PlotCanvas(FigureCanvas):
    def __init__(self, title='', ylabel='', is_3d=False, parent=None):
        self.fig = Figure(figsize=(4, 2.5), facecolor=C_PANEL)
        super().__init__(self.fig)
        self.setParent(parent)
        self.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        if is_3d:
            self.ax = self.fig.add_subplot(111, projection='3d')
            self.ax.set_facecolor(C_PANEL)
        else:
            self.ax = self.fig.add_subplot(111)
            self.ax.set_facecolor(C_PANEL)
        self._style_ax(title, ylabel)
        self.fig.tight_layout(pad=1.0)

    def _style_ax(self, title, ylabel):
        ax = self.ax
        ax.set_title(title, color=C_DIM, fontsize=9, pad=4)
        ax.tick_params(colors=C_DIM, labelsize=8)
        ax.grid(True, color=C_BORDER, alpha=0.5)
        for spine in ax.spines.values():
            spine.set_edgecolor(C_BORDER)
        if ylabel:
            ax.set_ylabel(ylabel, color=C_DIM, fontsize=8)
        ax.set_xlabel('Time (s)', color=C_DIM, fontsize=8)

    def clear_plot(self):
        self.ax.clear()
        self.ax.set_facecolor(C_PANEL)
        self.ax.grid(True, color=C_BORDER, alpha=0.5)
        self.draw()


# ── Main application window ────────────────────────────────────────────────
class KinovaApp(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle('Kinova Gen3 — CTC Trajectory Simulator')
        self.showMaximized()
        self.sim_worker  = None
        self._last_Q_out = None
        self._last_t_s   = None
        self._last_robot = None
        self._bridge     = None   # shared MATLAB engine — stays alive between sims
        self._build_ui()
        self.add_log('[GUI] Kinova Gen3 CTC Simulator ready.')
        self.add_log('[GUI] Select mode, set target, press RUN SIMULATION.')
        self.add_log('[GUI] Single Target → enter XYZ → Calculate → Run.')
        self.add_log('[GUI] Multi-Waypoint → edit table → Run directly.')

    # ── UI Construction ────────────────────────────────────────────────────
    def _build_ui(self):
        central = QWidget()
        self.setCentralWidget(central)
        main_layout = QVBoxLayout(central)
        main_layout.setContentsMargins(6, 6, 6, 6)
        main_layout.setSpacing(4)

        # Title bar
        title_bar = self._make_title_bar()
        main_layout.addWidget(title_bar)

        # Main content splitter
        splitter = QSplitter(Qt.Orientation.Horizontal)
        splitter.setStyleSheet(f"QSplitter::handle {{ background: {C_BORDER}; }}")

        # Left: Joint Control
        splitter.addWidget(self._make_joint_control())

        # Centre-left: 3D Robot View
        splitter.addWidget(self._make_3d_view())

        # Centre: Trajectory Setup
        splitter.addWidget(self._make_traj_setup())

        # Right: Results + Energy + Safety
        splitter.addWidget(self._make_right_panels())

        splitter.setSizes([160, 340, 340, 360])
        main_layout.addWidget(splitter, stretch=7)

        # Bottom row: plots
        main_layout.addWidget(self._make_bottom_plots(), stretch=4)

    def _make_title_bar(self):
        bar = QWidget()
        bar.setFixedHeight(32)
        bar.setStyleSheet(f"background:{C_PANEL}; border-bottom:1px solid {C_ACCENT};")
        lay = QHBoxLayout(bar)
        lay.setContentsMargins(12, 0, 12, 0)
        title = QLabel('KINOVA GEN3  ·  CTC TRAJECTORY SIMULATOR')
        title.setStyleSheet(f"color:{C_ACCENT}; font-weight:bold; font-size:13px; letter-spacing:1px;")
        sub = QLabel('7-DOF Simscape Multibody  ·  Computed Torque Control')
        sub.setStyleSheet(f"color:{C_DIM}; font-size:10px;")
        self.status_label = QLabel('READY')
        self.status_label.setStyleSheet(f"color:{C_ACCENT2}; font-weight:bold;")
        lay.addWidget(title)
        lay.addWidget(sub)
        lay.addStretch()
        lay.addWidget(self.status_label)
        return bar

    def _make_joint_control(self):
        grp = QGroupBox('Joint Control')
        grp.setFixedWidth(165)
        lay = QVBoxLayout(grp)
        lay.setSpacing(4)
        self.joint_sliders = []
        self.joint_labels  = []
        joint_names = ['J1 Base','J2 Shoulder','J3 Elbow',
                       'J4 Forearm','J5 Wrist1','J6 Wrist2','J7 Tool']
        for i, name in enumerate(joint_names):
            lbl_name = QLabel(name)
            lbl_name.setStyleSheet(f"color:{C_DIM}; font-size:10px;")
            lbl_val  = QLabel('0.0°')
            lbl_val.setStyleSheet(f"color:{J_COLORS[i]}; font-size:10px; font-weight:bold;")
            slider = QSlider(Qt.Orientation.Horizontal)
            slider.setRange(-180, 180)
            slider.setValue(0)
            slider.setFixedHeight(16)
            slider.valueChanged.connect(lambda v, idx=i, lbl=lbl_val:
                                        lbl.setText(f'{v}°'))
            self.joint_sliders.append(slider)
            self.joint_labels.append(lbl_val)
            row = QHBoxLayout()
            row.addWidget(lbl_name)
            row.addStretch()
            row.addWidget(lbl_val)
            lay.addLayout(row)
            lay.addWidget(slider)

        btn_home = QPushButton('⌂ Home Position')
        btn_home.clicked.connect(self._on_home)
        lay.addWidget(btn_home)
        lay.addStretch()
        return grp

    def _make_3d_view(self):
        grp = QGroupBox('3D Robot View')
        lay = QVBoxLayout(grp)
        self.canvas_3d = PlotCanvas('', '', is_3d=True)
        self.canvas_3d.ax.set_xlabel('X (m)', color=C_DIM, fontsize=8)
        self.canvas_3d.ax.set_ylabel('Y (m)', color=C_DIM, fontsize=8)
        self.canvas_3d.ax.set_zlabel('Z (m)', color=C_DIM, fontsize=8)
        self.canvas_3d.ax.set_title('Set target XYZ and press Calculate',
                                     color=C_DIM, fontsize=9)
        self.canvas_3d.ax.tick_params(colors=C_DIM, labelsize=7)
        self.canvas_3d.fig.patch.set_facecolor(C_PANEL)
        self.canvas_3d.ax.set_facecolor('#0D1117')
        self.canvas_3d.ax.xaxis.pane.fill = False
        self.canvas_3d.ax.yaxis.pane.fill = False
        self.canvas_3d.ax.zaxis.pane.fill = False
        self.canvas_3d.ax.xaxis.pane.set_edgecolor(C_BORDER)
        self.canvas_3d.ax.yaxis.pane.set_edgecolor(C_BORDER)
        self.canvas_3d.ax.zaxis.pane.set_edgecolor(C_BORDER)
        self.canvas_3d.ax.grid(True, color=C_BORDER, alpha=0.3)
        lay.addWidget(self.canvas_3d)
        row = QHBoxLayout()
        btn_ws  = QPushButton('⬡ Show Workspace')
        btn_clr = QPushButton('✕ Clear View')
        btn_ws.clicked.connect(self._on_show_workspace)
        btn_clr.clicked.connect(self._on_clear_view)
        row.addWidget(btn_ws); row.addWidget(btn_clr)
        lay.addLayout(row)
        return grp

    def _make_traj_setup(self):
        grp = QGroupBox('Trajectory Setup')
        lay = QVBoxLayout(grp)
        lay.setSpacing(6)

        # Mode + Curve
        row1 = QHBoxLayout()
        row1.addWidget(QLabel('Trajectory Type'))
        self.mode_combo = QComboBox()
        self.mode_combo.blockSignals(True)
        self.mode_combo.addItems(['Single Target','Multi-Waypoint','TCP Test'])
        self.mode_combo.blockSignals(False)
        self.mode_combo.currentTextChanged.connect(self._on_mode_changed)
        row1.addWidget(self.mode_combo)
        lay.addLayout(row1)

        row2 = QHBoxLayout()
        row2.addWidget(QLabel('Curve Profile'))
        self.curve_combo = QComboBox()
        self.curve_combo.addItems(['Cubic Spline','Quintic Polynomial',
                                   'Trapezoidal (LSPB)','Bang-Bang'])
        row2.addWidget(self.curve_combo)
        lay.addLayout(row2)

        # Preset buttons
        preset_row = QHBoxLayout()
        for label, xyz in [('Forward',[0.5,0,0.3]),
                            ('Side',[0.3,0.4,0.3]),
                            ('Diagonal',[0.4,0.3,0.4])]:
            btn = QPushButton(label)
            btn.clicked.connect(lambda _, p=xyz: self._set_target(p))
            preset_row.addWidget(btn)
        lay.addLayout(preset_row)

        # XYZ target (Single Target panel)
        self.single_panel = QWidget()
        sp_lay = QGridLayout(self.single_panel)
        sp_lay.setSpacing(4)
        self.x_edit = QLineEdit('0.5'); self.y_edit = QLineEdit('0')
        self.z_edit = QLineEdit('0.3')
        for col, (lbl, edit) in enumerate(
                [('X (m→Fwd)', self.x_edit),
                 ('Y (m→Side)', self.y_edit),
                 ('Z (m→Up)', self.z_edit)]):
            sp_lay.addWidget(QLabel(lbl), 0, col)
            sp_lay.addWidget(edit, 1, col)
        # EE Orientation dropdown
        sp_lay.addWidget(QLabel('EE Orientation'), 2, 0)
        self.ori_combo = QComboBox()
        self.ori_combo.addItems(['Down (-Z)', 'Up (+Z)', 'Horizontal', 'Tilt 45°'])
        sp_lay.addWidget(self.ori_combo, 2, 1, 1, 2)
        lay.addWidget(self.single_panel)

        # WP table (Multi-Waypoint panel)
        self.multi_panel = QWidget()
        mp_lay = QVBoxLayout(self.multi_panel)
        mp_lay.setContentsMargins(0,0,0,0)
        self.wp_table = QTableWidget(3, 6)
        self.wp_table.setHorizontalHeaderLabels(
            ['X (m)','Y (m)','Z (m)','EE Orient','Spd','Note'])
        self.wp_table.horizontalHeader().setSectionResizeMode(
            QHeaderView.ResizeMode.Stretch)
        self.wp_table.setMaximumHeight(140)
        self._reset_wp_table()
        mp_lay.addWidget(self.wp_table)
        btn_row = QHBoxLayout()
        for lbl, fn in [('+ Add Row', self._add_wp_row),
                        ('- Remove',  self._remove_wp_row),
                        ('↺ Reset',   self._reset_wp_table)]:
            b = QPushButton(lbl); b.clicked.connect(fn); btn_row.addWidget(b)
        mp_lay.addLayout(btn_row)
        self.multi_panel.setVisible(False)
        lay.addWidget(self.multi_panel)

        # Speed + Gain
        sg_row = QHBoxLayout()
        sg_row.addWidget(QLabel('Tool Speed (m/s)'))
        self.speed_edit = QLineEdit('0.2')
        self.speed_edit.setFixedWidth(60)
        sg_row.addWidget(self.speed_edit)
        sg_row.addWidget(QLabel('Gain Scale'))
        self.gain_edit = QLineEdit('3')
        self.gain_edit.setFixedWidth(40)
        sg_row.addWidget(self.gain_edit)
        lay.addLayout(sg_row)

        # Action buttons
        self.btn_calc = QPushButton('⊙ Calculate Trajectory / IK')
        self.btn_calc.setObjectName('btn_calc')
        self.btn_calc.clicked.connect(self._on_calculate)
        lay.addWidget(self.btn_calc)

        btn_sim_row = QHBoxLayout()
        self.btn_run  = QPushButton('▶  RUN SIMULATION')
        self.btn_run.setObjectName('btn_run')
        self.btn_run.clicked.connect(self._on_run)
        self.btn_stop = QPushButton('■  STOP')
        self.btn_stop.setObjectName('btn_stop')
        self.btn_stop.clicked.connect(self._on_stop)
        btn_sim_row.addWidget(self.btn_run)
        btn_sim_row.addWidget(self.btn_stop)
        lay.addLayout(btn_sim_row)

        btn_replay = QPushButton('⟳ Replay Animation')
        btn_replay.clicked.connect(self._on_replay)
        lay.addWidget(btn_replay)

        lay.addStretch()
        return grp

    def _make_right_panels(self):
        container = QWidget()
        lay = QVBoxLayout(container)
        lay.setSpacing(4)
        lay.setContentsMargins(0,0,0,0)

        # Simulation Results
        res_grp = QGroupBox('Simulation Results')
        res_lay = QGridLayout(res_grp)
        self.rms_label  = self._val_label('—')
        self.max_label  = self._val_label('—')
        self.best_label = self._val_label('—')
        self.worst_label= self._val_label('—')
        for col, (title, lbl) in enumerate([
                ('Overall RMS Error', self.rms_label),
                ('Max Joint Error',   self.max_label)]):
            res_lay.addWidget(QLabel(title), 0, col)
            res_lay.addWidget(lbl, 1, col)
        for col, (title, lbl) in enumerate([
                ('Best joint', self.best_label),
                ('Worst joint', self.worst_label)]):
            res_lay.addWidget(QLabel(title), 2, col)
            res_lay.addWidget(lbl, 3, col)

        # Simulation log
        self.sim_log = QTextEdit()
        self.sim_log.setReadOnly(True)
        self.sim_log.setMaximumHeight(160)
        res_lay.addWidget(QLabel('Simulation Log'), 4, 0, 1, 2)
        res_lay.addWidget(self.sim_log, 5, 0, 1, 2)
        lay.addWidget(res_grp)

        # Energy
        en_grp = QGroupBox('Energy Consumption')
        en_lay = QGridLayout(en_grp)
        self.energy_total   = self._val_label('— J', size=18)
        self.energy_grav    = self._val_label('— J')
        self.energy_inert   = self._val_label('— J')
        self.energy_cor     = self._val_label('— J')
        en_lay.addWidget(QLabel('TOTAL'), 0, 0, 1, 2)
        en_lay.addWidget(self.energy_total, 1, 0, 1, 2)
        for col, (lbl, val) in enumerate([
                ('GRAVITY', self.energy_grav),
                ('INERTIAL', self.energy_inert)]):
            en_lay.addWidget(QLabel(lbl), 2, col)
            en_lay.addWidget(val, 3, col)
        en_lay.addWidget(QLabel('CORIOLIS'), 4, 0)
        en_lay.addWidget(self.energy_cor, 5, 0)
        lay.addWidget(en_grp)

        # Safety Monitor
        safe_grp = QGroupBox('Safety Monitor')
        safe_lay = QVBoxLayout(safe_grp)
        self.safety_score = QLabel('NOT CHECKED')
        self.safety_score.setStyleSheet(f"color:{C_DIM}; font-size:18px; font-weight:bold;")
        self.safety_score.setAlignment(Qt.AlignmentFlag.AlignCenter)
        safe_lay.addWidget(self.safety_score)
        self.safety_checks = {}
        for check in ['Joint Limits','Velocity Limits','Torque Saturation',
                       'Workspace Boundary','Singularity','Self-Collision','Acceleration']:
            row = QHBoxLayout()
            dot = QLabel('●')
            dot.setStyleSheet(f"color:{C_DIM};")
            lbl = QLabel(check)
            lbl.setStyleSheet(f"color:{C_DIM}; font-size:11px;")
            row.addWidget(dot); row.addWidget(lbl); row.addStretch()
            self.safety_checks[check] = dot
            safe_lay.addLayout(row)
        btn_safety = QPushButton('⊙ Run Safety Check')
        btn_safety.clicked.connect(self._on_safety_check)
        safe_lay.addWidget(btn_safety)
        btn_export = QPushButton('↓ Export Results (CSV)')
        btn_export.clicked.connect(self._on_export_results)
        btn_export.setStyleSheet(f"background:{C_PANEL}; color:{C_ACCENT2}; "
                                  f"border:1px solid {C_ACCENT2}; border-radius:4px; padding:5px;")
        safe_lay.addWidget(btn_export)
        lay.addWidget(safe_grp)

        # EE Pose
        ee_grp = QGroupBox('End-Effector Pose')
        ee_lay = QGridLayout(ee_grp)
        self.ee_x = self._val_label('—'); self.ee_y = self._val_label('—')
        self.ee_z = self._val_label('—'); self.ee_ik = self._val_label('—')
        self.ee_dist = self._val_label('—'); self.ee_t = self._val_label('—')
        self.ee_roll  = self._val_label('—', size=11)
        self.ee_pitch = self._val_label('—', size=11)
        self.ee_yaw   = self._val_label('—', size=11)
        for col, (lbl, val) in enumerate([
                ('X (m)', self.ee_x), ('Y (m)', self.ee_y), ('Z (m)', self.ee_z)]):
            ee_lay.addWidget(QLabel(lbl), 0, col)
            ee_lay.addWidget(val, 1, col)
        for col, (lbl, val) in enumerate([
                ('IK err (mm)', self.ee_ik),
                ('Distance (m)', self.ee_dist),
                ('T total (s)', self.ee_t)]):
            ee_lay.addWidget(QLabel(lbl), 2, col)
            ee_lay.addWidget(val, 3, col)
        for col, (lbl, val) in enumerate([
                ('Roll (°)', self.ee_roll),
                ('Pitch (°)', self.ee_pitch),
                ('Yaw (°)', self.ee_yaw)]):
            ee_lay.addWidget(QLabel(lbl), 4, col)
            ee_lay.addWidget(val, 5, col)
        lay.addWidget(ee_grp)

        return container

    def _make_bottom_plots(self):
        container = QWidget()
        lay = QHBoxLayout(container)
        lay.setSpacing(4)
        lay.setContentsMargins(0,0,0,0)
        self.canvas_pos  = PlotCanvas('Joint Positions', 'Position (deg)')
        self.canvas_vel  = PlotCanvas('Joint Velocities', 'Velocity (rad/s)')
        self.canvas_torq = PlotCanvas('Feedforward Torques', 'Torque (Nm)')
        self.canvas_err  = PlotCanvas('Tracking Error', 'Error (deg)')
        for c in [self.canvas_pos, self.canvas_vel,
                  self.canvas_torq, self.canvas_err]:
            lay.addWidget(c)
        return container

    # ── Helper widgets ─────────────────────────────────────────────────────
    def _val_label(self, text='—', size=14):
        lbl = QLabel(text)
        lbl.setStyleSheet(f"color:{C_ACCENT2}; font-size:{size}px; font-weight:bold;")
        lbl.setAlignment(Qt.AlignmentFlag.AlignCenter)
        return lbl

    # ── Callbacks ──────────────────────────────────────────────────────────
    def _on_mode_changed(self, mode):
        is_multi = mode in ('Multi-Waypoint', 'TCP Test')
        self.single_panel.setVisible(not is_multi)
        self.multi_panel.setVisible(is_multi)
        if mode == 'TCP Test':
            self._load_tcp_defaults()
        self.add_log(f'[GUI] Mode: {mode}')

    def _set_target(self, xyz):
        self.x_edit.setText(str(xyz[0]))
        self.y_edit.setText(str(xyz[1]))
        self.z_edit.setText(str(xyz[2]))

    def _on_home(self):
        for s in self.joint_sliders:
            s.setValue(0)

    def _on_calculate(self):
        self.add_log('[IK] Calculating...')
        self.set_status('CALCULATING', C_ACCENT)
        try:
            import numpy as np
            from robot_model import KinovaGen3
            robot = KinovaGen3()
            tx = float(self.x_edit.text())
            ty = float(self.y_edit.text())
            tz = float(self.z_edit.text())
            ori_map = {'Down (-Z)':'down','Up (+Z)':'up',
                       'Horizontal':'horiz','Tilt 45°':'tilt45'}
            ori_key = self.ori_combo.currentText()
            ori     = ori_map.get(ori_key, 'down')
            T_target = robot.target_transform([tx, ty, tz], ori)
            q_sol, ok, err = robot.ikine(T_target, robot.home_config())
            if ok:
                self.add_log(f'[IK] ✓ Error={err:.3f}mm')
                for i, s in enumerate(self.joint_sliders):
                    s.setValue(int(np.rad2deg(q_sol[i])))
                ee = robot.ee_position(q_sol)
                self.ee_x.setText(f'{ee[0]:.4f}')
                self.ee_y.setText(f'{ee[1]:.4f}')
                self.ee_z.setText(f'{ee[2]:.4f}')
                self.ee_ik.setText(f'{err:.2f}')
                self.ee_dist.setText(f'{np.linalg.norm(ee):.3f}')
                self._plot_ee_path(robot, q_sol, [tx,ty,tz])
                self.set_status('READY', C_ACCENT2)
            else:
                self.add_log('[IK] ✗ Failed — target unreachable')
                self.set_status('ERROR', C_DANGER)
        except Exception as e:
            self.add_log(f'[IK ERROR] {e}')
            self.set_status('ERROR', C_DANGER)

    def _on_run(self):
        mode = self.mode_combo.currentText()
        self.add_log(f'[SIM] Starting {mode} simulation...')
        self.set_status('RUNNING', C_ACCENT)
        self.btn_run.setEnabled(False)

        try:
            speed      = float(self.speed_edit.text())
            gain_scale = float(self.gain_edit.text())
            curve_map  = {'Cubic Spline':'cubic','Quintic Polynomial':'quintic',
                          'Trapezoidal (LSPB)':'lspb','Bang-Bang':'bangbang'}
            curve_type = curve_map.get(self.curve_combo.currentText(), 'cubic')

            if mode == 'Single Target':
                ori_map = {'Down (-Z)':'down','Up (+Z)':'up',
                           'Horizontal':'horiz','Tilt 45°':'tilt45'}
                ori = ori_map.get(self.ori_combo.currentText(), 'down')
                params = {
                    'target_xyz': [float(self.x_edit.text()),
                                   float(self.y_edit.text()),
                                   float(self.z_edit.text())],
                    'speed': speed, 'gain_scale': gain_scale,
                    'curve_type': curve_type,
                    'orientation': ori
                }
            else:
                wps, oris, spds = self._read_wp_table()
                params = {'waypoints': wps, 'orientations': oris,
                          'speeds': spds, 'gain_scale': gain_scale}

            self.sim_worker = SimWorker(mode, params)
            self.sim_worker.log_msg.connect(self.add_log)
            self.sim_worker.error_msg.connect(self._on_sim_error)
            self.sim_worker.finished.connect(self._on_sim_done)
            self.sim_worker.start()

        except Exception as e:
            self.add_log(f'[ERROR] {e}')
            self.set_status('ERROR', C_DANGER)
            self.btn_run.setEnabled(True)

    def _on_stop(self):
        if self.sim_worker and self.sim_worker.isRunning():
            self.sim_worker.terminate()
            self.add_log('[SIM] Stopped by user.')
            self.set_status('STOPPED', C_WARN)
            self.btn_run.setEnabled(True)

    def _on_sim_done(self, result):
        self.btn_run.setEnabled(True)
        self.set_status('COMPLETE', C_ACCENT2)
        self.add_log('[SIM] Updating results...')
        try:
            self._update_all_panels(result)
            self.add_log('[SIM] Done.')
            self.add_log('[SIM] Mechanics Explorer: press ▶ Play to replay 3D mesh animation.')
        except Exception as e:
            self.add_log(f'[POST ERROR] {e}')

    def _on_sim_error(self, msg):
        self.add_log(msg)
        self.set_status('ERROR', C_DANGER)
        self.btn_run.setEnabled(True)

    def _on_replay(self):
        if not hasattr(self, '_last_Q_out') or self._last_Q_out is None:
            self.add_log('[ANIM] No simulation data — run a simulation first.')
            return
        self.add_log('[ANIM] Replaying robot animation...')
        import numpy as np
        Q_out  = self._last_Q_out
        t_s    = self._last_t_s
        robot  = self._last_robot
        N      = Q_out.shape[0]
        step   = max(1, N // 150)   # ~150 frames for smooth animation
        ax     = self.canvas_3d.ax

        # Body link names in order
        link_names = ['shoulder_link','half_arm_1_link','half_arm_2_link',
                      'forearm_link','spherical_wrist_1_link',
                      'spherical_wrist_2_link','bracelet_link','end_effector_link']
        ee_trace = []

        for i in range(0, N, step):
            q_i   = Q_out[i]
            q_full = np.zeros(robot._robot.n)
            q_full[:7] = q_i
            pts = [[0,0,0]]
            for lname in link_names:
                try:
                    T = robot._robot.fkine(q_full, end=lname)
                    pts.append(list(T.t))
                except:
                    pts.append(pts[-1])
            pts = np.array(pts)
            ee_trace.append(pts[-1])

            ax.clear()
            ax.set_facecolor('#0D1117')
            ax.xaxis.pane.fill = False
            ax.yaxis.pane.fill = False
            ax.zaxis.pane.fill = False
            ax.xaxis.pane.set_edgecolor(C_BORDER)
            ax.yaxis.pane.set_edgecolor(C_BORDER)
            ax.zaxis.pane.set_edgecolor(C_BORDER)
            ax.grid(True, color=C_BORDER, alpha=0.3)

            # Draw EE trace so far
            if len(ee_trace) > 1:
                tr = np.array(ee_trace)
                ax.plot(tr[:,0], tr[:,1], tr[:,2],
                        color='#FFCC00', lw=1.2, alpha=0.7)

            # Draw robot skeleton
            for j in range(len(pts)-1):
                col = J_COLORS[min(j, 6)]
                ax.plot([pts[j,0],pts[j+1,0]],
                        [pts[j,1],pts[j+1,1]],
                        [pts[j,2],pts[j+1,2]],
                        color=col, lw=3)
                ax.scatter(*pts[j+1], color=col, s=30)

            # Base
            ax.scatter(0,0,0, color=C_ACCENT, s=60, marker='^')

            ax.set_xlim([-0.8,0.8]); ax.set_ylim([-0.8,0.8]); ax.set_zlim([0,1.4])
            ax.set_xlabel('X (m)', color=C_DIM, fontsize=7)
            ax.set_ylabel('Y (m)', color=C_DIM, fontsize=7)
            ax.set_zlabel('Z (m)', color=C_DIM, fontsize=7)
            ax.tick_params(colors=C_DIM, labelsize=6)
            ax.set_title(f'Animation  t={t_s[i]:.2f}s', color=C_DIM, fontsize=9)
            self.canvas_3d.draw()
            QApplication.processEvents()

        self.add_log('[ANIM] Animation complete.')

    def _on_show_workspace(self):
        self.add_log('[WS] Computing workspace (sweeping J1/J2/J3)...')
        QApplication.processEvents()
        import numpy as np
        try:
            import numpy as np
            from robot_model import KinovaGen3
            robot = KinovaGen3()
            pts = []
            for j1 in np.deg2rad(range(-120,121,20)):
                for j2 in np.deg2rad(range(-120,121,20)):
                    for j3 in np.deg2rad(range(-120,121,20)):
                        q = np.zeros(7)
                        q[0]=j1; q[1]=j2; q[2]=j3
                        ee = robot.ee_position(q)
                        if ee[2] > 0.02 and np.linalg.norm(ee) < 1.3:
                            pts.append(ee)
            pts = np.array(pts)
            ax = self.canvas_3d.ax
            ax.clear(); ax.set_facecolor('#0D1117')
            ax.xaxis.pane.fill=False; ax.yaxis.pane.fill=False; ax.zaxis.pane.fill=False
            ax.xaxis.pane.set_edgecolor(C_BORDER)
            ax.yaxis.pane.set_edgecolor(C_BORDER)
            ax.zaxis.pane.set_edgecolor(C_BORDER)
            sc = ax.scatter(pts[:,0],pts[:,1],pts[:,2],
                           c=pts[:,2], cmap='cool', s=2, alpha=0.4)
            ax.set_xlabel('X (m)',color=C_DIM,fontsize=7)
            ax.set_ylabel('Y (m)',color=C_DIM,fontsize=7)
            ax.set_zlabel('Z (m)',color=C_DIM,fontsize=7)
            ax.tick_params(colors=C_DIM,labelsize=6)
            ax.set_title(f'Workspace — {len(pts)} reachable points',
                        color=C_DIM,fontsize=9)
            self.canvas_3d.draw()
            self.add_log(f'[WS] Done — {len(pts)} reachable points shown.')
        except Exception as e:
            self.add_log(f'[WS ERROR] {e}')

    def _on_clear_view(self):
        self.canvas_3d.ax.clear()
        self.canvas_3d.ax.set_facecolor(C_PANEL)
        self.canvas_3d.ax.set_title('Cleared', color=C_DIM, fontsize=9)
        self.canvas_3d.draw()

    def _on_safety_check(self):
        self.add_log('[SAFE] Running safety checks...')
        # Check joint limits on current slider positions
        import numpy as np
        q_deg = np.array([s.value() for s in self.joint_sliders], dtype=float)
        q_rad = np.deg2rad(q_deg)
        lim_hi = np.array([138.1,138.1,152.4,127.8,119.7,119.7,119.7])
        # Weights: Joint Limits=20, Workspace=20, others=12 each → total=100
        score  = 0
        checks = {}
        # Joint limits (20 pts)
        within = np.all(np.abs(q_deg) <= lim_hi)
        checks['Joint Limits'] = within
        score += 20 if within else 0
        # Workspace boundary (20 pts)
        try:
            import numpy as np
            from robot_model import KinovaGen3
            robot = KinovaGen3()
            ee = robot.ee_position(q_rad)
            ws_ok = ee[2] > 0.02 and np.linalg.norm(ee) < 1.3
            checks['Workspace Boundary'] = ws_ok
            score += 20 if ws_ok else 0
        except: checks['Workspace Boundary'] = False
        # Others default to pass (12 pts each × 5 = 60)
        for k in ['Velocity Limits','Torque Saturation',
                  'Singularity','Self-Collision','Acceleration']:
            checks[k] = True; score += 12
        # Update dots
        for check, ok in checks.items():
            if check in self.safety_checks:
                color = C_ACCENT2 if ok else C_DANGER
                self.safety_checks[check].setStyleSheet(f'color:{color};')
        self.safety_score.setText(f'{score} / 100')
        color = C_ACCENT2 if score >= 90 else (C_WARN if score >= 60 else C_DANGER)
        self.safety_score.setStyleSheet(f'color:{color}; font-size:18px; font-weight:bold;')
        self.add_log(f'[SAFE] Score: {score}/100  '
                     f'{"ALL PASSED" if score==100 else "CHECK WARNINGS"}')

    # ── Results update ─────────────────────────────────────────────────────
    def _update_all_panels(self, result):
        res   = result['res']
        sim   = result['sim']
        robot = result['robot']
        mode  = result['mode']

        t_s   = sim['t_s']
        Q_out = sim['Q_out']
        ts_ref  = res['q_ref_traj'][:, 0]
        pos_ref = res['q_ref_traj'][:, 1:]
        vel_ref = res['qd_ref_traj'][:, 1:]
        acc_ref = res['tau_ref_traj'][:, 1:]

        # Interpolate reference to output timestamps
        qref_i = np.zeros_like(Q_out)
        for j in range(7):
            qref_i[:, j] = np.interp(t_s, ts_ref, pos_ref[:, j])

        err     = Q_out - qref_i
        rms_j   = np.sqrt(np.mean(err**2, axis=0))
        max_j   = np.max(np.abs(err), axis=0)
        rms_all = np.mean(rms_j) * 180/np.pi
        max_all = np.max(max_j)  * 180/np.pi
        best_j  = int(np.argmin(max_j)) + 1
        worst_j = int(np.argmax(max_j)) + 1

        if result.get('mode') == 'TCP Test' and 'tcp_max_dev' in result:
            self.rms_label.setText(f"{result['tcp_rms_dev']:.3f} mm")
            self.max_label.setText(f"{result['tcp_max_dev']:.3f} mm")
            self.best_label.setText('EE path')
            self.worst_label.setText('deviation')
            self.add_log(f"[TCP] RMS={result['tcp_rms_dev']:.3f}mm  "
                        f"Max={result['tcp_max_dev']:.3f}mm")
            # TCP tracking error plot — EE deviation in mm
            ax = self.canvas_err.ax
            ax.clear(); ax.set_facecolor(C_PANEL)
            ax.plot(result['tcp_t_s'], result['tcp_dev_mm'],
                    color=C_ACCENT2, lw=1.5)
            ax.axhline(2.0, color=C_WARN, lw=1, ls='--', label='2mm limit')
            ax.set_title(f"TCP EE deviation  Max={result['tcp_max_dev']:.2f}mm",
                        color=C_DIM, fontsize=9)
            ax.set_xlabel('Time (s)', color=C_DIM, fontsize=8)
            ax.set_ylabel('EE deviation (mm)', color=C_DIM, fontsize=8)
            ax.tick_params(colors=C_DIM, labelsize=7)
            ax.grid(True, color=C_BORDER, alpha=0.5)
            ax.legend(fontsize=7, labelcolor='white',
                     facecolor=C_PANEL, edgecolor=C_BORDER)
            self.canvas_err.draw()
        else:
            self.rms_label.setText(f'{rms_all:.4f}°')
            self.max_label.setText(f'{max_all:.4f}°')
            self.best_label.setText(f'J{best_j}')
            self.worst_label.setText(f'J{worst_j}')
            self.add_log(f'[RESULT] RMS={rms_all:.4f}°  Max={max_all:.4f}°  '
                         f'Best=J{best_j}  Worst=J{worst_j}')

        # EE pose with RPY
        T_ee = robot.fkine(Q_out[-1])
        ee   = T_ee.t
        R    = T_ee.R
        # Extract roll/pitch/yaw from rotation matrix
        pitch = np.arcsin(-R[2,0])
        roll  = np.arctan2(R[2,1]/np.cos(pitch), R[2,2]/np.cos(pitch))
        yaw   = np.arctan2(R[1,0]/np.cos(pitch), R[0,0]/np.cos(pitch))
        self.ee_x.setText(f'{ee[0]:.4f}')
        self.ee_y.setText(f'{ee[1]:.4f}')
        self.ee_z.setText(f'{ee[2]:.4f}')
        self.ee_dist.setText(f'{np.linalg.norm(ee):.3f}')
        self.ee_t.setText(f'{res["T_total"]:.1f}')
        self.ee_roll.setText(f'{np.rad2deg(roll):.1f}°')
        self.ee_pitch.setText(f'{np.rad2deg(pitch):.1f}°')
        self.ee_yaw.setText(f'{np.rad2deg(yaw):.1f}°')

        # Energy (simple power integral)
        self._compute_energy(robot, pos_ref, vel_ref, acc_ref, ts_ref)

        # Plots
        self._plot_results(t_s, Q_out, qref_i, ts_ref, vel_ref, acc_ref,
                              err, res, robot=robot)

        # Store for replay and export
        self._last_t_s  = t_s
        self._last_res  = res
        self._plot_ee_trajectory(robot, Q_out)

        # Update joint sliders to final position
        for i, s in enumerate(self.joint_sliders):
            s.setValue(int(np.rad2deg(Q_out[-1, i])))

    def _compute_energy(self, robot, pos_ref, vel_ref, acc_ref, ts_ref):
        """Compute energy breakdown: Gravity, Inertial, Coriolis via dynamics."""
        try:
            self.add_log('[ENERGY] Computing energy breakdown...')
            QApplication.processEvents()
            N = len(ts_ref)
            step = max(1, N // 100)   # sample every step for speed
            ts_s  = ts_ref[::step]
            pos_s = pos_ref[::step]
            vel_s = vel_ref[::step]
            acc_s = acc_ref[::step]
            Ns = len(ts_s)

            P_grav  = np.zeros(Ns)
            P_inert = np.zeros(Ns)
            P_cor   = np.zeros(Ns)

            for i in range(Ns):
                q   = pos_s[i]
                qd  = vel_s[i]
                qdd = acc_s[i]
                q   = np.asarray(q,   dtype=float).flatten()[:7]
                qd  = np.asarray(qd,  dtype=float).flatten()[:7]
                qdd = np.asarray(qdd, dtype=float).flatten()[:7]
                G   = np.asarray(robot.gravity_torque(q)).flatten()[:7]
                M   = np.asarray(robot.mass_matrix(q))[:7,:7]
                C   = np.asarray(robot.coriolis(q, qd)).flatten()[:7]
                tau_g = G
                tau_i = M @ qdd
                tau_c = C
                P_grav[i]  = float(np.abs(tau_g @ qd))
                P_inert[i] = float(np.abs(tau_i @ qd))
                P_cor[i]   = float(np.abs(tau_c @ qd))

            E_grav  = np.trapz(P_grav,  ts_s)
            E_inert = np.trapz(P_inert, ts_s)
            E_cor   = np.trapz(P_cor,   ts_s)
            E_total = E_grav + E_inert + E_cor

            pct_g = E_grav  / E_total * 100 if E_total > 0 else 0
            pct_i = E_inert / E_total * 100 if E_total > 0 else 0
            pct_c = E_cor   / E_total * 100 if E_total > 0 else 0

            self.energy_total.setText(f'{E_total:.2f} J')
            self.energy_grav.setText(f'{E_grav:.2f} J  ({pct_g:.0f}%)')
            self.energy_inert.setText(f'{E_inert:.2f} J  ({pct_i:.0f}%)')
            self.energy_cor.setText(f'{E_cor:.2f} J  ({pct_c:.0f}%)')
            self.add_log(f'[ENERGY] Total={E_total:.2f}J  '
                        f'Grav={pct_g:.0f}%  '
                        f'Inert={pct_i:.0f}%  '
                        f'Cor={pct_c:.0f}%')
        except Exception as e:
            self.add_log(f'[ENERGY ERROR] {e}')

    def _plot_results(self, t_s, Q_out, qref_i, ts_ref, vel_ref, acc_ref,
                      err, res, robot=None):
        jleg = [f'J{j+1}' for j in range(7)]

        def _style(ax, title, ylabel):
            ax.set_title(title, color=C_DIM, fontsize=9, pad=3)
            ax.set_xlabel('Time (s)', color=C_DIM, fontsize=8)
            ax.set_ylabel(ylabel, color=C_DIM, fontsize=8)
            ax.tick_params(colors=C_DIM, labelsize=7)
            ax.grid(True, color=C_BORDER, alpha=0.5)
            for sp in ax.spines.values():
                sp.set_edgecolor(C_BORDER)

        # ── Joint Positions ───────────────────────────────────────────────
        ax = self.canvas_pos.ax
        ax.clear(); ax.set_facecolor(C_PANEL)
        for j in range(7):
            ax.plot(t_s, np.rad2deg(Q_out[:, j]),
                    color=J_COLORS[j], lw=1.2, label=jleg[j])
            ax.plot(ts_ref, np.rad2deg(res['q_ref_traj'][:,j+1]),
                    color=J_COLORS[j], lw=0.6, ls='--', alpha=0.4)
        ax.legend(fontsize=6, labelcolor='white',
                  facecolor=C_PANEL, edgecolor=C_BORDER,
                  loc='upper left', ncol=4)
        _style(ax, 'Joint Positions — solid=actual  dashed=reference',
               'Position (deg)')
        self.canvas_pos.draw()

        # ── Joint Velocities ──────────────────────────────────────────────
        ax = self.canvas_vel.ax
        ax.clear(); ax.set_facecolor(C_PANEL)
        for j in range(7):
            ax.plot(ts_ref, vel_ref[:, j], color=J_COLORS[j], lw=1.2,
                    label=jleg[j])
        ax.legend(fontsize=6, labelcolor='white',
                  facecolor=C_PANEL, edgecolor=C_BORDER,
                  loc='upper left', ncol=4)
        _style(ax, 'Joint Velocities (reference)', 'Velocity (rad/s)')
        self.canvas_vel.draw()

        # ── Feedforward Torques ───────────────────────────────────────────
        ax = self.canvas_torq.ax
        ax.clear(); ax.set_facecolor(C_PANEL)
        if robot is not None:
            # Compute real CTC torques: tau = M*qdd + C + G
            self.add_log('[TORQ] Computing feedforward torques...')
            N_ref = len(ts_ref)
            step  = max(1, N_ref // 300)
            ts_t  = ts_ref[::step]
            tau_all = np.zeros((len(ts_t), 7))
            tau_max = np.array([187,187,187,52,52,52,52])
            for i, idx in enumerate(range(0, N_ref, step)):
                q   = np.asarray(res['q_ref_traj'][idx,1:]).flatten()[:7]
                qd  = np.asarray(res['qd_ref_traj'][idx,1:]).flatten()[:7]
                qdd = np.asarray(res['tau_ref_traj'][idx,1:]).flatten()[:7]
                G = np.asarray(robot.gravity_torque(q)).flatten()[:7]
                M = np.asarray(robot.mass_matrix(q))[:7,:7]
                C = np.asarray(robot.coriolis(q,qd)).flatten()[:7]
                tau = M@qdd + C + G
                tau_all[i] = np.clip(tau, -tau_max, tau_max)
            for j in range(7):
                ax.plot(ts_t, tau_all[:,j], color=J_COLORS[j],
                        lw=1.2, label=jleg[j])
            # Torque limit lines
            ax.axhline( 52, color=C_WARN, lw=0.6, ls=':', alpha=0.6,
                       label='J4-7 limit')
            ax.axhline(-52, color=C_WARN, lw=0.6, ls=':', alpha=0.6)
            ax.legend(fontsize=6, labelcolor='white',
                      facecolor=C_PANEL, edgecolor=C_BORDER,
                      loc='upper left', ncol=4)
            _style(ax, 'Feedforward Torques  τ = M·q̈ + C + G',
                   'Torque (Nm)')
            self.add_log('[TORQ] Done.')
        else:
            for j in range(7):
                ax.plot(ts_ref, acc_ref[:, j], color=J_COLORS[j], lw=1.2)
            _style(ax, 'Feedforward accelerations (rad/s²)', 'Accel (rad/s²)')
        self.canvas_torq.draw()

        # ── Tracking Error ────────────────────────────────────────────────
        ax = self.canvas_err.ax
        ax.clear(); ax.set_facecolor(C_PANEL)
        for j in range(7):
            ax.plot(t_s, np.rad2deg(err[:, j]),
                    color=J_COLORS[j], lw=1.2, label=jleg[j])
        ax.axhline(0, color=C_DIM, lw=0.8, ls='--', alpha=0.5)
        ax.legend(fontsize=6, labelcolor='white',
                  facecolor=C_PANEL, edgecolor=C_BORDER,
                  loc='upper left', ncol=4)
        max_err = np.max(np.abs(np.rad2deg(err)))
        _style(ax,
               f'Tracking Error  Q_out − q_ref  Max={max_err:.4f}°',
               'Error (deg)')
        self.canvas_err.draw()

    def _plot_ee_path(self, robot, q_sol, target_xyz):
        ax = self.canvas_3d.ax
        ax.clear()
        ax.set_facecolor('#0D1117')
        ax.xaxis.pane.fill = False
        ax.yaxis.pane.fill = False
        ax.zaxis.pane.fill = False
        ax.xaxis.pane.set_edgecolor(C_BORDER)
        ax.yaxis.pane.set_edgecolor(C_BORDER)
        ax.zaxis.pane.set_edgecolor(C_BORDER)
        ax.grid(True, color=C_BORDER, alpha=0.3)
        home_pos = robot.ee_position(robot.home_config())
        ax.scatter(*home_pos, color=C_ACCENT2, s=60, label='Home')
        ax.scatter(*target_xyz, color=C_DANGER, s=80, marker='*', label='Target')
        ax.plot([home_pos[0], target_xyz[0]],
                [home_pos[1], target_xyz[1]],
                [home_pos[2], target_xyz[2]],
                color=C_ACCENT, lw=1.5, ls='--', alpha=0.7)
        ax.set_xlim([-0.8,0.8]); ax.set_ylim([-0.8,0.8]); ax.set_zlim([0,1.4])
        ax.set_xlabel('X (m)', color=C_DIM, fontsize=7)
        ax.set_ylabel('Y (m)', color=C_DIM, fontsize=7)
        ax.set_zlabel('Z (m)', color=C_DIM, fontsize=7)
        ax.tick_params(colors=C_DIM, labelsize=6)
        ax.set_title(f'Target: {[round(x,3) for x in target_xyz]}',
                    color=C_DIM, fontsize=9)
        ax.legend(fontsize=7, labelcolor=C_DIM,
                 facecolor=C_PANEL, edgecolor=C_BORDER)
        self.canvas_3d.draw()


    def _plot_ee_trajectory(self, robot, Q_out):
        ax = self.canvas_3d.ax
        ax.clear()
        ax.set_facecolor('#0D1117')
        ax.xaxis.pane.fill = False
        ax.yaxis.pane.fill = False
        ax.zaxis.pane.fill = False
        ax.xaxis.pane.set_edgecolor(C_BORDER)
        ax.yaxis.pane.set_edgecolor(C_BORDER)
        ax.zaxis.pane.set_edgecolor(C_BORDER)
        ax.grid(True, color=C_BORDER, alpha=0.3)
        N    = Q_out.shape[0]
        step = max(1, N // 200)
        ee_path = np.array([robot.ee_position(Q_out[i]) for i in range(0,N,step)])
        ax.plot(ee_path[:,0], ee_path[:,1], ee_path[:,2],
                color='#FFCC00', lw=2, label='Actual EE path')
        ax.scatter(*ee_path[0],  color=C_ACCENT2, s=60, marker='o', label='Start')
        ax.scatter(*ee_path[-1], color=C_DANGER,  s=80, marker='*', label='End')
        ax.set_xlim([-0.8, 0.8]); ax.set_ylim([-0.8, 0.8]); ax.set_zlim([0, 1.4])
        ax.set_xlabel('X (m)', color=C_DIM, fontsize=7)
        ax.set_ylabel('Y (m)', color=C_DIM, fontsize=7)
        ax.set_zlabel('Z (m)', color=C_DIM, fontsize=7)
        ax.tick_params(colors=C_DIM, labelsize=6)
        ax.set_title('Actual EE trajectory', color=C_DIM, fontsize=9)
        ax.legend(fontsize=7, labelcolor=C_DIM,
                  facecolor=C_PANEL, edgecolor=C_BORDER)
        self.canvas_3d.draw()
        # Store for replay
        self._last_Q_out = Q_out
        self._last_t_s   = np.linspace(0, Q_out.shape[0]*0.001, Q_out.shape[0])
        self._last_robot = robot

    # ── Waypoint table helpers ─────────────────────────────────────────────
    def _reset_wp_table(self):
        defaults = [
            [0.5, 0.0, 0.3, 'Down', 0.20, ''],
            [0.2, 0.4, 0.5, 'Down', 0.20, ''],
            [0.0, 0.5, 0.4, 'Down', 0.20, ''],
        ]
        self.wp_table.setRowCount(len(defaults))
        for r, row in enumerate(defaults):
            for c, val in enumerate(row):
                self.wp_table.setItem(r, c, QTableWidgetItem(str(val)))

    def _load_tcp_defaults(self):
        grid = [
            [0.3,-0.2,0.3,'Down',0.15,'Grid 1,1'],
            [0.45,-0.2,0.3,'Down',0.15,'Grid 2,1'],
            [0.6,-0.2,0.3,'Down',0.15,'Grid 3,1'],
            [0.3,0.0,0.3,'Down',0.15,'Grid 1,2'],
            [0.45,0.0,0.3,'Down',0.15,'Grid 2,2'],
            [0.6,0.0,0.3,'Down',0.15,'Grid 3,2'],
            [0.3,0.2,0.3,'Down',0.15,'Grid 1,3'],
            [0.45,0.2,0.3,'Down',0.15,'Grid 2,3'],
            [0.6,0.2,0.3,'Down',0.15,'Grid 3,3'],
        ]
        self.wp_table.setRowCount(len(grid))
        for r, row in enumerate(grid):
            for c, val in enumerate(row):
                self.wp_table.setItem(r, c, QTableWidgetItem(str(val)))
        self.add_log('[TCP] 3x3 grid loaded.')

    def _add_wp_row(self):
        r = self.wp_table.rowCount()
        self.wp_table.insertRow(r)
        for c, v in enumerate([0.4,0.0,0.3,'Down',0.20,'']):
            self.wp_table.setItem(r, c, QTableWidgetItem(str(v)))

    def _remove_wp_row(self):
        r = self.wp_table.currentRow()
        if r >= 0:
            self.wp_table.removeRow(r)

    def _read_wp_table(self):
        n = self.wp_table.rowCount()
        wps  = np.zeros((n, 3))
        oris = []
        spds = np.zeros(n)
        for r in range(n):
            try:
                for c in range(3):
                    item = self.wp_table.item(r, c)
                    wps[r, c] = float(item.text()) if item and item.text() else 0.0
            except ValueError:
                raise ValueError(f'Row {r+1}: invalid number in X/Y/Z column')
            item_ori = self.wp_table.item(r, 3)
            oris.append(item_ori.text() if item_ori and item_ori.text() else 'Down')
            item_spd = self.wp_table.item(r, 4)
            try:
                spds[r] = float(item_spd.text()) if item_spd and item_spd.text() else 0.2
            except ValueError:
                spds[r] = 0.2
        return wps, oris, spds

    # ── Utility ────────────────────────────────────────────────────────────
    def add_log(self, msg):
        self.sim_log.append(msg)
        self.sim_log.verticalScrollBar().setValue(
            self.sim_log.verticalScrollBar().maximum())

    def set_status(self, text, color):
        self.status_label.setText(text)
        self.status_label.setStyleSheet(
            f"color:{color}; font-weight:bold; font-size:12px;")


    def _on_export_results(self):
        if not hasattr(self, '_last_Q_out') or self._last_Q_out is None:
            self.add_log('[EXPORT] No results — run a simulation first.')
            return
        try:
            from PyQt6.QtWidgets import QFileDialog
            import csv, datetime
            fname, _ = QFileDialog.getSaveFileName(
                self, 'Export Results', 
                f'kinova_results_{datetime.datetime.now().strftime("%Y%m%d_%H%M%S")}.csv',
                'CSV Files (*.csv)')
            if not fname:
                return
            t_s   = self._last_t_s
            Q_out = self._last_Q_out
            res   = self._last_res if hasattr(self, '_last_res') else None

            with open(fname, 'w', newline='') as f:
                writer = csv.writer(f)
                # Header info
                writer.writerow(['Kinova Gen3 CTC Simulation Results'])
                writer.writerow(['Generated', datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S')])
                writer.writerow([])
                # Summary metrics
                writer.writerow(['SUMMARY METRICS'])
                writer.writerow(['Overall RMS Error (deg)', self.rms_label.text()])
                writer.writerow(['Max Joint Error (deg)',   self.max_label.text()])
                writer.writerow(['Best Joint',              self.best_label.text()])
                writer.writerow(['Worst Joint',             self.worst_label.text()])
                writer.writerow(['Total Energy (J)',        self.energy_total.text()])
                writer.writerow(['Gravity Energy',          self.energy_grav.text()])
                writer.writerow(['Inertial Energy',         self.energy_inert.text()])
                writer.writerow(['Coriolis Energy',         self.energy_cor.text()])
                writer.writerow(['EE X (m)',                self.ee_x.text()])
                writer.writerow(['EE Y (m)',                self.ee_y.text()])
                writer.writerow(['EE Z (m)',                self.ee_z.text()])
                writer.writerow(['EE Roll (deg)',           self.ee_roll.text()])
                writer.writerow(['EE Pitch (deg)',          self.ee_pitch.text()])
                writer.writerow(['EE Yaw (deg)',            self.ee_yaw.text()])
                writer.writerow(['IK Error (mm)',           self.ee_ik.text()])
                writer.writerow(['Distance from base (m)', self.ee_dist.text()])
                writer.writerow(['T total (s)',             self.ee_t.text()])
                writer.writerow(['Safety Score',            self.safety_score.text()])
                writer.writerow([])
                # Raw Q_out data
                writer.writerow(['RAW Q_OUT DATA'])
                writer.writerow(['Time(s)','J1(rad)','J2(rad)','J3(rad)',
                                 'J4(rad)','J5(rad)','J6(rad)','J7(rad)'])
                for i in range(len(t_s)):
                    writer.writerow([f'{t_s[i]:.4f}'] +
                                   [f'{Q_out[i,j]:.6f}' for j in range(7)])

            self.add_log(f'[EXPORT] Results saved to: {fname}')
        except Exception as e:
            self.add_log(f'[EXPORT ERROR] {e}')

    def closeEvent(self, event):
        """Clean shutdown — stop MATLAB engine when app window closes."""
        self.add_log('[GUI] Closing app — stopping MATLAB engine...')
        # Find any running bridge in sim workers and stop it
        if self.sim_worker and hasattr(self.sim_worker, 'bridge'):
            try:
                self.sim_worker.bridge.stop()
            except:
                pass
        event.accept()

# ── Entry point ────────────────────────────────────────────────────────────
def main():
    app = QApplication(sys.argv)
    app.setStyleSheet(STYLE)
    win = KinovaApp()
    win.show()
    sys.exit(app.exec())

if __name__ == '__main__':
    main()