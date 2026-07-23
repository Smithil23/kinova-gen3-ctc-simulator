"""
kinova_teleop.py  —  Kinova Gen3 Teleoperation Interface

A standalone teleoperation GUI for the Kinova Gen3 7-DOF arm.
Uses robot_model.py for real-time kinematics (no MATLAB required).

Modes
-----
Joint control   : 7 sliders drive each joint directly.
Cartesian control: XYZ + orientation target, IK computed on the fly.

Session recording
-----------------
Press Record → move the arm → Stop.  The session is saved as:
  <timestamp>_teleop.csv   — human-readable, importable into Excel / pandas
  <timestamp>_teleop.mat   — directly loadable by validateKinovaResults.m

Replay
------
Load a saved CSV and play it back at adjustable speed.

Run
---
    python kinova_teleop.py        (from the repo root)
"""

from __future__ import annotations

import csv
import os
import sys
import time
from datetime import datetime
from pathlib import Path

os.environ.setdefault("QT_API", "PySide6")

import matplotlib
matplotlib.use("QtAgg")
from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure

import numpy as np
from PySide6.QtCore import Qt, QThread, QTimer, Signal
from PySide6.QtWidgets import (
    QApplication, QComboBox, QDoubleSpinBox, QFileDialog, QFormLayout,
    QFrame, QGridLayout, QGroupBox, QHBoxLayout, QLabel, QMainWindow,
    QMessageBox, QPlainTextEdit, QPushButton, QSlider, QSplitter,
    QStackedWidget, QTabWidget, QVBoxLayout, QWidget,
)

# ---------------------------------------------------------------------------
# Attempt to import the real robot model.  If roboticstoolbox isn't installed
# we fall back to a lightweight stub so the GUI still opens for UI review.
# ---------------------------------------------------------------------------
_REPO_ROOT = Path(__file__).resolve().parent
if str(_REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(_REPO_ROOT))

try:
    from robot_model import KinovaGen3
    _ROBOT_AVAILABLE = True
except Exception as _e:
    _ROBOT_AVAILABLE = False
    print(f"[teleop] robot_model unavailable ({_e}); running in stub mode.")

    class KinovaGen3:                         # type: ignore[no-redef]
        N_JOINTS = 7
        TAU_MAX  = np.array([187,187,187,52,52,52,52], float)
        QD_MAX   = np.array([1.396]*3+[1.745]*4)
        Q_LIM_LO = np.deg2rad([-138.1,-138.1,-152.4,-127.8,-119.7,-119.7,-119.7])
        Q_LIM_HI = np.deg2rad([ 138.1, 138.1, 152.4, 127.8, 119.7, 119.7, 119.7])
        def __init__(self):    pass
        def home_config(self): return np.zeros(7)
        def ee_position(self, q): return np.array([0.5,0.,0.4])
        def ikine(self, T, q0=None):
            return np.zeros(7), False, 999.
        def target_transform(self, xyz, orientation="down"):
            from spatialmath import SE3
            return SE3.Trans(xyz)
        def gravity_torque(self, q): return np.zeros(7)

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------
JOINT_NAMES  = [f"J{i+1}" for i in range(7)]
ORIENTATIONS = ["down", "up", "horiz", "tilt45"]
SAVE_DIR     = Path("teleop_sessions")
PLOT_BG      = "#1b1e24"
FG           = "#c9d1d9"
OK_COL       = "#3fb950"
WARN_COL     = "#e3b341"
BAD_COL      = "#f0a6ad"
ACCENT       = "#4f9cf9"

STYLE = """
QWidget{background:#14171c;color:#c9d1d9;font-size:13px;}
QGroupBox{border:1px solid #2a2f38;border-radius:8px;margin-top:10px;
          padding:8px;font-weight:600;}
QGroupBox::title{subcontrol-origin:margin;left:10px;padding:0 4px;color:#8b98a9;}
QLabel{background:transparent;}
QPushButton{background:#232833;border:1px solid #313846;border-radius:6px;
            padding:6px 12px;color:#d7dee8;}
QPushButton:hover{background:#2c333f;}
QPushButton:disabled{color:#5a6472;border-color:#23272f;}
QPushButton#rec{background:#c0392b;border:none;color:white;font-weight:600;}
QPushButton#rec:checked{background:#27ae60;}
QPushButton#play{background:#2563eb;border:none;color:white;font-weight:600;}
QPushButton#home{background:#5f3dc4;border:none;color:white;}
QSlider::groove:horizontal{height:4px;background:#2a2f38;border-radius:2px;}
QSlider::handle:horizontal{width:14px;height:14px;margin:-5px 0;
    background:#4f9cf9;border-radius:7px;}
QSlider::sub-page:horizontal{background:#2563eb;border-radius:2px;}
QDoubleSpinBox,QComboBox{background:#1b1f27;border:1px solid #2f3644;
    border-radius:5px;padding:3px 6px;}
QPlainTextEdit{background:#0f1216;border:1px solid #2a2f38;border-radius:6px;
    color:#a7b2c0;font-family:Consolas,monospace;font-size:12px;}
QFrame#card{background:#1b1f27;border:1px solid #2a2f38;border-radius:8px;}
QTabBar::tab{background:#1b1f27;padding:6px 16px;border-radius:4px 4px 0 0;}
QTabBar::tab:selected{background:#232833;color:#e6edf3;}
"""

# ---------------------------------------------------------------------------
# Session logger
# ---------------------------------------------------------------------------
class SessionLogger:
    """Records timestamped (t, q1..q7, tcp_x, tcp_y, tcp_z) rows."""

    def __init__(self):
        self._rows: list[dict] = []
        self._t0: float | None = None
        self.active = False

    def start(self):
        self._rows.clear()
        self._t0 = time.perf_counter()
        self.active = True

    def stop(self):
        self.active = False

    def record(self, q: np.ndarray, tcp: np.ndarray):
        if not self.active or self._t0 is None:
            return
        t = time.perf_counter() - self._t0
        row = {"t": round(t, 4)}
        for i, v in enumerate(q):
            row[f"q{i+1}_deg"] = round(float(np.rad2deg(v)), 4)
        row["tcp_x"] = round(float(tcp[0]), 6)
        row["tcp_y"] = round(float(tcp[1]), 6)
        row["tcp_z"] = round(float(tcp[2]), 6)
        self._rows.append(row)

    def save(self) -> str | None:
        if not self._rows:
            return None
        SAVE_DIR.mkdir(exist_ok=True)
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        csv_path = SAVE_DIR / f"{stamp}_teleop.csv"
        mat_path = SAVE_DIR / f"{stamp}_teleop.mat"

        # CSV
        fields = list(self._rows[0].keys())
        with open(csv_path, "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=fields)
            w.writeheader()
            w.writerows(self._rows)

        # MAT (scipy if available, else skip gracefully)
        try:
            import scipy.io as sio
            mat = {k: np.array([r[k] for r in self._rows]) for k in fields}
            sio.savemat(str(mat_path), mat)
        except ImportError:
            mat_path = None

        return str(csv_path)

    @property
    def n_samples(self) -> int:
        return len(self._rows)

    def load_csv(self, path: str) -> tuple[np.ndarray, np.ndarray]:
        """Load a saved CSV → (t, Q) where Q is N×7 in radians."""
        rows = []
        with open(path, newline="") as f:
            for row in csv.DictReader(f):
                rows.append(row)
        t = np.array([float(r["t"]) for r in rows])
        Q = np.deg2rad(np.array(
            [[float(r[f"q{i+1}_deg"]) for i in range(7)] for r in rows]
        ))
        return t, Q


# ---------------------------------------------------------------------------
# IK worker (runs in background — IK can be slow)
# ---------------------------------------------------------------------------
class IKWorker(QThread):
    result = Signal(object, object, float)   # q, success, err_mm

    def __init__(self, robot, xyz, orientation, q0):
        super().__init__()
        self.robot = robot
        self.xyz = xyz
        self.orientation = orientation
        self.q0 = q0

    def run(self):
        T = self.robot.target_transform(self.xyz, self.orientation)
        q, ok, err = self.robot.ikine(T, self.q0)
        self.result.emit(q, ok, err)


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------
def _style_ax(ax, title=""):
    ax.set_facecolor(PLOT_BG)
    for s in ax.spines.values(): s.set_color("#2a2f38")
    ax.tick_params(colors=FG, labelsize=8)
    ax.grid(True, color="#242a33", lw=0.6)
    if title: ax.set_title(title, color=FG, fontsize=9)


def _card(label, value="—", value_color="#e6edf3"):
    f = QFrame(); f.setObjectName("card")
    lay = QVBoxLayout(f)
    cap = QLabel(label); cap.setStyleSheet("color:#8b98a9;font-size:11px;")
    val = QLabel(value)
    val.setObjectName("cardval")
    val.setStyleSheet(f"font-size:18px;font-weight:600;color:{value_color};")
    lay.addWidget(cap); lay.addWidget(val)
    return f, val


# ---------------------------------------------------------------------------
# Joint-control panel
# ---------------------------------------------------------------------------
class JointPanel(QWidget):
    q_changed = Signal(np.ndarray)

    def __init__(self, robot):
        super().__init__()
        self._robot = robot
        self._q = robot.home_config().copy()
        self._sliders: list[QSlider] = []
        self._labels:  list[QLabel]  = []
        grid = QGridLayout(self)
        grid.setSpacing(6)
        grid.addWidget(QLabel("<b>Joint</b>"),   0, 0)
        grid.addWidget(QLabel("<b>Angle (°)</b>"), 0, 1)
        grid.addWidget(QLabel("<b>Slider</b>"),  0, 2)
        grid.addWidget(QLabel("<b>Limit</b>"),   0, 3)
        for i in range(7):
            lo = int(np.rad2deg(robot.Q_LIM_LO[i]))
            hi = int(np.rad2deg(robot.Q_LIM_HI[i]))
            sl = QSlider(Qt.Horizontal)
            sl.setRange(lo * 10, hi * 10)
            sl.setValue(0)
            sl.setMinimumWidth(160)
            lbl = QLabel("0.0°")
            lbl.setFixedWidth(52)
            lim = QLabel(f"[{lo}°, {hi}°]")
            lim.setStyleSheet("color:#8b98a9;font-size:11px;")
            sl.valueChanged.connect(lambda v, idx=i: self._on_slider(idx, v))
            self._sliders.append(sl)
            self._labels.append(lbl)
            grid.addWidget(QLabel(JOINT_NAMES[i]), i+1, 0)
            grid.addWidget(lbl, i+1, 1)
            grid.addWidget(sl,  i+1, 2)
            grid.addWidget(lim, i+1, 3)
        btn_row = QHBoxLayout()
        home_btn = QPushButton("Go home")
        home_btn.setObjectName("home")
        home_btn.clicked.connect(self.go_home)
        btn_row.addWidget(home_btn)
        btn_row.addStretch(1)
        grid.addLayout(btn_row, 8, 0, 1, 4)

    def _on_slider(self, idx: int, val_10: int):
        deg = val_10 / 10.0
        self._q[idx] = np.deg2rad(deg)
        self._labels[idx].setText(f"{deg:.1f}°")
        self.q_changed.emit(self._q.copy())

    def go_home(self):
        for sl in self._sliders: sl.setValue(0)

    def set_q(self, q: np.ndarray):
        for i, v in enumerate(q):
            self._sliders[i].blockSignals(True)
            self._sliders[i].setValue(int(np.rad2deg(v) * 10))
            self._labels[i].setText(f"{np.rad2deg(v):.1f}°")
            self._sliders[i].blockSignals(False)
        self._q = q.copy()


# ---------------------------------------------------------------------------
# Cartesian-control panel
# ---------------------------------------------------------------------------
class CartesianPanel(QWidget):
    q_solved = Signal(np.ndarray)
    ik_status = Signal(str, str)   # message, colour

    def __init__(self, robot):
        super().__init__()
        self._robot = robot
        self._q0 = robot.home_config().copy()
        self._worker = None
        form = QFormLayout(self)

        def dspin(lo, hi, val, step=0.01):
            w = QDoubleSpinBox()
            w.setRange(lo, hi); w.setSingleStep(step)
            w.setDecimals(3); w.setValue(val)
            return w

        self.x = dspin(-1.0, 1.0, 0.40)
        self.y = dspin(-1.0, 1.0, 0.00)
        self.z = dspin( 0.0, 1.2, 0.40)
        self.ori = QComboBox(); self.ori.addItems(ORIENTATIONS)
        form.addRow("Target X (m)", self.x)
        form.addRow("Target Y (m)", self.y)
        form.addRow("Target Z (m)", self.z)
        form.addRow("Orientation",  self.ori)

        self.ik_btn = QPushButton("Solve IK & move")
        self.ik_btn.setObjectName("play")
        self.ik_btn.clicked.connect(self._solve)
        self.status_lbl = QLabel("Enter a target and press Solve.")
        self.status_lbl.setWordWrap(True)
        self.status_lbl.setStyleSheet("color:#8b98a9;font-size:11px;")
        form.addRow(self.ik_btn)
        form.addRow(self.status_lbl)

    def set_q0(self, q: np.ndarray):
        """Update the IK seed from the current joint state."""
        self._q0 = q.copy()

    def _solve(self):
        if self._worker and self._worker.isRunning():
            return
        self.ik_btn.setEnabled(False)
        self.status_lbl.setText("Solving IK…")
        self._worker = IKWorker(
            self._robot,
            [self.x.value(), self.y.value(), self.z.value()],
            self.ori.currentText(),
            self._q0,
        )
        self._worker.result.connect(self._on_result)
        self._worker.start()

    def _on_result(self, q, ok, err):
        self.ik_btn.setEnabled(True)
        if ok:
            self.status_lbl.setText(f"IK solved — error {err:.3f} mm")
            self.q_solved.emit(q)
        else:
            self.status_lbl.setText(
                f"IK failed (error {err:.1f} mm). Try a different target.")


# ---------------------------------------------------------------------------
# Main window
# ---------------------------------------------------------------------------
class TeleopWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Kinova Gen3 — Teleoperation Interface")
        self.resize(1100, 720)

        self._robot  = KinovaGen3()
        self._q      = self._robot.home_config().copy()
        self._logger = SessionLogger()
        self._replay_timer = QTimer(self)
        self._replay_timer.timeout.connect(self._replay_step)
        self._replay_data: tuple | None = None
        self._replay_idx  = 0

        self._build_ui()

        # Update rate for live FK / plot refresh
        self._update_timer = QTimer(self)
        self._update_timer.setInterval(50)   # 20 Hz
        self._update_timer.timeout.connect(self._on_tick)
        self._update_timer.start()

        self._log("Kinova Gen3 teleoperation interface ready.")
        self._log(f"Robot: {self._robot.N_JOINTS}-DOF  |  "
                  f"{'robot_model loaded' if _ROBOT_AVAILABLE else 'STUB MODE'}")
        self._update_tcp()

    # ------------------------------------------------------------------
    def _build_ui(self):
        central = QWidget()
        self.setCentralWidget(central)
        root = QHBoxLayout(central)

        # ---- left: control panels ------------------------------------
        left = QVBoxLayout()
        tabs = QTabWidget()

        # Joint tab
        self._joint_panel = JointPanel(self._robot)
        self._joint_panel.q_changed.connect(self._on_q_changed)
        tabs.addTab(self._joint_panel, "Joint control")

        # Cartesian tab
        self._cart_panel = CartesianPanel(self._robot)
        self._cart_panel.q_solved.connect(self._on_q_changed)
        self._cart_panel.q_solved.connect(self._joint_panel.set_q)
        tabs.addTab(self._cart_panel, "Cartesian control")

        left.addWidget(tabs, 1)

        # Recording controls
        rec_box = QGroupBox("Session recording")
        rl = QVBoxLayout(rec_box)
        btn_row = QHBoxLayout()
        self._rec_btn = QPushButton("⏺  Record")
        self._rec_btn.setObjectName("rec")
        self._rec_btn.setCheckable(True)
        self._rec_btn.clicked.connect(self._toggle_record)
        self._stop_btn = QPushButton("⏹  Stop")
        self._stop_btn.setEnabled(False)
        self._stop_btn.clicked.connect(self._stop_record)
        btn_row.addWidget(self._rec_btn)
        btn_row.addWidget(self._stop_btn)
        rl.addLayout(btn_row)
        self._rec_lbl = QLabel("Not recording.")
        self._rec_lbl.setStyleSheet("color:#8b98a9;font-size:11px;")
        rl.addWidget(self._rec_lbl)

        replay_row = QHBoxLayout()
        load_btn = QPushButton("📂  Load & replay CSV")
        load_btn.clicked.connect(self._load_replay)
        self._replay_speed = QDoubleSpinBox()
        self._replay_speed.setRange(0.1, 5.0)
        self._replay_speed.setValue(1.0)
        self._replay_speed.setSingleStep(0.1)
        self._replay_speed.setPrefix("×")
        self._replay_speed.setToolTip("Replay speed multiplier")
        replay_row.addWidget(load_btn)
        replay_row.addWidget(QLabel("Speed"))
        replay_row.addWidget(self._replay_speed)
        rl.addLayout(replay_row)
        left.addWidget(rec_box)

        wrap = QWidget()
        wrap.setLayout(left)
        wrap.setFixedWidth(360)
        root.addWidget(wrap)

        # ---- right: telemetry ----------------------------------------
        right = QVBoxLayout()

        # Cards
        cards = QGridLayout()
        self._card_x,   self._cv_x   = _card("TCP X")
        self._card_y,   self._cv_y   = _card("TCP Y")
        self._card_z,   self._cv_z   = _card("TCP Z")
        self._card_tau, self._cv_tau = _card("Max torque %")
        self._card_rec, self._cv_rec = _card("Recording", "Off", BAD_COL)
        for i, c in enumerate([self._card_x, self._card_y, self._card_z,
                                self._card_tau, self._card_rec]):
            cards.addWidget(c, 0, i)
        right.addLayout(cards)

        # Live joint-angle plot
        self._fig = Figure(figsize=(5, 2.2), facecolor=PLOT_BG, tight_layout=True)
        self._ax  = self._fig.add_subplot(111)
        _style_ax(self._ax, "Joint angles (deg)")
        self._ax.set_xlabel("Joint", color=FG)
        self._lines = self._ax.bar(JOINT_NAMES, np.zeros(7),
                                    color=ACCENT, alpha=0.7)
        self._ax.set_ylim(-160, 160)
        self._canvas = FigureCanvas(self._fig)
        right.addWidget(self._canvas)

        # TCP trajectory plot (session history)
        self._fig2 = Figure(figsize=(5, 2.0), facecolor=PLOT_BG, tight_layout=True)
        self._ax2  = self._fig2.add_subplot(111)
        _style_ax(self._ax2, "TCP trajectory (recorded session)")
        self._ax2.set_xlabel("Time (s)", color=FG)
        self._ax2.set_ylabel("Position (m)", color=FG)
        self._canvas2 = FigureCanvas(self._fig2)
        right.addWidget(self._canvas2)

        # Log
        log_box = QGroupBox("Log")
        lb = QVBoxLayout(log_box)
        self._log_view = QPlainTextEdit()
        self._log_view.setReadOnly(True)
        self._log_view.setMaximumHeight(90)
        lb.addWidget(self._log_view)
        right.addWidget(log_box)

        root.addLayout(right, 1)

    # ------------------------------------------------------------------
    def _log(self, msg: str):
        stamp = datetime.now().strftime("%H:%M:%S")
        self._log_view.appendPlainText(f"[{stamp}] {msg}")

    def _on_q_changed(self, q: np.ndarray):
        self._q = q.copy()
        self._cart_panel.set_q0(q)
        self._update_tcp()

    def _update_tcp(self):
        tcp = self._robot.ee_position(self._q)
        self._cv_x.setText(f"{tcp[0]:.3f} m")
        self._cv_y.setText(f"{tcp[1]:.3f} m")
        self._cv_z.setText(f"{tcp[2]:.3f} m")
        # Gravity torque as a % of rated
        try:
            tau = self._robot.gravity_torque(self._q)
            pct = np.max(np.abs(tau) / self._robot.TAU_MAX) * 100
            color = OK_COL if pct < 70 else (WARN_COL if pct < 90 else BAD_COL)
            self._cv_tau.setText(f"{pct:.1f} %")
            self._cv_tau.setStyleSheet(
                f"font-size:18px;font-weight:600;color:{color};")
        except Exception:
            pass
        # Log to session
        if self._logger.active:
            self._logger.record(self._q, tcp)

    def _on_tick(self):
        """Refresh the bar chart at 20 Hz."""
        degs = np.rad2deg(self._q)
        for bar, d in zip(self._lines, degs):
            bar.set_height(d)
        self._canvas.draw_idle()

    # ------------------------------------------------------------------
    # Recording
    # ------------------------------------------------------------------
    def _toggle_record(self, checked: bool):
        if checked:
            self._logger.start()
            self._rec_btn.setText("● Recording…")
            self._stop_btn.setEnabled(True)
            self._cv_rec.setText("ON")
            self._cv_rec.setStyleSheet(
                f"font-size:18px;font-weight:600;color:{OK_COL};")
            self._rec_lbl.setText("Recording in progress…")
            self._log("Recording started.")
        else:
            self._stop_record()

    def _stop_record(self):
        self._logger.stop()
        self._rec_btn.setChecked(False)
        self._rec_btn.setText("⏺  Record")
        self._stop_btn.setEnabled(False)
        self._cv_rec.setText("Off")
        self._cv_rec.setStyleSheet(
            f"font-size:18px;font-weight:600;color:{BAD_COL};")
        n = self._logger.n_samples
        if n == 0:
            self._rec_lbl.setText("Stopped — no data recorded.")
            self._log("Recording stopped — no data.")
            return
        path = self._logger.save()
        self._rec_lbl.setText(f"Saved {n} samples → {Path(path).name}")
        self._log(f"Session saved: {path}  ({n} samples)")
        self._plot_session()

    def _plot_session(self):
        """Plot the TCP path of the last recorded session."""
        rows = self._logger._rows
        if not rows: return
        t   = [r["t"] for r in rows]
        tx  = [r["tcp_x"] for r in rows]
        ty  = [r["tcp_y"] for r in rows]
        tz  = [r["tcp_z"] for r in rows]
        self._ax2.clear()
        _style_ax(self._ax2, "TCP trajectory (recorded session)")
        self._ax2.set_xlabel("Time (s)", color=FG)
        self._ax2.set_ylabel("Position (m)", color=FG)
        self._ax2.plot(t, tx, color="#4f9cf9", lw=1, label="X")
        self._ax2.plot(t, ty, color="#3fb950", lw=1, label="Y")
        self._ax2.plot(t, tz, color="#e3b341", lw=1, label="Z")
        self._ax2.legend(fontsize=7, facecolor=PLOT_BG, labelcolor=FG)
        self._canvas2.draw()

    # ------------------------------------------------------------------
    # Replay
    # ------------------------------------------------------------------
    def _load_replay(self):
        path, _ = QFileDialog.getOpenFileName(
            self, "Load teleop session CSV", str(SAVE_DIR), "CSV files (*.csv)")
        if not path: return
        try:
            t, Q = self._logger.load_csv(path)
        except Exception as e:
            QMessageBox.warning(self, "Load error", str(e))
            return
        self._replay_data = (t, Q)
        self._replay_idx  = 0
        n = len(t)
        self._log(f"Loaded {n} samples from {Path(path).name} — starting replay.")
        # Interval between steps (ms), adjusted for speed
        dt_ms = int((t[1] - t[0]) * 1000 / self._replay_speed.value()) if n > 1 else 50
        self._replay_timer.start(max(dt_ms, 16))

    def _replay_step(self):
        if self._replay_data is None:
            self._replay_timer.stop(); return
        t, Q = self._replay_data
        idx = self._replay_idx
        if idx >= len(Q):
            self._replay_timer.stop()
            self._log("Replay complete.")
            return
        q = Q[idx]
        self._joint_panel.set_q(q)
        self._on_q_changed(q)
        self._replay_idx += 1


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------
def main():
    app = QApplication(sys.argv)
    app.setStyleSheet(STYLE)
    win = TeleopWindow()
    win.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
