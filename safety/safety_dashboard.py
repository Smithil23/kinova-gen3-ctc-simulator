"""
Kinova Safety Monitor — dashboard GUI.

A standalone PySide6 application that runs a trajectory through the safety monitor
with a simulated approaching human, and visualises the result: a workspace view,
safety-function status lights, live readouts, and the separation/speed plot.

This is the INDEPENDENT safety layer's front end. It imports the existing project
(robot model, trajectory generator) read-only and modifies nothing.

Run:  python -m safety.safety_dashboard      (from the repo root)
  or: python safety/safety_dashboard.py

Editable in the GUI: the collaborative safety parameters (safe speed, distances,
times). Read-only: the rated hardware limits (TAU_MAX, QD_MAX, Q_LIM) — a safety
system must not let a protective hardware limit be silently weakened.
"""

from __future__ import annotations

import os
import sys

os.environ.setdefault("QT_API", "PySide6")

import matplotlib

matplotlib.use("QtAgg")
from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure

import numpy as np
from PySide6.QtCore import Qt, QThread, Signal
from PySide6.QtWidgets import (
    QApplication,
    QComboBox,
    QDoubleSpinBox,
    QFormLayout,
    QFrame,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QMainWindow,
    QPushButton,
    QVBoxLayout,
    QWidget,
)

_REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _REPO_ROOT not in sys.path:
    sys.path.insert(0, _REPO_ROOT)

from safety.safety_monitor import SafetyParams

PLOT_BG = "#1b1e24"
FG = "#c9d1d9"
OK = "#3fb950"
WARN = "#e3b341"
BAD = "#f0a6ad"
ACCENT = "#4f9cf9"

STYLE = """
QWidget { background: #14171c; color: #c9d1d9; font-size: 13px; }
QGroupBox { border: 1px solid #2a2f38; border-radius: 8px; margin-top: 10px;
            padding: 8px; font-weight: 600; }
QGroupBox::title { subcontrol-origin: margin; left: 10px; padding: 0 4px; color: #8b98a9; }
QLabel { background: transparent; }
QPushButton#run { background: #2563eb; border: none; color: white; font-weight: 600;
                  border-radius: 6px; padding: 8px 14px; }
QPushButton#run:disabled { background: #24304a; color: #6b7686; }
QDoubleSpinBox, QComboBox { background: #1b1f27; border: 1px solid #2f3644;
                            border-radius: 5px; padding: 3px 6px; }
QDoubleSpinBox:disabled { color: #6b7686; background: #171a20; }
QFrame#card { background: #1b1f27; border: 1px solid #2a2f38; border-radius: 8px; }
"""


def dspin(lo, hi, val, step, dec=2, enabled=True):
    w = QDoubleSpinBox()
    w.setRange(lo, hi)
    w.setSingleStep(step)
    w.setDecimals(dec)
    w.setValue(val)
    w.setEnabled(enabled)
    return w


class Worker(QThread):
    done = Signal(dict, object)
    failed = Signal(str)

    def __init__(self, params, target, speed, curve, hstart, htoward):
        super().__init__()
        self.params, self.target = params, target
        self.speed, self.curve = speed, curve
        self.hstart, self.htoward = hstart, htoward

    def run(self):
        try:
            from safety.safety_runner import run_scenario

            robot, results = run_scenario(
                target_xyz=self.target, tool_speed=self.speed, curve_type=self.curve,
                human_start=self.hstart, human_toward=self.htoward, params=self.params,
            )
            self.done.emit(results, robot)
        except Exception as exc:  # noqa: BLE001
            self.failed.emit(f"{type(exc).__name__}: {exc}")


class Light(QWidget):
    def __init__(self, label):
        super().__init__()
        lay = QHBoxLayout(self)
        lay.setContentsMargins(0, 2, 0, 2)
        self.dot = QLabel("\u25cf")
        self.dot.setStyleSheet(f"color: {OK}; font-size: 14px;")
        lay.addWidget(self.dot)
        lay.addWidget(QLabel(label))
        lay.addStretch(1)

    def set(self, color):
        self.dot.setStyleSheet(f"color: {color}; font-size: 14px;")


class Dashboard(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Kinova Safety Monitor")
        self.resize(1080, 720)
        self._build()

    def _build(self):
        central = QWidget()
        self.setCentralWidget(central)
        root = QHBoxLayout(central)

        # ---- left: parameters ----
        left = QVBoxLayout()

        scen = QGroupBox("Scenario")
        sf = QFormLayout(scen)
        self.tx = dspin(-1, 1, 0.4, 0.05)
        self.ty = dspin(-1, 1, 0.2, 0.05)
        self.tz = dspin(0, 1, 0.3, 0.05)
        self.speed = dspin(0.01, 1.0, 0.20, 0.01)
        self.curve = QComboBox()
        self.curve.addItems(["cubic", "quintic", "lspb", "hermite", "bangbang"])
        sf.addRow("Target x", self.tx)
        sf.addRow("Target y", self.ty)
        sf.addRow("Target z", self.tz)
        sf.addRow("Tool speed (m/s)", self.speed)
        sf.addRow("Profile", self.curve)

        safe = QGroupBox("Safety parameters (editable)")
        sfa = QFormLayout(safe)
        self.v_safe = dspin(0.05, 1.0, 0.25, 0.05)
        self.s_min = dspin(0.05, 1.0, 0.30, 0.05)
        self.v_human = dspin(0.1, 3.0, 1.6, 0.1)
        self.t_react = dspin(0.0, 1.0, 0.10, 0.01)
        self.t_stop = dspin(0.0, 1.0, 0.20, 0.01)
        self.tau_frac = dspin(0.1, 1.0, 0.50, 0.05)
        sfa.addRow("Safe TCP speed (m/s)", self.v_safe)
        sfa.addRow("Min separation (m)", self.s_min)
        sfa.addRow("Human speed (m/s)", self.v_human)
        sfa.addRow("Reaction time (s)", self.t_react)
        sfa.addRow("Stop time (s)", self.t_stop)
        sfa.addRow("Torque fraction", self.tau_frac)

        hw = QGroupBox("Hardware limits (read-only \u2014 protected)")
        hf = QFormLayout(hw)
        hf.addRow("Torque limit J1\u2013J3", self._ro("187 Nm"))
        hf.addRow("Torque limit J4\u2013J7", self._ro("52 Nm"))
        hf.addRow("Velocity limit", self._ro("1.40 / 1.75 rad/s"))
        note = QLabel("Rated limits are shown but not editable: a safety system\n"
                      "must not let a protective hardware limit be weakened.")
        note.setWordWrap(True)
        note.setStyleSheet("color: #8b98a9; font-size: 11px;")

        self.run_btn = QPushButton("Run safety analysis")
        self.run_btn.setObjectName("run")
        self.run_btn.clicked.connect(self._run)

        for w in (scen, safe, hw, note, self.run_btn):
            left.addWidget(w)
        left.addStretch(1)
        wrap = QWidget()
        wrap.setLayout(left)
        wrap.setFixedWidth(320)
        root.addWidget(wrap)

        # ---- right: results ----
        right = QVBoxLayout()

        cards = QGridLayout()
        self.card_state = self._card("System state")
        self.card_sep = self._card("Min separation")
        self.card_stop = self._card("Safe stop")
        for i, c in enumerate((self.card_state, self.card_sep, self.card_stop)):
            cards.addWidget(c["frame"], 0, i)
        right.addLayout(cards)

        funcs = QGroupBox("Safety functions")
        fl = QVBoxLayout(funcs)
        self.lights = {}
        for key, lbl in [
            ("pos", "Joint position limits"),
            ("vel", "Joint speed limits"),
            ("tau", "Torque / force limit"),
            ("cart", "Cartesian speed limit"),
            ("ssm", "Speed & separation (SSM)"),
            ("plan", "Plan integrity"),
        ]:
            lt = Light(lbl)
            self.lights[key] = lt
            fl.addWidget(lt)
        right.addWidget(funcs)

        self.fig = Figure(figsize=(5, 3), facecolor=PLOT_BG, tight_layout=True)
        self.ax = self.fig.add_subplot(111)
        self.canvas = FigureCanvas(self.fig)
        self._reset_plot()
        right.addWidget(self.canvas, 1)

        self.status = QLabel("Set parameters and run a safety analysis.")
        self.status.setStyleSheet("color: #8b98a9;")
        right.addWidget(self.status)

        root.addLayout(right, 1)

    def _ro(self, text):
        lb = QLabel(text)
        lb.setStyleSheet("color: #8b98a9;")
        return lb

    def _card(self, label):
        f = QFrame()
        f.setObjectName("card")
        lay = QVBoxLayout(f)
        cap = QLabel(label)
        cap.setStyleSheet("color: #8b98a9; font-size: 11px;")
        val = QLabel("\u2014")
        val.setStyleSheet("font-size: 20px; font-weight: 600;")
        lay.addWidget(cap)
        lay.addWidget(val)
        return {"frame": f, "value": val}

    def _reset_plot(self):
        self.ax.clear()
        self.ax.set_facecolor(PLOT_BG)
        for s in self.ax.spines.values():
            s.set_color("#2a2f38")
        self.ax.tick_params(colors=FG, labelsize=8)
        self.ax.set_title("Separation vs. protective distance & speed factor", color=FG)
        self.ax.set_xlabel("Time (s)", color=FG)
        self.ax.grid(True, color="#242a33", lw=0.6)
        self.canvas.draw()

    def _params(self):
        return SafetyParams(
            v_safe=self.v_safe.value(),
            s_min=self.s_min.value(),
            v_human=self.v_human.value(),
            reaction_time=self.t_react.value(),
            stop_time=self.t_stop.value(),
            tau_fraction=self.tau_frac.value(),
        )

    def _run(self):
        self.run_btn.setEnabled(False)
        self.status.setText("Running safety analysis\u2026")
        self.worker = Worker(
            self._params(),
            (self.tx.value(), self.ty.value(), self.tz.value()),
            self.speed.value(),
            self.curve.currentText(),
            (2.0, self.ty.value(), self.tz.value()),
            (self.tx.value(), self.ty.value(), self.tz.value()),
        )
        self.worker.done.connect(self._show)
        self.worker.failed.connect(self._err)
        self.worker.start()

    def _show(self, results, robot):
        self.run_btn.setEnabled(True)
        stopped = results["stopped"]
        state = "Safe stop" if stopped else (
            "Reduced" if "Reduced speed" in results["state"] else "Normal")
        color = BAD if stopped else (WARN if state == "Reduced" else OK)
        self.card_state["value"].setText(state)
        self.card_state["value"].setStyleSheet(
            f"font-size: 20px; font-weight: 600; color: {color};")
        sep = results["separation"]
        finite = sep[np.isfinite(sep)]
        self.card_sep["value"].setText(f"{finite.min():.2f} m" if finite.size else "\u2014")
        self.card_stop["value"].setText("Yes" if stopped else "No")
        self.card_stop["value"].setStyleSheet(
            f"font-size: 20px; font-weight: 600; color: {BAD if stopped else OK};")

        # Aggregate which functions ever tripped.
        tripped = set()
        for v in results["violations"]:
            tripped.update(v)
        for w in results["warnings"]:
            tripped.update(w)
        mapping = {
            "pos": {"SR-4.1"}, "vel": {"SR-5.1"}, "tau": {"SR-1.1", "SR-1.2", "SR-1.3"},
            "cart": {"SR-2.1"}, "ssm": {"SR-3.4", "SR-3.5"}, "plan": set(),
        }
        for key, ids in mapping.items():
            hit = bool(ids & tripped)
            self.lights[key].set(WARN if hit else OK)
        self.lights["plan"].set(OK if results["plan_ok"] else BAD)

        self._reset_plot()
        t = results["t"]
        self.ax.plot(t, results["separation"], color=FG, lw=1.2, label="separation")
        self.ax.plot(t, results["protective_distance"], color=WARN, lw=1, ls="--",
                     label="protective dist.")
        self.ax.plot(t, results["speed_scale"], color=ACCENT, lw=1.5, label="speed factor")
        self.ax.legend(fontsize=7, facecolor=PLOT_BG, labelcolor=FG)
        self.canvas.draw()
        self.status.setText(
            f"Done. {'Safe stop triggered.' if stopped else 'Completed without safe stop.'}")

    def _err(self, msg):
        self.run_btn.setEnabled(True)
        self.status.setText(f"Error: {msg}")


def main():
    app = QApplication(sys.argv)
    app.setStyleSheet(STYLE)
    win = Dashboard()
    win.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
