"""
Simulated human model for Speed & Separation Monitoring (SSM).

This is a *standalone* model — it does not touch the robot model, the controller,
or the Simulink file. It simply answers one question: where is the human at time t?

In a real system the human position would come from a safety-rated sensor
(laser scanner, 3D camera). Here it is a scripted trajectory so the SSM logic can
be demonstrated responding to an approach. This is a concept demonstration, not a
sensor-based safety function.
"""

from __future__ import annotations

import numpy as np


class SimulatedHuman:
    """A human/operator whose position varies with time.

    Default scenario: the human starts ~2 m from the robot base and walks
    toward it along a straight line, then holds a minimum standoff.
    """

    def __init__(
        self,
        start_xyz=(2.0, 0.0, 0.4),
        toward_xyz=(0.3, 0.0, 0.4),
        approach_speed=1.6,   # m/s, ISO/TS 15066 default human speed
        start_time=0.0,       # s, when the human begins approaching
    ):
        self.start = np.asarray(start_xyz, dtype=float)
        self.toward = np.asarray(toward_xyz, dtype=float)
        self.approach_speed = float(approach_speed)
        self.start_time = float(start_time)

        direction = self.toward - self.start
        self._distance = float(np.linalg.norm(direction))
        self._unit = direction / self._distance if self._distance > 1e-9 else np.zeros(3)
        # Time at which the human reaches the closest point.
        self._arrive_time = self.start_time + self._distance / max(self.approach_speed, 1e-9)

    def position_at(self, t: float) -> np.ndarray:
        """Return the human [x, y, z] position at time t (seconds)."""
        if t <= self.start_time:
            return self.start.copy()
        if t >= self._arrive_time:
            return self.toward.copy()
        travelled = self.approach_speed * (t - self.start_time)
        return self.start + self._unit * travelled

    def velocity_toward(self, t: float, robot_point: np.ndarray) -> float:
        """Component of the human's velocity directed at the robot point (m/s).

        Used by the SSM protective-distance formula. Positive means approaching.
        """
        if t <= self.start_time or t >= self._arrive_time:
            return 0.0
        human = self.position_at(t)
        to_robot = np.asarray(robot_point, dtype=float) - human
        n = np.linalg.norm(to_robot)
        if n < 1e-9:
            return self.approach_speed
        to_robot_unit = to_robot / n
        return float(max(0.0, np.dot(self._unit * self.approach_speed, to_robot_unit)))
