# Item Definition — Kinova Gen3 Collaborative Safety Layer

**Document ID:** SAF-ITM-001
**Project:** Kinova Gen3 CTC Simulator — Functional-Safety Layer
**Standards context:** ISO 12100, ISO 10218-1/-2, ISO/TS 15066, ISO 13849-1
**Status:** Concept demonstration (see Scope & Limitations)

> **Scope & limitations.** This document and the associated safety artifacts are a
> *concept demonstration* applying functional-safety methods to a simulated robot.
> They do **not** constitute a certified, standards-compliant safety system. No
> safety-rated hardware, tool qualification, or independent assessment is involved.
> The purpose is to demonstrate the functional-safety lifecycle and collaborative-
> robot safety concepts on an existing digital twin.

---

## 1. Item under consideration

The **item** is the closed-loop motion system of a 7-DOF Kinova Gen3 manipulator,
comprising:

- the **plant** — a Kinova Gen3 arm simulated in Simscape Multibody with full
  rigid-body dynamics (mass matrix, Coriolis, gravity);
- the **controller** — a Computed Torque Controller,
  `τ = M(q)·[q̈_ff + Kp·e + Kd·ė] + C(q,q̇) + G(q)`;
- the **trajectory planner** — joint-space profiles (cubic, quintic, LSPB,
  Hermite, bang-bang) and multi-waypoint Hermite paths;
- the **proposed safety layer** — an independent monitor that supervises the
  motion and can command a safe state.

## 2. Purpose and function

The robot follows planned Cartesian/joint trajectories to move its end effector
between targets (single-target and multi-waypoint pick-and-place-style motions).

## 3. Operational context — collaborative

The robot is considered to operate in a **collaborative workspace**: a human may
share the working area without physical fencing. This is the operating mode that
drives the ISO/TS 15066 collaborative safety requirements (Power & Force Limiting,
Speed & Separation Monitoring). Collaborative operation is the relevant framing
for human-robot collaboration products.

## 4. Operating modes

| Mode | Description |
|---|---|
| Automatic | Executes a planned trajectory at full commanded speed |
| Reduced-speed (collaborative) | Speed limited when a human is within the monitored zone |
| Safe stop | Motion halted and held in a safe state after a violation |

## 5. System boundary and interfaces

**Inside the boundary:** trajectory planner, CTC controller, plant, safety monitor.

**Signals available to the safety layer** (from the existing Python model):

| Signal | Source (existing code) |
|---|---|
| Joint positions `q` | `Q_out` / trajectory |
| Joint velocities `q̇` | trajectory `qd_ref` / finite difference of `Q_out` |
| Joint torques `τ` | `KinovaGen3.ctc_torque()` |
| TCP position | `KinovaGen3.ee_position()` (forward kinematics) |
| TCP velocity | `KinovaGen3.jacobian()` · `q̇` |
| Human/obstacle position | simulated operator model (new) |

**Outside the boundary:** the physical Kinova hardware, real sensors, real
safety-rated I/O, the human operator's own behaviour.

## 6. Relevant hardware limits (from `robot_model.py`)

| Parameter | J1–J3 | J4–J7 |
|---|---|---|
| Torque limit `TAU_MAX` | 187 Nm | 52 Nm |
| Velocity limit `QD_MAX` | 1.396 rad/s | 1.745 rad/s |
| Position limit `Q_LIM` | ±138.1° / ±152.4° | ±127.8° / ±119.7° |

These physical limits are the basis for the safety-function thresholds defined in
the Safety Requirements Specification (SAF-SRS-001).

## 7. Assumptions

- The simulation faithfully represents the robot's kinematics and dynamics
  (validated by the existing 7-section validation framework).
- A single human may be present; their position is provided by a simulated model.
- The safety layer executes at the control sample rate (1 ms).
- Collaborative speed/force thresholds are illustrative values derived from
  ISO/TS 15066 guidance, not a certified biomechanical assessment.
