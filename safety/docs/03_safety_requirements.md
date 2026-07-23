# Safety Requirements Specification (SRS)

**Document ID:** SAF-SRS-001
**Item:** Kinova Gen3 Collaborative Safety Layer (see SAF-ITM-001)
**Derived from:** HARA safety goals SG1–SG9 (SAF-HARA-001)
**Status:** Concept demonstration.

> **Scope note.** The threshold values below are illustrative, drawn from ISO/TS
> 15066 and ISO 10218 guidance. They are engineering defaults for a simulated
> concept demonstration, **not** values from a certified biomechanical assessment.

---

## 1. Purpose

This document refines each HARA safety goal into concrete, testable **safety
requirements**. Each requirement has a unique ID, a measurable threshold, a trace
to the hazard it mitigates, and a trace to the verification test that confirms it.
This hazard → goal → requirement → verification chain is the core of the safety
argument.

## 2. Parameter set (illustrative defaults)

| Symbol | Meaning | Value | Source |
|---|---|---|---|
| v_safe | Safe collaborative TCP speed | 0.25 m/s | ISO/TS 15066 |
| τ_col | Collaborative torque cap | 50% of `TAU_MAX` per joint | Derived |
| v_H | Human approach speed | 1.6 m/s | ISO/TS 15066 |
| T_r | System reaction time | 0.10 s | Assumed |
| T_s | Robot stopping time | 0.20 s | Assumed |
| S_min | Minimum protective distance | 0.30 m | Assumed |
| C, Z | Uncertainty margins | 0.10 m total | Assumed |

The dynamic protective separation distance (ISO/TS 15066) is:

**S_p = v_H·(T_r + T_s) + v_R·T_r + v_R²/(2·a_R) + (C + Z)**

where v_R is the robot's TCP speed toward the human and a_R its deceleration.

## 3. Safety requirements

### SG1 — Power & Force Limiting (hazards H1, H6)

| ID | Requirement | Threshold | Verifies |
|---|---|---|---|
| SR-1.1 | Commanded joint torque shall not exceed the rated torque limit | `|τ_i| ≤ TAU_MAX[i]` | VP-1.1 |
| SR-1.2 | In collaborative mode, joint torque shall be capped to the collaborative limit | `|τ_i| ≤ τ_col[i]` | VP-1.2 |
| SR-1.3 | Torque saturation shall be flagged as a warning | saturation detected | VP-1.3 |

### SG2 — Safe collaborative speed (hazard H2)

| ID | Requirement | Threshold | Verifies |
|---|---|---|---|
| SR-2.1 | TCP speed shall not exceed the safe collaborative speed while a human is in the collaborative zone | `|v_TCP| ≤ v_safe` | VP-2.1 |

### SG3 — Speed & Separation Monitoring (hazard H3) ⭐

| ID | Requirement | Threshold | Verifies |
|---|---|---|---|
| SR-3.1 | The monitor shall continuously compute separation `d` between the TCP and the human | d computed every cycle | VP-3.1 |
| SR-3.2 | The monitor shall compute the required protective distance `S_p` from current speeds | S_p per §2 formula | VP-3.2 |
| SR-3.3 | When `d ≥ S_p`, full speed is permitted | speed scale = 1.0 | VP-3.3 |
| SR-3.4 | When `S_min < d < S_p`, robot speed shall be reduced proportionally | 0 < scale < 1.0 | VP-3.4 |
| SR-3.5 | When `d ≤ S_min`, the robot shall enter a safe stop | speed scale = 0 | VP-3.5 |

### SG4 — Joint range limiting (hazard H4)

| ID | Requirement | Threshold | Verifies |
|---|---|---|---|
| SR-4.1 | Joint positions shall remain within validated limits | `Q_LIM_LO[i] ≤ q_i ≤ Q_LIM_HI[i]` | VP-4.1 |
| SR-4.2 | A trajectory commanding an out-of-range joint shall be blocked before execution | plan rejected | VP-4.2 |

### SG5 — Joint speed limiting (hazard H5)

| ID | Requirement | Threshold | Verifies |
|---|---|---|---|
| SR-5.1 | Joint velocities shall not exceed rated limits | `|q̇_i| ≤ QD_MAX[i]` | VP-5.1 |

### SG6 — Torque supervision (hazard H6)

| ID | Requirement | Threshold | Verifies |
|---|---|---|---|
| SR-6.1 | Applied joint torque shall be monitored against rated limits every cycle | `|τ_i| ≤ TAU_MAX[i]` | VP-6.1 |

### SG7 — Singularity avoidance (hazard H7)

| ID | Requirement | Threshold | Verifies |
|---|---|---|---|
| SR-7.1 | The monitor shall compute a manipulability measure and warn near singularities | `w = √det(J·Jᵀ) < w_min` | VP-7.1 |

### SG8 — Plan integrity (hazard H8)

| ID | Requirement | Threshold | Verifies |
|---|---|---|---|
| SR-8.1 | Trajectories containing NaN/Inf shall be rejected before execution | no NaN/Inf | VP-8.1 |
| SR-8.2 | Trajectory time vector shall be strictly increasing | monotonic t | VP-8.2 |

### SG9 — Reliable safe stop (hazard H9)

| ID | Requirement | Threshold | Verifies |
|---|---|---|---|
| SR-9.1 | On any hard violation, the monitor shall command a safe stop within one control cycle | latency ≤ 1 cycle | VP-9.1 |
| SR-9.2 | Once in safe stop, the system shall latch until explicitly reset | latched state | VP-9.2 |

## 4. Safety-function modes

The requirements resolve into three operating states:

| State | Entry condition | Robot behaviour |
|---|---|---|
| Normal | all requirements satisfied, `d ≥ S_p` | full commanded speed |
| Reduced speed | `S_min < d < S_p`, or a soft limit approached | speed scaled 0–1 |
| Safe stop | any hard limit violated, or `d ≤ S_min` | motion halted, latched |

## 5. Traceability summary

Every requirement traces backward to a hazard (via its safety goal) and forward
to a verification procedure (VP-x.x, defined in the V&V plan SAF-VNV-001). This
bidirectional traceability is the evidence base for the Safety Case (SAF-CASE-001).

```
Hazard (H1–H9) ─► Safety goal (SG1–SG9) ─► Requirement (SR-x.x) ─► Test (VP-x.x)
```
