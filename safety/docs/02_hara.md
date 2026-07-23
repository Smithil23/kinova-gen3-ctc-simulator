# Hazard Analysis and Risk Assessment (HARA)

**Document ID:** SAF-HARA-001
**Item:** Kinova Gen3 Collaborative Safety Layer (see SAF-ITM-001)
**Method:** ISO 12100 hazard identification + risk rating by Severity / Exposure /
Controllability, yielding safety goals. Required Performance Level (PLr) is assigned
per ISO 13849-1 risk graph.
**Status:** Concept demonstration.

---

## 1. Risk-rating scheme

Following the ISO 13849-1 risk-graph parameters:

**Severity of injury (S)**
- S1 — slight, normally reversible injury
- S2 — serious, normally irreversible injury, or death

**Frequency/exposure to hazard (F)**
- F1 — seldom to less often / short exposure
- F2 — frequent to continuous / long exposure

**Possibility of avoiding the hazard (P)**
- P1 — possible under specific conditions
- P2 — scarcely possible

The combination maps to a **Required Performance Level (PLr)** from **a** (lowest)
to **e** (highest) via the ISO 13849-1 risk graph.

---

## 2. Hazard analysis table

| ID | Hazard / hazardous event | Cause | S | F | P | PLr | Safety goal |
|---|---|---|---|---|---|---|---|
| **H1** | Excessive contact force / crushing of human by moving arm | Arm follows trajectory through space a human occupies; no force limiting | S2 | F2 | P2 | **e** | Limit contact force/energy to a non-injurious level (PFL) |
| **H2** | Impact from high TCP speed during collaboration | Commanded Cartesian speed exceeds safe collaborative speed near a human | S2 | F2 | P1 | **d** | Limit TCP speed when a human is in the collaborative zone |
| **H3** | Collision due to insufficient separation | Robot continues at speed as human approaches; no speed/separation monitoring | S2 | F2 | P2 | **e** | Maintain protective separation distance; slow/stop as human approaches (SSM) |
| **H4** | Joint driven beyond mechanical position limit | Planner/IK commands a configuration outside `Q_LIM`; controller tracks it | S2 | F1 | P2 | **d** | Prevent motion beyond validated joint position limits |
| **H5** | Joint over-speed | Trajectory commands joint velocity above `QD_MAX` | S1 | F2 | P2 | **c** | Prevent joint velocity exceeding rated limits |
| **H6** | Actuator over-torque / unstable motion | CTC output exceeds `TAU_MAX`; saturation or instability | S2 | F1 | P2 | **d** | Keep joint torque within rated limits; detect saturation |
| **H7** | Unexpected/uncontrolled motion near singularity | IK near kinematic singularity produces large joint rates | S2 | F1 | P2 | **d** | Detect near-singular configurations and inhibit motion |
| **H8** | Motion executed despite invalid plan | Trajectory contains NaN/Inf, non-monotonic time, or IK failure | S2 | F1 | P1 | **c** | Validate plan integrity before execution; block invalid plans |
| **H9** | Failure to reach a safe state on demand | Violation detected but robot does not stop | S2 | F2 | P2 | **e** | Guarantee transition to safe stop within one control cycle of a violation |

---

## 3. Safety goals summary

Each hazard yields a top-level **safety goal** — the highest-level safety
requirement. These are refined into concrete, testable requirements in the Safety
Requirements Specification (SAF-SRS-001).

| Safety goal | Addresses | Realised by (safety function) |
|---|---|---|
| SG1 — Power & Force Limiting | H1, H6 | Torque cap to injury-threshold-derived limit |
| SG2 — Safe collaborative speed | H2 | TCP speed limit via Jacobian |
| SG3 — Speed & Separation Monitoring | H3 | Distance-based speed scaling / stop |
| SG4 — Joint range limiting | H4 | Position-limit monitor |
| SG5 — Joint speed limiting | H5 | Velocity-limit monitor |
| SG6 — Torque supervision | H6 | Torque-limit monitor |
| SG7 — Singularity avoidance | H7 | Manipulability / Jacobian-condition monitor |
| SG8 — Plan integrity | H8 | Pre-flight trajectory validation |
| SG9 — Reliable safe stop | H9 | Safe-state machine |

---

## 4. Notes on rating rationale

- **H1 and H3 are rated PLr e** because they combine the most severe outcome
  (S2), continuous exposure in a shared workspace (F2), and little possibility of
  avoidance (P2) — the defining risk of collaborative operation, and the reason
  ISO/TS 15066 exists.
- **H9 is rated PLr e** because a safety layer that fails to act on demand
  undermines every other function; the safe-stop path is the most safety-critical
  element.
- Position/torque hazards (H4, H6) are rated with **F1** because they arise from
  specific planner/IK conditions rather than continuous exposure, but remain S2.

> The S/F/P ratings and resulting PLr values are illustrative and reflect
> engineering judgement for a concept demonstration. A certified assessment would
> require a formal, reviewed risk analysis with biomechanical force thresholds per
> ISO/TS 15066 Annex A.
