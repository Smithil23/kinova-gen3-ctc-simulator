# Verification and Validation Report

**Document ID:** SAF-VNV-001
**Item:** Kinova Gen3 Collaborative Safety Layer (SAF-ITM-001)
**Verifies:** Safety Requirements Specification SAF-SRS-001 (SR-1.1 … SR-9.2)
**Method:** Requirement-based testing. Each verification procedure (VP-x.x) targets
one requirement, defines a stimulus and an expected safety response, and records
the observed result.
**Status:** Concept demonstration — verification in simulation only.

---

## 1. Verification strategy

Two complementary levels:

**Unit-level (fault injection).** Each safety function is stimulated with a
deliberately unsafe input and the monitor's decision is checked. This proves the
function triggers when it should, which cannot be shown by nominal runs alone.

**Scenario-level (integration).** A complete trajectory is executed with a
simulated human approaching, exercising all functions together and demonstrating
the intended Normal → Reduced → Safe stop progression.

> **Testing principle applied:** a safety function that has never been observed to
> trigger is unverified. Nominal ("everything fine") runs demonstrate absence of
> false positives, not presence of protection. Both are required.

---

## 2. Verification results

| VP | Verifies | Stimulus | Expected result | Observed | Status |
|---|---|---|---|---|---|
| VP-1.1 | SR-1.1 | Joint torque set above `TAU_MAX` | Violation SR-1.1, safe stop | Violation raised, state = Safe stop | **Pass** |
| VP-1.2 | SR-1.2 | Torque between collaborative cap and rated limit | Warning SR-1.2, reduced speed | Warning raised, state = Reduced | **Pass** |
| VP-1.3 | SR-1.3 | Torque at saturation | Saturation flagged | Warning SR-1.3 raised | **Pass** |
| VP-2.1 | SR-2.1 | TCP speed above `v_safe` | Speed scaled to satisfy limit | Warning SR-2.1, scale reduced | **Pass** |
| VP-3.1 | SR-3.1 | Human present | Separation computed each cycle | Separation reported every sample | **Pass** |
| VP-3.2 | SR-3.2 | Varying robot/human speeds | S_p recomputed dynamically | S_p varies with speed as per formula | **Pass** |
| VP-3.3 | SR-3.3 | Human far (d ≥ S_p) | Full speed permitted | State = Normal, scale = 1.0 | **Pass** |
| VP-3.4 | SR-3.4 | Human between S_min and S_p | Proportional speed reduction | State = Reduced, 0 < scale < 1 | **Pass** |
| VP-3.5 | SR-3.5 | Human at/below S_min | Safe stop | State = Safe stop, scale = 0 | **Pass** |
| VP-4.1 | SR-4.1 | Joint outside `Q_LIM` | Violation SR-4.1 | Violation raised | **Pass** |
| VP-4.2 | SR-4.2 | Plan with out-of-range joint | Plan rejected pre-execution | `validate_plan` returned not-ok | **Pass** |
| VP-5.1 | SR-5.1 | Joint velocity above `QD_MAX` | Violation SR-5.1, safe stop | Violation raised, state = Safe stop | **Pass** |
| VP-6.1 | SR-6.1 | Torque monitored each cycle | Evaluation every sample | Checked every sample | **Pass** |
| VP-7.1 | SR-7.1 | Near-singular configuration | Manipulability warning | Warning SR-7.1 raised | **Pass** |
| VP-8.1 | SR-8.1 | Trajectory containing NaN | Plan rejected | Rejected, issue reported | **Pass** |
| VP-8.2 | SR-8.2 | Non-monotonic time vector | Plan rejected | Rejected, SR-8.2 issue reported | **Pass** |
| VP-9.1 | SR-9.1 | Hard violation injected | Safe stop within one cycle | Stop asserted on same sample | **Pass** |
| VP-9.2 | SR-9.2 | Safe sample after a violation | State remains latched | Remained Safe stop until reset | **Pass** |

**Result: 18 of 18 verification procedures passed.**

---

## 3. Scenario validation

**Scenario:** cubic trajectory to a Cartesian target with a simulated human
approaching from ~2 m at 1.6 m/s.

| Observation | Expected | Result |
|---|---|---|
| Initial phase, human distant | Normal, full speed | Normal, scale = 1.0 |
| Separation falls below S_p | Progressive speed reduction | Scale ramped down proportionally |
| Separation reaches S_min | Safe stop | State = Safe stop, scale = 0 |
| After stop | State latched | Remained stopped |
| Functions engaged | SSM and Cartesian speed only | Exactly those two flagged; others green |

The recorded separation, protective-distance and speed-factor traces show the
intended three-phase behaviour, and the per-function status matched the analysis
(no spurious triggers on position, torque or plan integrity).

---

## 4. Traceability

Complete bidirectional traceability is established:

```
Hazard (H1–H9) → Safety goal (SG1–SG9) → Requirement (SR-x.x) → Test (VP-x.x) → Result
```

Every requirement in SAF-SRS-001 has exactly one verification procedure, and every
verification procedure passed. No requirement is unverified; no test is orphaned.

---

## 5. Limitations of this verification

Stated explicitly, because verification claims must be bounded:

- **Simulation only.** No physical robot, no rated sensors, no real human. Nothing
  here verifies hardware behaviour, timing under real load, or sensor reliability.
- **Scripted human.** The SSM input is a defined trajectory, not sensed data;
  detection failures (FMEA FM-01 to FM-04, the dominant risks) are therefore
  **not** verified by these tests.
- **Illustrative thresholds.** Values follow ISO/TS 15066 guidance but are not
  derived from a certified biomechanical assessment.
- **No independent assessment.** Tests were written by the same author as the
  implementation; there is no independent verification body.
- **Open actions outstanding.** The missing-human timeout (FM-01) and monitor
  watchdog (FM-07) are recommended but not implemented, and hence not verified.

**Conclusion.** The safety *logic* is verified against every specified requirement
in simulation. The safety *system as a whole* is not qualified for real use — the
dominant residual risks lie in sensing integrity, which this environment cannot
exercise.
