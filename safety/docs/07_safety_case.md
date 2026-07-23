# Safety Case

**Document ID:** SAF-CASE-001
**Item:** Kinova Gen3 Collaborative Safety Layer (SAF-ITM-001)
**Purpose:** Present a structured argument, supported by evidence, about the
safety of the demonstrated system — and state clearly what is *not* claimed.
**Status:** Concept demonstration.

---

## 1. What is claimed — and what is not

**Claimed.** A functional-safety lifecycle has been applied to a simulated 7-DOF
collaborative manipulator: hazards were identified and risk-rated, safety goals
and requirements were derived and allocated, protective functions were designed
and implemented as an independent monitor, and every requirement was verified in
simulation with full traceability.

**Not claimed.** This system is **not** certified, **not** compliant with ISO 13849
/ IEC 61508 / ISO 10218 in any assessable sense, and **not** fit for use with a
real robot or a real human. It uses no safety-rated hardware, no qualified tools,
no independent assessment, and illustrative rather than derived thresholds.

This distinction is stated first because a safety argument that overstates its
scope is itself a hazard: it invites reliance the evidence does not support.

---

## 2. The safety argument

**Claim C1 — The hazards of the item have been systematically identified.**
*Evidence:* SAF-HARA-001 identifies nine hazards (H1–H9) covering contact/crushing,
over-speed, insufficient separation, joint-range and torque violations,
singularity, invalid plans, and failure of the safe stop itself. Each is rated by
Severity / Frequency / Possibility and assigned a required Performance Level.
Hazards were derived from the item definition and the robot's actual rated limits.

**Claim C2 — Safety requirements are complete with respect to those hazards.**
*Evidence:* SAF-SRS-001 derives nine safety goals (SG1–SG9) from the nine hazards,
refined into 19 numbered requirements (SR-1.1 … SR-9.2), each with a measurable
threshold. Every hazard maps to at least one requirement; no requirement is
orphaned.

**Claim C3 — The requirements are correctly implemented.**
*Evidence:* `safety_monitor.py` implements each requirement as an explicit check
that reports the requirement ID it enforces. The implementation is an **independent
observer**: it reads the robot model and trajectory but modifies neither the
controller nor the plant, satisfying freedom from interference between the safety
and functional channels. Protective hardware limits are exposed read-only in the
user interface so they cannot be silently weakened.

**Claim C4 — The implementation has been verified.**
*Evidence:* SAF-VNV-001 records 18 verification procedures, one per requirement,
each using fault injection to confirm the function triggers when it should. All 18
passed. A scenario-level validation demonstrates the intended Normal → Reduced
speed → Safe stop progression with a simulated human approach, with only the
expected functions engaging.

**Claim C5 — Failure of the protective system itself has been analysed.**
*Evidence:* SAF-FMEA-001 analyses twelve failure modes of the safety layer, and
SAF-FTA-001 develops a fault tree for the top hazard. The fault tree shows that
injury requires **both** the protective stop to be ineffective **and** the contact
energy to exceed the injury limit — an AND relationship providing defence in
depth through two complementary functions (Speed & Separation Monitoring and
Power & Force Limiting).

**Claim C6 — Residual risks are identified and stated.**
*Evidence:* Both the FMEA and FTA converge on the same conclusion: the dominant
residual risk is **sensing and data integrity**, not control logic. Detection
failures (FM-01 to FM-04) carry the highest RPNs and appear in the majority of
minimal cut sets. Two mitigating actions — missing-human timeout to safe state,
and a monitor watchdog — are identified and remain **open**.

---

## 3. Evidence index

| Claim | Supporting document | Artefact |
|---|---|---|
| C1 | SAF-HARA-001 | Hazard table, PLr ratings |
| C2 | SAF-SRS-001 | 19 requirements, traceability matrix |
| C3 | Implementation | `safety_monitor.py`, `human_model.py` |
| C4 | SAF-VNV-001 | 18 verification procedures, all passed |
| C5 | SAF-FMEA-001, SAF-FTA-001 | 12 failure modes, fault tree, cut sets |
| C6 | SAF-FMEA-001 §3, SAF-FTA-001 §5 | Residual risk analysis, open actions |

---

## 4. Residual risk statement

Within the simulated environment, all specified safety requirements are met and
verified. The residual risk is concentrated in areas the environment cannot
exercise:

1. **Sensing integrity** — the human position is scripted, not sensed. A real
   system requires rated detection with diagnostic coverage.
2. **Timing under real load** — reaction and stopping times are assumed parameters,
   not measured system properties.
3. **Threshold derivation** — collaborative speed and force limits are illustrative;
   real values require biomechanical assessment per ISO/TS 15066 Annex A.
4. **Independence of assessment** — implementation and verification share an author.

---

## 5. Conclusion

The demonstrated safety layer provides a **coherent, traceable, and verified
safety argument within its stated scope**: a simulated collaborative manipulator
with a scripted human model. The lifecycle from hazard identification through
verification is complete and internally consistent, and the principal residual
risks are identified rather than hidden.

The system is suitable as a **concept demonstration and learning artefact**. It is
not suitable, and is not offered, as a basis for protecting a real person.
