# Failure Mode and Effects Analysis (FMEA)

**Document ID:** SAF-FMEA-001
**Item:** Kinova Gen3 Collaborative Safety Layer (SAF-ITM-001)
**Scope:** Failure modes of the **safety-related control system** — the monitor,
its inputs, and its safe-state path — and their effect on the safety goals.
**Method:** Design FMEA with Severity / Occurrence / Detection ratings (1–10) and
Risk Priority Number (RPN = S × O × D).
**Status:** Concept demonstration.

> **Why analyse the safety layer itself?** The HARA (SAF-HARA-001) analysed hazards
> of the *robot*. This FMEA analyses how the *protective system* could fail. A
> safety function that fails silently is more dangerous than no safety function at
> all, because operators rely on it. This corresponds to the "freedom from
> interference" and diagnostic-coverage concerns in ISO 13849-1 / IEC 61508.

---

## 1. Rating scales

| Rating | Severity (S) | Occurrence (O) | Detection (D) |
|---|---|---|---|
| 1–3 | Minor / no safety effect | Unlikely | Almost certainly detected |
| 4–6 | Degraded protection | Occasional | Likely detected |
| 7–8 | Loss of a safety function | Frequent | Poorly detected |
| 9–10 | Loss of protection, injury possible | Very frequent | Undetectable |

RPN = S × O × D. Items with **RPN ≥ 100** or **S ≥ 9** require mitigation.

---

## 2. FMEA table

| ID | Function / element | Failure mode | Cause | Effect on safety goal | S | O | D | RPN | Mitigation |
|---|---|---|---|---|---|---|---|---|---|
| FM-01 | Human position input | Human not detected (missing/late data) | Sensor dropout; scripted model ends; occlusion | SSM (SG3) blind — no slow-down or stop; collision possible | 10 | 4 | 7 | **280** | Treat missing human data as worst case (assume closest credible position); timeout → safe stop |
| FM-02 | Human position input | Position reported farther than actual | Sensor error; calibration offset | Protective distance under-estimated; robot runs too fast near human | 10 | 3 | 8 | **240** | Add uncertainty margin (C + Z) to S_p; plausibility-check against previous sample |
| FM-03 | Separation computation | Wrong distance computed | Frame/coordinate mismatch between robot and human data | SSM decision based on wrong geometry | 9 | 3 | 6 | **162** | Single source of truth for frames; unit tests on known geometry |
| FM-04 | TCP velocity computation | Velocity under-estimated | Jacobian evaluated at stale configuration; wrong q̇ | S_p too small; insufficient stopping margin | 8 | 3 | 6 | **144** | Compute J and q̇ at the same timestep; verification test VP-2.1 |
| FM-05 | Safe-stop path | Stop commanded but not executed | Latch not applied; downstream ignores speed scale | Violation detected but robot keeps moving (SG9 lost) | 10 | 2 | 5 | **100** | Latching state machine (SR-9.2); verified by VP-9.1/9.2 |
| FM-06 | Safe-stop path | Spurious stop (false positive) | Noisy input; threshold too tight | Availability loss, no safety loss; may tempt users to disable | 4 | 5 | 2 | 40 | Hysteresis / proportional reduction band rather than binary stop |
| FM-07 | Monitor execution | Monitor not executed / cycle skipped | Thread starvation; exception in evaluation | No supervision during that period | 10 | 2 | 7 | **140** | Exception handling defaults to safe state; watchdog on monitor cycle |
| FM-08 | Limit parameters | Protective limit weakened | Hardware limit edited via UI or config | All limit-based functions degraded | 9 | 2 | 4 | 72 | Hardware limits are read-only in the GUI (implemented); parameters logged per run |
| FM-09 | Plan validation | Invalid trajectory accepted | NaN/Inf or non-monotonic time not detected | Undefined motion; SG8 lost | 8 | 2 | 3 | 48 | Pre-flight validation (SR-8.1/8.2); verified by VP-8.1/8.2 |
| FM-10 | Torque monitoring | Saturation not flagged | Comparison tolerance too loose | Excessive contact force undetected (SG1) | 9 | 3 | 5 | **135** | Explicit saturation check (SR-1.3); collaborative cap at 50% rated |
| FM-11 | Singularity detection | Near-singularity missed | Manipulability threshold too low | Large joint rates unflagged (SG7) | 7 | 4 | 6 | **168** | Tune w_min; warn early rather than at the limit |
| FM-12 | State reporting | Status shown as safe while violated | GUI update lag; state not propagated | Operator misled about protection status | 8 | 2 | 4 | 64 | State derived from monitor output only; no independent GUI state |

---

## 3. Highest-risk items

Ranked by RPN, the dominant risks are **input-related, not logic-related**:

1. **FM-01 (280)** — human not detected. The SSM function is only as good as its
   knowledge of where the human is. This is the single largest risk.
2. **FM-02 (240)** — human position over-estimated (reported too far).
3. **FM-11 (168)** — singularity missed.
4. **FM-03 (162)** — coordinate-frame mismatch.

**Conclusion:** the safety logic is comparatively easy to verify; the vulnerability
lies in **sensing and data integrity**. In a real deployment this is exactly why
ISO 13849 demands rated sensors with diagnostic coverage, and why ISO/TS 15066
includes uncertainty terms (C, Z_d, Z_r) in the protective-distance formula. The
concept demonstration mitigates these only by assumption, which is stated openly.

---

## 4. Actions carried into the design

| Action | Source | Status |
|---|---|---|
| Latching safe stop | FM-05 | Implemented (`SafetyMonitor._latched`) |
| Read-only hardware limits in GUI | FM-08 | Implemented |
| Uncertainty margin in S_p | FM-02 | Implemented (`margin` parameter) |
| Proportional speed reduction (not binary) | FM-06 | Implemented |
| Pre-flight plan validation | FM-09 | Implemented (`validate_plan`) |
| Missing-human timeout → safe state | FM-01 | **Open** — recommended for future work |
| Watchdog on monitor cycle | FM-07 | **Open** — recommended for future work |
