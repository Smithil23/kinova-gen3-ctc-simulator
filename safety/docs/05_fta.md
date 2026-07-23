# Fault Tree Analysis (FTA)

**Document ID:** SAF-FTA-001
**Item:** Kinova Gen3 Collaborative Safety Layer (SAF-ITM-001)
**Top event:** Human injured by contact with the robot (hazard **H1**, safety goals
SG1/SG3)
**Method:** Deductive top-down fault tree with AND/OR gates, tracing the top event
to basic events.
**Status:** Concept demonstration.

---

## 1. Why this top event

H1 (excessive contact force / crushing) is rated **PLr e** in the HARA — the
highest required performance level, because it combines severe injury potential
(S2), continuous exposure in a shared workspace (F2), and little possibility of
avoidance (P2). It is therefore the correct top event for a fault tree.

---

## 2. Fault tree

```
                    ┌───────────────────────────────┐
                    │  TOP: Human injured by robot  │
                    │          contact              │
                    └───────────────┬───────────────┘
                                    │ AND
              ┌─────────────────────┴─────────────────────┐
              │                                           │
   ┌──────────┴───────────┐                   ┌───────────┴────────────┐
   │ G1: Human present in  │                  │ G2: Robot delivers      │
   │ the hazard zone       │                  │ harmful contact         │
   └──────────┬───────────┘                   └───────────┬────────────┘
              │ OR                                        │ AND
      ┌───────┴────────┐                     ┌────────────┴─────────────┐
      │                │                     │                          │
  ┌───┴────┐      ┌────┴─────┐     ┌─────────┴────────┐     ┌───────────┴──────────┐
  │ B1     │      │ B2       │     │ G3: Protective   │     │ G4: Energy at contact │
  │ Human  │      │ Human    │     │ stop not effective│     │ exceeds injury limit  │
  │ enters │      │ remains  │     └─────────┬────────┘     └───────────┬──────────┘
  │ zone   │      │ in zone  │               │ OR                       │ OR
  └────────┘      └──────────┘     ┌─────────┴─────────┐      ┌─────────┴─────────┐
                                   │                   │      │                   │
                          ┌────────┴──────┐   ┌────────┴───┐ ┌┴────────┐ ┌────────┴┐
                          │ G5: Human not │   │ B6 Stop    │ │ B7      │ │ B8      │
                          │ detected /    │   │ command    │ │ Speed   │ │ Torque  │
                          │ mis-located   │   │ not acted  │ │ too high│ │ too high│
                          └────────┬──────┘   │ on         │ │ at      │ │ (PFL    │
                                   │ OR       └────────────┘ │ contact │ │ failed) │
                        ┌──────────┴─────────┐               └─────────┘ └─────────┘
                        │                    │
                  ┌─────┴─────┐      ┌───────┴──────┐
                  │ B3        │      │ B4           │
                  │ Sensing   │      │ Position     │
                  │ dropout / │      │ error (frame │
                  │ occlusion │      │ / calib.)    │
                  └───────────┘      └──────────────┘
                                     ┌──────────────┐
                                     │ B5           │
                                     │ Monitor not  │
                                     │ executed     │
                                     └──────────────┘
```

---

## 3. Gate and event definitions

| Ref | Type | Description |
|---|---|---|
| TOP | Event | Human injured by contact with the robot |
| G1 | OR gate | Human is present in the hazard zone |
| G2 | AND gate | Robot delivers harmful contact (needs both G3 and G4) |
| G3 | OR gate | Protective stop is not effective |
| G4 | OR gate | Contact energy exceeds the injury threshold |
| G5 | OR gate | Human not detected or mis-located |
| B1 | Basic | Human enters the collaborative workspace |
| B2 | Basic | Human remains in the workspace during motion |
| B3 | Basic | Sensing dropout / occlusion (FMEA FM-01) |
| B4 | Basic | Position error — frame mismatch or calibration (FM-02, FM-03) |
| B5 | Basic | Monitor cycle not executed (FM-07) |
| B6 | Basic | Stop commanded but not acted upon (FM-05) |
| B7 | Basic | Robot speed too high at contact (SG2/SG3 failure) |
| B8 | Basic | Torque/force above injury limit — PFL failed (SG1, FM-10) |

---

## 4. Minimal cut sets

A **cut set** is a combination of basic events that together cause the top event.
The **minimal cut sets** here are of order 2–3, meaning at least two independent
failures must coincide:

| # | Cut set | Interpretation |
|---|---|---|
| 1 | B1/B2 · B3 · B7 | Human present, not detected, robot moving fast |
| 2 | B1/B2 · B3 · B8 | Human present, not detected, excessive force |
| 3 | B1/B2 · B4 · B7 | Human present, mis-located, robot moving fast |
| 4 | B1/B2 · B5 · B7 | Human present, monitor not running, robot moving fast |
| 5 | B1/B2 · B6 · B7 | Human present, stop ignored, robot moving fast |

**Key observation:** every cut set requires a **detection/execution failure**
(B3–B6) *combined with* an **energy failure** (B7 or B8). This is by design — the
architecture provides two independent protective mechanisms:

- **Speed & Separation Monitoring** (detect and stop before contact), and
- **Power & Force Limiting** (limit energy if contact occurs anyway).

Because the top event requires *both* to fail (AND gate at G2), the design has
defence in depth. A single failure does not cause injury.

---

## 5. Conclusions

**Strength:** the AND relationship between "protective stop ineffective" and
"energy exceeds injury limit" means SSM and PFL act as complementary layers. This
matches the ISO/TS 15066 intent that collaborative methods can be combined.

**Dominant weakness:** the **detection branch (G5)** appears in the majority of
cut sets, consistent with the FMEA finding that sensing integrity — not control
logic — is the principal vulnerability. Improving detection reliability
(redundant sensing, diagnostic coverage, timeout-to-safe-state) reduces more cut
sets than any change to the logic.

**Recommended actions:** implement the two open FMEA actions — missing-human
timeout to safe state (FM-01) and a monitor watchdog (FM-07) — since both cut
directly into the dominant branch.

> Quantitative probabilities are deliberately not assigned. Doing so credibly
> requires failure-rate data for rated hardware (per ISO 13849-1 / IEC 61508),
> which is outside the scope of this simulated concept demonstration.
