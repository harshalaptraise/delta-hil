# delta-hil

Hardware-in-the-loop simulator for **challenging pick-and-place**: an ABB **Delta**
robot on **Isaac Sim**, closed around a real **Allen-Bradley ControlLogix 1756-L8x**
controller, with deliberate fault injection and honest pose calibration.

It runs **fully headless today** on a mock PLC + mock plant — the loop closes, the
challenge degrades it, and the calibration eval self-scores — with no GPU and no
controller. The real controller and Isaac Sim drop in behind two interfaces.

```
python -m deltahil.run     # closed-loop demo + self-score
pytest                     # 10 tests, incl. eval 10 (calibration)
```

## The constitution (fixed)

Every module cites these by number; see `src/deltahil/constitution.py`.

| # | Principle |
|---|---|
| P1 | Real-time closed loop — PLC free-runs; added latency is physically real |
| P2 | I/O contract is the only channel — control acts solely on the tag map |
| P3 | A pick is a physical coincidence — pose ∧ timing ∧ force, jointly |
| P4 | Parallel closed-chain kinematics — Delta loop closure (PhysX guide joints) |
| P5 | Calibration corrects bias, not variance — noise floors reliability |
| P6 | Reproducibility bounded by the live loop — evals are statistical |
| P7 | HIL value is conditional — real program, real faults, $0 core |
| A  | Two-tier I/O — FAST (EtherCAT/EtherNet-IP) under the jitter bound; SLOW (OPC UA) exempt |

## Architecture

```
        actuator commands ->                 <- sensor states
  ┌──────────┐   fast    ┌──────────────┐   fast    ┌──────────────┐
  │ PLC L8x  │◄─────────►│  HIL bridge  │◄─────────►│  Isaac plant │
  │  (DUT)   │   slow    │ fast · slow  │           │ Delta+contact│
  └──────────┘           └──────────────┘           └──────────────┘
       ▲                        ▲                          ▲
  ┌──────────┐            ┌──────────────┐           ┌──────────────┐
  │Telemetry │            │ Calibration  │           │Fault inject  │
  │taps+score│            │ removes bias │           │ jams,misfeed │
  └──────────┘            └──────────────┘           └──────────────┘
```

- `bridge.py` — one scan = one turn of the loop; the only writer across the seam
  (P2 holds by construction). Meters fast-tier latency/jitter (home for eval 5).
- `calibration.py` — Kabsch point registration (the desk-verifiable form of
  AX=XB). Removes the identifiable frame bias, **reports the residual noise floor
  instead of hiding it** (P5).
- `scenario.py` — splits every disturbance into *systematic* (removable) vs
  *stochastic* (not). The stochastic faults are the "challenge."
- `evals.py` / `telemetry.py` — eval 10 + the closed-loop scorecard.

## The seams (drop in real hardware here)

Both sides sit behind `interfaces.py`; nothing upstream changes when you swap them.

| Seam | Mock (runs now) | Real (stub + checklist) |
|------|-----------------|--------------------------|
| Controller | `plc/mock_plc.py` | `plc/logix_plc.py` — pycomm3 fast tier + OPC UA slow tier |
| Plant | `plant/mock_plant.py` | `plant/isaac_plant.py` — PhysX Delta, guide-joint rig |

Install the extras when you wire them up: `pip install -e ".[logix]"` /
`".[isaac]"`. Each stub's docstring lists the exact integration steps and which
eval gates them.

## Eval status

| Eval | Where | Status |
|------|-------|--------|
| 10 · calibration (P5, P3) | desk / CI | **self-scored PASS** here |
| 1 · 0.5 mm IK error (P3, P4) | your rig | rig-verifiable — after the PhysX Delta rig |
| 5 · <10 ms / σ<1 ms, fast tier (P1, A) | your rig | rig-verifiable — soak test on real PLC+bridge |
| 3 / 9 · RTF≥1.0, ≥30 FPS (P4, P7) | your GPU | rig-verifiable — on your Isaac + RTX GPU |

## Handoff

Open this repo in Claude Code and continue from the seams. The headless path is
your regression net: keep `pytest` green while you build out `isaac_plant.py` and
`logix_plc.py`. The RobotStudio seat is the offline validation oracle for motion
profiles (eval 4b) — a validator, not a runtime dependency.
