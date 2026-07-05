# delta-hil

Hardware-in-the-loop simulator for **challenging pick-and-place**: ABB **IRB 360
Delta** robots on **NVIDIA Isaac Sim**, closed around a **real industrial PLC**,
with honest pose calibration and deliberate fault injection.

The headline system is a **two-robot tortilla cell**: a real **Beckhoff TwinCAT**
soft-PLC runs a continuous conveyor-tracking line — it tracks streamed tortillas,
splits them between an upstream and a downstream robot, velocity-matches each pick
and place on the fly, and the simulation just executes, senses, and conserves. The
same code also runs **fully headless on a laptop** (mock PLC + mock plant, no GPU,
no controller), which is the regression net.

> The controller is real, its program is unchanged from sim to bench, and the sim
> reads the controller's own clock. See `docs/blog_digital_twin.md` for the story.

---

## Run it — easiest first, full cell last

Everything is additive and each rung is independently useful. Start at the top
(needs nothing) and work down as you add a GPU, then the PLC.

### 0 · Laptop — no GPU, no PLC

```bash
pytest -q                      # 39 tests: plant, controller, calibration evals
python -m deltahil.run         # single-robot mock HIL loop + self-scored calibration
```
`pytest` proves the whole control/plant/calibration stack. `deltahil.run` closes a
mock loop and self-scores **eval 10 (calibration) → PASS** — no hardware at all.

### 1 · Rig — Isaac Sim only, still no PLC

Renders (deterministic mock controller) — the fastest way to *see* the cell:

```bash
python scripts/run_twincat_cell.py mock         # the two-robot cell, deterministic
python scripts/run_twincat_cell.py mock 60      #   ...for 60 s of sim
python scripts/cell_animation.py                # scripted two-robot cell
python scripts/animate_irb360.py                # one IRB 360 pick-and-place (kinematics check)
```
Each writes a video/GIF under `assets/render/`. `run_twincat_cell.py mock` produces
the **same cell** the live PLC drives — use it to preview visuals without TwinCAT.

### 2 · Rig + live TwinCAT — single-robot program

Load `docs/twincat_program.md` (GVLs + `MAIN`) in TwinCAT, build, **Activate (RUN)**,
then pass the target's **AMS NetId**:

```bash
python scripts/run_twincat_mock.py  <AMS_NET_ID> [secs]   # PLC ↔ mock plant, NO Isaac (pure pyads)
python scripts/run_twincat_loop.py  <AMS_NET_ID> [secs]   # PLC drives the Isaac kinematic Delta
python scripts/run_twincat_render.py <AMS_NET_ID> [frames] # PLC drives the articulated IRB 360 → GIF
```
`run_twincat_mock` is the quickest proof the real controller closes the loop (no GPU
boot). All three report the **FAST-tier round-trip latency/jitter** (eval-5 home).

### 3 · Rig + Isaac + live TwinCAT — the full HIL cell  ⭐

Load `docs/twincat_cell_program.md` (`GVL_Cell` + `FB_CellRobot` + `MAIN`) in TwinCAT,
build, **Activate (RUN)**, then:

```bash
python scripts/run_twincat_cell.py <AMS_NET_ID>        # the real PLC runs the whole line
python scripts/run_twincat_cell.py <AMS_NET_ID> 50     #   ...for 50 s of sim
```
The real PLC tracks the streamed tortillas, assigns robots, and commands every TCP +
grip live; the sim derives its `dt` from the PLC's own clock. Output:
`assets/render/twincat_cell.mp4` (falls back to `.gif` if no H.264 encoder). Console
prints the clock source, **per-robot A/B picks**, the **ADS round-trip mean/jitter**,
and the conservation ledger (`picked / placed / passed / conserved`).
Force `GVL_Cell.enable := FALSE` in a Watch window to **freeze** the cell live.

### Command table

| Command | Where | Needs | What you get |
|---|---|---|---|
| `pytest -q` | laptop | nothing | 39 tests pass |
| `python -m deltahil.run` | laptop | nothing | mock HIL loop + eval-10 calibration self-score |
| `python scripts/run_twincat_cell.py mock [secs]` | rig | Isaac | deterministic two-robot cell → `twincat_cell.mp4/.gif` |
| `python scripts/cell_animation.py` | rig | Isaac | scripted two-robot cell → `cell_pick.gif` |
| `python scripts/animate_irb360.py` | rig | Isaac | one IRB 360 cycle → `irb360_pick.gif` |
| `python scripts/run_twincat_mock.py <AMS> [secs]` | rig | TwinCAT¹ | real PLC ↔ mock plant, FAST latency (no Isaac) |
| `python scripts/run_twincat_loop.py <AMS> [secs]` | rig | Isaac + TwinCAT¹ | real PLC drives kinematic Delta, latency/jitter |
| `python scripts/run_twincat_render.py <AMS> [frames]` | rig | Isaac + TwinCAT¹ | real PLC drives articulated IRB 360 → GIF |
| **`python scripts/run_twincat_cell.py <AMS> [secs]`** | rig | Isaac + TwinCAT² | **the full live cell → `twincat_cell.mp4`** |

¹ single-robot program (`docs/twincat_program.md`) · ² cell program (`docs/twincat_cell_program.md`)

---

## Rig prerequisites

- **GPU workstation** — Windows + an NVIDIA **RTX** GPU (developed on an RTX 4090).
- **NVIDIA Isaac Sim 5.1**, installed out-of-band; run scripts inside its Python
  environment (`isaacenv`). The scripts import clean on a laptop but only *run* the
  render/loop under Isaac.
- **Beckhoff TwinCAT 3** runtime (for tiers 2–3) with an **ADS route** to this
  machine, and the relevant PLC program from `docs/` loaded, built, and **activated
  (RUN)**. The ST programs live in `docs/*.md` — paste each POU into the matching
  TwinCAT pane (mind the `FUNCTION_BLOCK`/`PROGRAM` headers).
- **Python 3.10+** with this package installed editable:
  `pip install -e ".[twincat]"` (adds `pyads` for ADS). `numpy` is core; `Pillow`
  for GIF output; an **ffmpeg-capable `imageio`** (`imageio-ffmpeg`) for the HD MP4 —
  without it, the cell render falls back to a downscaled GIF.
- **USD assets** in `assets/` (the IRB 360 and the cell scene).
- The TwinCAT target's **AMS NetId** (e.g. `5.1.204.123.1.1`) for the live scripts.

---

## The constitution (fixed)

Every module cites these by number; see `src/deltahil/constitution.py`.

| # | Principle |
|---|---|
| P1 | Real-time closed loop — the PLC free-runs on its own oscillator; added latency is physically real |
| P2 | I/O contract is the only channel — the PLC acts solely on its sampled tag map |
| P3 | A pick is a physical coincidence — succeeds iff pose < tol **and** grip in window (the cell adds velocity-match for tracking) |
| P4 | Parallel closed-chain kinematics — the Delta's loop closure (PhysX guide joints) |
| P5 | Calibration corrects bias, not variance — it drives the identifiable bias down, noise floors reliability |
| P6 | Reproducibility is bounded by the live loop — no bit-exact replay; evals are statistical |
| P7 | HIL value is conditional — worth it only if the program under test is real; real faults, $0 core plant |
| A  | Two-tier I/O — FAST (EtherCAT/EtherNet-IP) under the eval-5 jitter bound; SLOW (OPC UA) supervisory |

The cell adds two working invariants on top: **sampled-data honesty** (the sim
advances by the PLC's own `plc_time_ns` clock, one step per sample) and a **bounded
reach envelope** (every command is clamped to the measured reach — no over-stretch).

## Cell architecture

```
   sensors (parts/totes, TCP, grip)  ->            <-  commands (TCP + grip)
  ┌────────────────┐   ADS sum-read/write   ┌────────────────────────────┐
  │  TwinCAT PLC   │◄──────────────────────►│  CellPlant (pure plant)    │
  │  FB_CellRobot  │   GVL_Cell (mm/LREAL)  │  streams belts, adjudicates │
  │  x2 + MAIN     │                        │  grasp coincidence, ledger  │
  └────────────────┘                        └────────────────────────────┘
        ▲ plc_time_ns (the shared clock)              ▼ snapshots
        └───────────────────────────────►  Isaac Sim render (IRB 360 x2)
```

- `plant/cell_plant.py` — the pure plant: streams tortillas + totes, executes the
  commanded TCPs, decides whether a grasp *coincided* in position **and** velocity,
  and conserves every part. It senses and actuates; it never decides control (P1/P2).
- `plc/cell_controller.py` — `MockCellController`, the golden reference the TwinCAT
  `FB_CellRobot` mirrors 1:1. Claim → track (velocity-matched) → grip → transfer →
  place; upstream robot splits, downstream is the catch-all; never abandons a pick.
- `plc/cell_link.py` — `CellAdsLink`: one ADS sum-write of sensors, one sum-read of
  commands + the PLC clock (m ↔ mm at the seam).
- `scripts/run_twincat_cell.py` — boots Isaac, runs the loop against the live PLC
  (or the mock), then renders. `cell_scene.py` (frozen) builds the USD; the render
  script dresses it (steel frame, belts, lighting) additively.

## The seams (drop in real hardware)

Both sides sit behind `interfaces.py`; nothing upstream changes when you swap them.

| Seam | Mock (runs now) | Real |
|------|-----------------|------|
| Controller | `plc/mock_plc.py`, `plc/cell_controller.py` | TwinCAT via `plc/twincat_plc.py` / `plc/cell_link.py` (ADS); `plc/logix_plc.py` stub for ControlLogix |
| Plant | `plant/mock_plant.py`, `plant/cell_plant.py` | `plant/isaac_plant.py` — Isaac Sim |

## Eval status

| Eval | Where | Status |
|------|-------|--------|
| 10 · calibration (P5, P3) | laptop / CI | **self-scored PASS** |
| 3 · grasp on pose∧velocity coincidence (P3) | laptop / rig | mock: **passed 0, conserved** |
| 5 · <10 ms / σ<1 ms round-trip (P1, A) | your rig | **met** — cell ADS ≈ 2.0 ms, σ ≈ 0.25 ms |
| reach envelope never violated (cell) | laptop / rig | **0 violations** in mock + live |

## Handoff

Keep `pytest` green as the regression net. The next step off the twin is the bench:
the **PLC program is unchanged**, physics and a real gripper replace the plant's
grasp adjudication, and a hard EtherCAT fieldbus replaces the polled ADS clock. See
`docs/blog_digital_twin.md` ("Taking it to the bench") for what stays, drops, and
arrives.
