# delta-hil

<p align="center">
  <img src="docs/hero.gif" width="760" alt="Two ABB IRB 360 deltas on a live TwinCAT PLC — velocity-matched conveyor tracking in Isaac Sim">
</p>

<p align="center"><em>Two ABB IRB 360 deltas driven by a real Beckhoff TwinCAT PLC — velocity-matched conveyor tracking in NVIDIA Isaac Sim (top-right: TCP X-velocity locked to the belt at each pick).</em></p>

**Hardware-in-the-loop pick-and-place.** Two ABB **IRB 360** deltas run a continuous
conveyor-tracking cell, closed around a **real Beckhoff TwinCAT PLC**. The idea worth
stealing: the **plant and the renderer are swappable behind one _unchanged_ controller** —
the same `cell_controller` and TwinCAT program drive **Isaac Sim** (RTX), a **GPU-free
browser** viewer, **MuJoCo** contact physics, and **Rapier** (WASM) physics.

📚 **[Interactive digital-twin course →](https://harshalaptraise.github.io/delta-hil/course/)** — a guided walkthrough of the ideas and the build.

## Quick start — laptop, no GPU, no PLC

**Install what you'll run** — each backend is its own extra (they combine, e.g. `".[web,mujoco,dev]"`):

| To run | Install |
|--------|---------|
| Browser viewer + **kinematic** plant (default) | `pip install -e ".[web]"` |
| …with **MuJoCo** physics (`--plant mujoco`) | `pip install -e ".[web,mujoco]"` |
| …with **Rapier** physics (`--plant rapier`) | `pip install -e ".[web]"` **plus** `node` on PATH (WASM is vendored — no npm) |
| The **test suite** (`pytest`) | add `dev` → `pip install -e ".[web,dev]"` |
| Drive any of them from a **live TwinCAT PLC** (`--plc`) | add `twincat` (`pyads`) → `".[web,twincat]"` |
| Full-fidelity **Isaac** render | Isaac Sim 5.1 (installed out-of-band) + an RTX GPU — see [Run on the rig](#run-on-the-rig--isaac--live-twincat) |

**Run** (installs the tests too, so the line below works as written):

```bash
pip install -e ".[web,dev]"
python -m pytest -q                          # 40 tests pass (+5 with the [mujoco] extra, +5 more with node → 50)
python scripts/run_web_cell.py --realbot     # → open http://127.0.0.1:8080
```

The browser shows the two real-CAD ABB deltas tracking the belt and dropping tortillas into
totes, with a live **TCP-vx-vs-belt overlay** and a **conservation ledger**. It's one script —
add a flag at a time; every combination is valid:

```bash
python scripts/run_web_cell.py                                       # stylized delta, mock controller (instant, no CAD)
python scripts/run_web_cell.py --realbot                             #   + the real ABB IRB 360 CAD
python scripts/run_web_cell.py --realbot --plant mujoco              #   + MuJoCo physics — needs the [mujoco] extra
python scripts/run_web_cell.py --realbot --plant rapier              #   + Rapier (WASM) physics — needs `node` on PATH
python scripts/run_web_cell.py --realbot --plant mujoco --native     #   + MuJoCo's own contact-debug window
python scripts/run_web_cell.py --realbot --plant mujoco --plc <AMS>  #   + driven by the LIVE TwinCAT PLC over ADS
```
Full-fidelity **Isaac** render (RTX GPU): `python scripts/run_twincat_cell.py mock`. Any `--plant`
works with or without `--realbot` / `--plc`.

> The controller is real, its program is unchanged from sim to bench, and the sim reads the
> controller's own clock — swapping the plant or the renderer never touches it.

---

## The cell

A real **Beckhoff TwinCAT** soft-PLC runs a continuous conveyor-tracking line: it tracks
streamed food items, splits them between an upstream and a downstream robot, and
**velocity-matches** every pick and place on the fly. The PLC commands a **velocity
feed-forward** (the tool's X is *slaved* to the conveyor speed, PickMaster-style — not a chased
position), which the plant integrates; the plant only executes, senses, and conserves.
Everything also runs **fully headless on a laptop** (mock PLC + mock plant) as the regression net.

## One controller, swappable plant

The whole point: **the plant changes, the controller does not.** `cell_controller` — and the
TwinCAT ST program it mirrors 1:1 — is **byte-identical** across every option below (`git diff`
proves it). Pick the plant that matches what you believe in:

- **`--plant kinematic`** *(default — the pure-kinematics path)* — no contact solver: the ideal
  servo integrates the feed-forward and the grasp is adjudicated by coincidence. Fastest and
  deterministic; for when you care about the **control** — tracking, velocity-match, conservation —
  not the dynamics.
- **`--plant mujoco`** — real **MuJoCo** contact physics: a friction-driven belt, a weld grasp,
  tortillas that physically pile in the totes. For those who prefer/trust MuJoCo.
- **`--plant rapier`** — the same cell in **Rapier** (Rust/WASM), the physics running in a node
  worker. For those siding with Rapier.

Two renderers sit behind the same snapshots: the **GPU-free browser** viewer (any plant,
`--realbot` for the real ABB CAD) and full-fidelity **Isaac Sim** (RTX). Swap the plant *or* the
renderer — the controller never notices.

---

## Run on the rig — Isaac + live TwinCAT

Quick start above is the whole **laptop** path (browser viewer, every `--plant`, `--realbot`, and
`--plc`, all air-gapped). The rungs below add the RTX GPU and, at the end, the real PLC.

### 1 · Isaac Sim render — full fidelity, still no PLC

Deterministic mock controller — the prettiest way to *see* the cell:

```bash
python scripts/run_twincat_cell.py mock         # the two-robot cell, deterministic
python scripts/run_twincat_cell.py mock 60      #   ...for 60 s of sim
```
Writes a video/GIF under `assets/render/`; it's the **same cell** the live PLC drives, so use it
to preview visuals without TwinCAT. (`cell_animation.py` / `animate_irb360.py` are the older
scripted and single-robot renders.)

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

### 3 · Rig + Isaac + live TwinCAT — the full HIL cell (max fidelity)

Load `docs/twincat_cell_program.md` (`GVL_Cell` + `FB_CellRobot` + `MAIN`) in TwinCAT,
build, **Activate (RUN)**, then:

```bash
python scripts/run_twincat_cell.py <AMS_NET_ID>        # the real PLC runs the whole line
python scripts/run_twincat_cell.py <AMS_NET_ID> 50     #   ...for 50 s of sim
```
The real PLC tracks the streamed food items, assigns robots, and commands every TCP +
**velocity feed-forward** + grip live; the sim derives its `dt` from the PLC's own
clock. Output: `assets/render/twincat_cell.mp4` (falls back to `.gif` if no H.264
encoder) with a top-right **TCP-vx-vs-belt overlay** (bold where velocity-locked, a
dot on the belt line at each grab) + a `…_velocity.csv` trace. Console prints the
clock source, **per-robot A/B picks**, the **velocity-lock** split (pick-track vs
place-track), the **ADS round-trip mean/jitter**, and the conservation ledger
(`picked / placed / passed / conserved`). Force `GVL_Cell.enable := FALSE` in a Watch
window to **freeze** the cell live; set `VPLOT = False` for a clean beauty render.

### Command table

| Command | Where | Needs | What you get |
|---|---|---|---|
| `python -m pytest -q` | laptop | `[dev]` (+ `[mujoco]` / `node` for all 50) | 40 tests pass; 50 with MuJoCo + node |
| `python -m deltahil.run` | laptop | nothing | mock HIL loop + eval-10 calibration self-score |
| **`python scripts/run_web_cell.py`** | laptop | `[web]` | **GPU-free browser viewer of the cell (stylized delta)** |
| `python scripts/run_web_cell.py --realbot` | laptop | `[web]` | …with the real ABB IRB 360 CAD (on-demand glTF) |
| **`python scripts/run_web_cell.py --plant mujoco`** | laptop | `[web,mujoco]` | **real MuJoCo contact physics (friction belt, weld grasp, piling)** |
| `python scripts/run_web_cell.py --plant mujoco --native` | laptop | `[web,mujoco]` | …also open MuJoCo's own contact-debug window |
| **`python scripts/run_web_cell.py --plant rapier`** | laptop | `[web]` + `node` | **Rapier (Rust/WASM) physics via a vendored node worker** |
| `python scripts/run_web_cell.py --plc <AMS>` | laptop | `[web]` + TwinCAT | browser viewer driven live by the PLC (any `--plant`) |
| `python scripts/run_twincat_cell.py mock [secs]` | rig | Isaac | deterministic two-robot cell → `twincat_cell.mp4/.gif` |
| `python scripts/cell_animation.py` | rig | Isaac | scripted two-robot cell → `cell_pick.gif` |
| `python scripts/animate_irb360.py` | rig | Isaac | one IRB 360 cycle → `irb360_pick.gif` |
| `python scripts/run_twincat_mock.py <AMS> [secs]` | rig | TwinCAT¹ | real PLC ↔ mock plant, FAST latency (no Isaac) |
| `python scripts/run_twincat_loop.py <AMS> [secs]` | rig | Isaac + TwinCAT¹ | real PLC drives kinematic Delta, latency/jitter |
| `python scripts/run_twincat_render.py <AMS> [frames]` | rig | Isaac + TwinCAT¹ | real PLC drives articulated IRB 360 → GIF |
| **`python scripts/run_twincat_cell.py <AMS> [secs]`** | rig | Isaac + TwinCAT² | **the full live cell → `twincat_cell.mp4`** |

¹ single-robot program (`docs/twincat_program.md`) · ² cell program (`docs/twincat_cell_program.md`)

---

## Prerequisites

The **browser viewer** (Quick start) needs none of the rig — just `pip install -e ".[web]"` on any
laptop (Python 3.10+, no GPU, no Windows). Everything below is only for the Isaac / live-TwinCAT rungs.

- **GPU workstation** — Windows + an NVIDIA **RTX** GPU (developed on an RTX 4090) — Isaac renders only.
- **NVIDIA Isaac Sim 5.1**, installed out-of-band; run scripts inside its Python
  environment (`isaacenv`). The scripts import clean on a laptop but only *run* the
  render/loop under Isaac.
- **Beckhoff TwinCAT 3** runtime (for the live rungs 2–3) with an **ADS route** to this
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

The cell adds three working invariants on top: **sampled-data honesty** (the sim
advances by the PLC's own `plc_time_ns` clock, one step per sample), a **bounded reach
envelope** (every command is clamped to the measured reach — no over-stretch), and
**velocity-slaved tracking** (the PLC commands a velocity feed-forward so the tracked
axis moves at the conveyor speed, not a one-sample-lagged chased position).

## Cell architecture

```
   sensors (parts/boxes, TCP, grip)  ->        <-  commands (TCP + velocity + grip)
  ┌────────────────┐   ADS sum-read/write   ┌────────────────────────────┐
  │  TwinCAT PLC   │◄──────────────────────►│  CellPlant (pure plant)    │
  │  FB_CellRobot  │   GVL_Cell (mm/LREAL)  │  integrates vel, adjudicates│
  │  x2 + MAIN     │                        │  grasp coincidence, ledger  │
  └────────────────┘                        └────────────────────────────┘
        ▲ plc_time_ns (the shared clock)              ▼ snapshots
        └──►  renderer (SWAPPABLE):  Isaac Sim (RTX)  |  Three.js/WebSocket (browser, no GPU)
```

- `plant/cell_plant.py` — the pure plant: streams food items + boxes, **integrates the
  commanded velocity feed-forward** (`tcp += vel·dt`) then trims to the position target,
  decides whether a grasp *coincided* in position **and** velocity, and conserves every
  part. It senses and actuates; it never decides control (P1/P2).
- `plc/cell_controller.py` — `MockCellController`, the golden reference the TwinCAT
  `FB_CellRobot` mirrors 1:1. Claim → track (X velocity-slaved to the belt) → grip →
  transfer → place; upstream robot splits, downstream is the catch-all; never abandons a
  pick.
- `plc/cell_link.py` — `CellAdsLink`: one ADS sum-write of sensors, one sum-read of
  commands (TCP + velocity feed-forward + grip) + the PLC clock (m ↔ mm at the seam;
  the velocity symbols are optional, so an older PLC still runs).
- `scripts/run_twincat_cell.py` — boots Isaac, runs the loop against the live PLC
  (or the mock), then renders. `cell_scene.py` (frozen) builds the USD; the render
  script dresses it (steel frame, belts, lighting) additively.
- `render/web/server.py` + `static/viewer.html` — the **web render seam**: runs the same
  plant + controller (or live TwinCAT) headless and streams ~30 Hz JSON snapshots to a
  Three.js viewer. `--realbot` articulates the real IRB 360 CAD via a JS port of
  `plant/irb360_pose.pose()` (`scripts/build_robot_glb.py` makes the glTF from the STEP).
- `plant/mujoco_cell_plant.py` — the **MuJoCo plant seam**: the SAME public
  contract as `cell_plant` (so `cell_controller` / `cell_link` / the ST program are unchanged),
  backed by real contact dynamics — a frictional slide-jointed belt (conveyor idiom), a weld
  grasp on the P3 gate, and tortillas that physically pile in the totes. `--plant mujoco`
  renders it in the browser; `--native` opens MuJoCo's own contact view (per-part quaternion is
  streamed so the browser shows the tumble).
- `plant/rapier_cell_plant.py` + `render/rapier/rapier_worker.mjs` — the **Rapier plant seam**:
  the SAME contract again, but the rigid-body world is Rapier (Rust/WASM). With no Python binding,
  the Python plant holds all the cell logic and drives a small **node worker** (one JSON line per
  step over stdio) that owns only the Rapier bodies — kinematic grippers/totes, dynamic tortillas
  on a frictionless belt + scripted carry, a fixed-joint weld on grasp. The WASM engine is vendored
  (`render/rapier/vendor`, ~3.9 MB) so it runs offline with just `node`.

## The seams (drop in real hardware)

Both sides sit behind `interfaces.py`; nothing upstream changes when you swap them.

| Seam | Light / mock (runs now) | Full / real |
|------|-----------------|------|
| Controller | `plc/mock_plc.py`, `plc/cell_controller.py` | TwinCAT via `plc/twincat_plc.py` / `plc/cell_link.py` (ADS); `plc/logix_plc.py` stub for ControlLogix |
| Plant | `plant/mock_plant.py`, `plant/cell_plant.py` (kinematic), **`mujoco_cell_plant.py` (MuJoCo)**, **`rapier_cell_plant.py` (Rapier)** | `plant/isaac_plant.py` — Isaac Sim |
| Render | `render/web/` — Three.js browser viewer, no GPU | Isaac Sim (`scripts/run_twincat_cell.py`, RTX) |

## Eval status

| Eval | Where | Status |
|------|-------|--------|
| 10 · calibration (P5, P3) | laptop / CI | **self-scored PASS** |
| 3 · grasp on pose∧velocity coincidence (P3) | laptop / rig | mock: **passed 0, conserved** |
| velocity-lock at pick (P3) | laptop / rig | **matched** — grab latches only at &#124;vx−belt&#124; < 0.015 m/s |
| 5 · <10 ms / σ<1 ms round-trip (P1, A) | your rig | **met** — cell ADS ≈ 2.0 ms, σ ≈ 0.25 ms |
| reach envelope never violated (cell) | laptop / rig | **0 violations** in mock + live |
| controller invariance (every plant + renderer) | laptop | **`git diff` empty** — `cell_controller`/`cell_link`/ST identical across all backends |
| web viewer runs GPU-free + offline | laptop | streams + renders (stylized & real CAD), no CDN |
| MuJoCo belt carries by friction (P3, plant) | laptop | **0.220 m/s** emergent — no one scripts the motion |
| MuJoCo grasp needs velocity coincidence (P3) | laptop | pose-only, zero-velocity contact **rejected** |
| MuJoCo places pile + conserve (P5) | laptop | tortillas stack (z 0.354 / 0.366), **conserved every step**, 0 reach |
| Rapier belt carries at belt speed (P3, plant) | laptop | **0.220 m/s** (frictionless slab + scripted carry, node worker) |
| Rapier places pile + conserve (P5) | laptop | tortillas land in totes (z ≈ 0.35), **conserved every step**, 0 reach |
| cross-backend agreement (kinematic ↔ MuJoCo ↔ Rapier) | laptop | same controller, placed counts within tolerance |

## Handoff

Keep `pytest` green as the regression net. The web, MuJoCo, and Rapier backends
show the point of the seam architecture: **both the renderer and the plant swap behind the
controller** — Isaac → a laptop browser, and the kinematic plant → real MuJoCo *or* Rapier contact
physics (two different engines, one seam) — with `cell_controller` and the ST program untouched.
The next step off the twin is the bench: the
**PLC program is unchanged**, physics and a real gripper replace the plant's grasp adjudication
(exactly what MuJoCo is a laptop-scale rehearsal of), and a hard EtherCAT fieldbus replaces the
polled ADS clock — the controller keeps its logic; only its
senses swap from silicon to steel.
