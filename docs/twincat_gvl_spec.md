# TwinCAT ↔ delta-hil — GVL spec & bridge setup

The `TwinCATLink` (`src/deltahil/plc/twincat_plc.py`) exchanges the tag map
(`src/deltahil/tags.py`) with a TwinCAT runtime through the **Loupe Omniverse
Beckhoff Bridge** (ADS). This doc is the contract the TwinCAT side must expose.

## 1. Global Variable Lists (declare these in TwinCAT)

The harness tag `<ns>.<leaf>` maps to symbol `GVL_<Ns>.<leaf>`. Declare three GVLs.
Types: `vec3` = `ARRAY[0..2] OF LREAL` (x,y,z in **mm**), and mm/base-frame throughout.

```iecst
// GVL_Cmd  — PLC → plant (the controller's outputs), FAST tier
VAR_GLOBAL
    target_xyz : ARRAY[0..2] OF LREAL;   // commanded pick point (mm)
    grip       : BOOL;                    // gripper close request
    tracking   : BOOL;                    // conveyor-tracking enabled
END_VAR

// GVL_Sensor  — plant → PLC (the sim's feedback), FAST tier
VAR_GLOBAL
    part_present : BOOL;                  // part detected at pick window
    grip_confirm : BOOL;                  // grasp achieved (P3 coincidence)
    tcp_xyz      : ARRAY[0..2] OF LREAL;  // tool-centre-point pose (mm)
END_VAR

// GVL_Sup  — supervisory, SLOW tier (exempt from the eval-5 jitter bound)
VAR_GLOBAL
    mode             : DINT;                  // 0=idle 1=run 2=calibrate   (PLC→plant)
    calib_offset_xyz : ARRAY[0..2] OF LREAL;  // vision→base correction     (PLC→plant)
    cycle_count      : DINT;                  // completed pick cycles       (plant→PLC)
    last_residual_mm : LREAL;                 // reported calibration residual (plant→PLC)
END_VAR
```

kind → IEC type: `bool`→`BOOL`, `int`→`DINT`, `real`→`LREAL`, `vec3`→`ARRAY[0..2] OF LREAL`.

## 2. What the PLC program does

Replicate `MockPLC`'s behaviour (`src/deltahil/plc/mock_plc.py`) in ST/ladder:
- Drive the pick state machine `await_part → approach → grip → done` from
  `GVL_Sensor.part_present` and `GVL_Sensor.grip_confirm`.
- Emit `GVL_Cmd.target_xyz = reported_pick + GVL_Sup.calib_offset_xyz` (the PLC
  applies the calibration correction), assert `GVL_Cmd.grip` only in the grip phase.
- The link **reads** `GVL_Cmd.*` + `GVL_Sup.mode`/`calib_offset_xyz` and **writes**
  `GVL_Sensor.*` (and, via a supervisory hook, `GVL_Sup.cycle_count`/`last_residual_mm`).

## 3. Loupe Beckhoff Bridge setup (in Isaac Sim on the rig)

1. Clone the extension and add its `exts/` to Isaac's extension search path (or install
   per its README): https://github.com/loupeteam/Omniverse_Beckhoff_Bridge_Extension
2. In Isaac: **Window → Extensions**, search *Beckhoff*, enable it (Third Party).
3. **Beckhoff Bridge → Open Bridge Settings**: set **PLC AMS Net ID** (e.g.
   `5.1.204.123.1.1`), **Refresh Rate** (ms), tick **Enable ADS Client**. Save.
4. `pyads` must be importable in Isaac's Python (the extension bundles/uses it).

`scripts/run_twincat_loop.py` wires `TwinCATLink(ams_net_id) + KinematicDeltaPlant +
Bridge` and interleaves `app.update()` (drives the bridge's cyclic ADS reads →
data-callback) with `bridge.scan()`; it reports `bridge.fast_meter.summary_ms()` —
the eval-5 (P1/A) home. That latency is **rig-verifiable only**; polled ADS may not
meet <10 ms / σ<1 ms — true EtherCAT process-image I/O is the upgrade if needed.
