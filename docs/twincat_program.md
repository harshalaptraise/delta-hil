# TwinCAT PLC program for the delta-hil loop

Import this into a TwinCAT 3 PLC project so the sim can close the loop against a
real controller. It mirrors `MockPLC` (`src/deltahil/plc/mock_plc.py`): a
pick state machine driven only by the sensor tags (P2). Types/units per
`docs/twincat_gvl_spec.md` (mm, base frame).

## 1. GVLs (Add > Global Variable List)

```iecst
// GVL_Cmd   (PLC -> plant, FAST)
VAR_GLOBAL
    target_xyz : ARRAY[0..2] OF LREAL;
    grip       : BOOL;
    tracking   : BOOL;
END_VAR

// GVL_Sensor (plant -> PLC, FAST)
VAR_GLOBAL
    part_present : BOOL;
    grip_confirm : BOOL;
    tcp_xyz      : ARRAY[0..2] OF LREAL;
END_VAR

// GVL_Sup   (supervisory, SLOW)
VAR_GLOBAL
    mode             : DINT := 1;             // 0 idle 1 run 2 calibrate  (PLC->plant)
    calib_offset_xyz : ARRAY[0..2] OF LREAL;  //                            (PLC->plant)
    cycle_count      : DINT;                   //                            (plant->PLC)
    last_residual_mm : LREAL;                  //                            (plant->PLC)
END_VAR
```

## 2. MAIN (Structured Text) — run in a cyclic task

```iecst
PROGRAM MAIN
VAR
    phase : INT := 0;                                  // 0 await 1 approach 2 grip 3 done
    pick  : ARRAY[0..2] OF LREAL := [0.0, 0.0, -900.0]; // fixed pick point (mm), base frame
    i     : INT;
END_VAR

// defaults each scan
GVL_Cmd.tracking := FALSE;

CASE phase OF
    0:  // await part
        FOR i := 0 TO 2 DO GVL_Cmd.target_xyz[i] := 0.0; END_FOR
        GVL_Cmd.grip := FALSE;
        IF GVL_Sensor.part_present THEN phase := 1; END_IF

    1:  // approach: command the calibrated pick point, grip open
        FOR i := 0 TO 2 DO
            GVL_Cmd.target_xyz[i] := pick[i] + GVL_Sup.calib_offset_xyz[i];
        END_FOR
        GVL_Cmd.grip := FALSE;
        phase := 2;

    2:  // grip: hold target, close gripper, wait for confirm
        FOR i := 0 TO 2 DO
            GVL_Cmd.target_xyz[i] := pick[i] + GVL_Sup.calib_offset_xyz[i];
        END_FOR
        GVL_Cmd.grip := TRUE;
        IF GVL_Sensor.grip_confirm THEN
            GVL_Sup.cycle_count := GVL_Sup.cycle_count + 1;
            phase := 3;
        END_IF

    3:  // done: release, then look for the next part
        GVL_Cmd.grip := FALSE;
        phase := 0;
END_CASE
```

Set the cyclic task to ~1–4 ms if you want the eval-5 timing regime; for a first
connectivity test any task rate is fine. Activate the configuration and run.

## 3. Bring-up (on the rig, where TwinCAT runs)

```
D:\Harshal\Isacsim\isaacenv\Scripts\activate
pip install pyads

# find the AMS NetId: TwinCAT tray icon -> Router -> Edit Routes (local target),
# or it's <local-ip>.1.1  (PLC runtime port = 851)

# stage 1 -- prove the loop with the mock plant (NO Isaac, fast to debug):
python scripts\run_twincat_mock.py <AMS_NET_ID>

# stage 2 -- same loop with the Isaac kinematic plant (boots Isaac):
python scripts\run_twincat_loop.py <AMS_NET_ID>
```

Stage 1 should show `cmd.target` following the PLC and `grip_confirm` going true as
the mock plant reaches the point, with `cycle_count` incrementing. `fast_meter`
reports the round-trip latency (eval-5 home; ADS polling may not meet <10 ms /
σ<1 ms — true EtherCAT process-image I/O is the upgrade).
