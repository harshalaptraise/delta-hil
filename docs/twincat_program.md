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

Home/pick points are in the **robot-local frame** (mm) so they line up with the
IRB 360 render (`scripts/run_twincat_render.py` uses the same numbers). The `dwell`
timer holds each phase long enough that the motion is watchable in the render.

```iecst
PROGRAM MAIN
VAR
    phase : INT := 0;                                     // 0 await 1 approach 2 grip 3 retract
    home  : ARRAY[0..2] OF LREAL := [0.0, 0.0, -1010.0];  // raised home (mm, robot frame)
    pick  : ARRAY[0..2] OF LREAL := [120.0, -80.0, -1180.0]; // pick point (mm, robot frame)
    dwell : TON;                                          // per-phase dwell
    i     : INT;
END_VAR

GVL_Cmd.tracking := FALSE;

CASE phase OF
    0:  // await part -- sit at home
        FOR i := 0 TO 2 DO GVL_Cmd.target_xyz[i] := home[i]; END_FOR
        GVL_Cmd.grip := FALSE;
        dwell(IN := FALSE);
        IF GVL_Sensor.part_present THEN phase := 1; END_IF

    1:  // approach: move to the calibrated pick point, grip open
        FOR i := 0 TO 2 DO GVL_Cmd.target_xyz[i] := pick[i] + GVL_Sup.calib_offset_xyz[i]; END_FOR
        GVL_Cmd.grip := FALSE;
        dwell(IN := TRUE, PT := T#3S);
        IF dwell.Q THEN phase := 2; dwell(IN := FALSE); END_IF

    2:  // grip: hold on the part, close, wait for confirm (+dwell)
        FOR i := 0 TO 2 DO GVL_Cmd.target_xyz[i] := pick[i] + GVL_Sup.calib_offset_xyz[i]; END_FOR
        GVL_Cmd.grip := TRUE;
        dwell(IN := TRUE, PT := T#2S);
        IF GVL_Sensor.grip_confirm AND dwell.Q THEN
            GVL_Sup.cycle_count := GVL_Sup.cycle_count + 1;
            phase := 3; dwell(IN := FALSE);
        END_IF

    3:  // retract to home, then look for the next part
        FOR i := 0 TO 2 DO GVL_Cmd.target_xyz[i] := home[i]; END_FOR
        GVL_Cmd.grip := FALSE;
        dwell(IN := TRUE, PT := T#3S);
        IF dwell.Q THEN phase := 0; dwell(IN := FALSE); END_IF
END_CASE
```

Set the cyclic task to ~1–4 ms for the eval-5 timing regime (the dwell is real-time,
independent of task rate). Activate the configuration and run. For a pure connectivity
check the dwell just makes the cycle count climb slowly — that's fine.

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
