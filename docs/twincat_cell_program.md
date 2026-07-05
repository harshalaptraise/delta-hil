# TwinCAT cell program — the PLC runs the continuous two-robot line

This is the TwinCAT twin of `MockCellController`
(`src/deltahil/plc/cell_controller.py`, verified on the laptop). The sim
(`CellPlant`) streams parts/boxes and reports their live positions; **the PLC**
tracks them, assigns robots, and commands each TCP + grip on the fly. Exchanged
via `CellAdsLink` (`src/deltahil/plc/cell_link.py`) — its own `GVL_Cell`, separate
from the single-robot tag map. Units: **mm**, cell/world frame (the same frame the
sim renders; `x` along the belts, product lane `y≈−150`, box lane `y≈+150`).

Array sizes **must** match `cell_plant.py`: `K_PARTS = 6`, `K_BOXES = 4`.

## 1. GVL_Cell

```iecst
VAR_GLOBAL
    // --- sim -> PLC (sensors) ---
    part_id    : ARRAY[0..5] OF DINT;    // nearest belt parts (id, live pos)
    part_x     : ARRAY[0..5] OF LREAL;
    part_y     : ARRAY[0..5] OF LREAL;
    part_valid : ARRAY[0..5] OF BOOL;
    box_id     : ARRAY[0..3] OF DINT;    // nearest boxes
    box_x      : ARRAY[0..3] OF LREAL;
    box_fill   : ARRAY[0..3] OF DINT;
    box_valid  : ARRAY[0..3] OF BOOL;
    rA_tcp     : ARRAY[0..2] OF LREAL;   // robot actual TCP (feedback)
    rA_confirm : BOOL;                    // grasp achieved
    rB_tcp     : ARRAY[0..2] OF LREAL;
    rB_confirm : BOOL;
    belt_v_src : LREAL;                   // belt velocities (mm/s, feed-forward)
    belt_v_box : LREAL;

    // --- PLC -> sim (commands) ---
    cA_tcp : ARRAY[0..2] OF LREAL;        // commanded TCP (mm)
    cA_grip : BOOL;
    cB_tcp : ARRAY[0..2] OF LREAL;
    cB_grip : BOOL;
END_VAR
```

## 2. FB_CellRobot — one robot's tracking state machine

> **Paste note:** the `FUNCTION_BLOCK FB_CellRobot` header line below MUST be present
> in the POU's declaration pane. If you paste the `VAR … END_VAR` blocks without it
> (or overwrite the auto-generated header), TwinCAT has no POU to bind the variables
> to and every identifier — `ptmr`, `gx`, `box_id`, … — reports "not defined."
> Declaration pane = `FUNCTION_BLOCK` header + all `VAR`/`VAR CONSTANT` blocks;
> implementation pane = only the code after the last `END_VAR`.

```iecst
FUNCTION_BLOCK FB_CellRobot
VAR_INPUT
    rx           : LREAL;    // robot axis x (mm): A = -700, B = +700
    upstream     : BOOL;     // A = TRUE (takes even-id share only); B = FALSE (catch-all)
    grip_confirm : BOOL;
    other_claim  : DINT;     // the other robot's currently-claimed part id
END_VAR
VAR_IN_OUT
    my_claim : DINT;         // this robot's claimed part id (-1 = none)
END_VAR
VAR_OUTPUT
    cmd_x : LREAL; cmd_y : LREAL; cmd_z : LREAL; cmd_grip : BOOL;
END_VAR
VAR
    phase      : INT := 0;    // 0 idle 1 track 2 lift 3 transfer 4 place 5 retract
    phase_prev : INT := 0;
    part       : DINT := -1;
    gx         : LREAL;
    gy         : LREAL;
    ptmr       : TON;         // per-phase elapsed timer
    elapsed    : TIME;
    locktmr    : TON;         // tracking-lock dwell before gripping
    plctmr     : TON;         // place descend timer (only while tote in-window)
    box_id     : DINT := -1;  // committed tote id (one tote per place, no switch)
    i          : INT;
    found      : BOOL;
    boxfound   : BOOL;
    bestd      : LREAL;
    px         : LREAL;
    py         : LREAL;
    bx         : LREAL;
    bfill      : DINT;
    dx         : LREAL;
    dy         : LREAL;
    lat        : LREAL;
END_VAR
VAR CONSTANT
    WIN       : LREAL := 80.0;    // track only within clean reach (no over-stretch)
    CLAIM_LO  : LREAL := 300.0;
    REACH     : LREAL := 170.0;
    Z_MIN     : LREAL := 100.0;
    Z_MAX     : LREAL := 600.0;
    HOME_Z    : LREAL := 420.0;
    PICK_Z    : LREAL := 480.0;
    PICK_HI   : LREAL := 580.0;
    BOX_Y     : LREAL := 150.0;
    PLACE_HI  : LREAL := 460.0;
    STACK0    : LREAL := 180.0;
    THICK     : LREAL := 14.0;
    LOCK_TIME : TIME  := T#300MS; // tracking-lock dwell (could be a GVL input)
END_VAR

// per-phase elapsed time (resets the cycle after phase changes)
ptmr(IN := (phase = phase_prev), PT := T#30S);
phase_prev := phase;
elapsed := ptmr.ET;

// only the pick chase gives up (a part slipped by before grabbing). Once a part is
// GRABBED the robot NEVER abandons it -- carrying phases wait as long as needed.
IF phase = 1 AND elapsed > T#4S THEN phase := 0; part := -1; my_claim := -1; END_IF

CASE phase OF
0:  // idle -- claim nearest upstream, catchable, un-claimed part in my share
    cmd_x := rx; cmd_y := 0.0; cmd_z := HOME_Z; cmd_grip := FALSE;
    part := -1; my_claim := -1; box_id := -1;
    found := FALSE; bestd := 1.0E9;
    FOR i := 0 TO 5 DO
        IF GVL_Cell.part_valid[i] AND (GVL_Cell.part_id[i] <> other_claim) THEN
            IF (NOT upstream) OR (GVL_Cell.part_id[i] MOD 2 = 0) THEN
                px := GVL_Cell.part_x[i]; py := GVL_Cell.part_y[i];
                // claim any REACHABLE part (upstream OR already in the window) so a
                // free robot never watches a reachable part pass
                IF (px >= rx - CLAIM_LO) AND (px <= rx + WIN) AND (ABS(py) < REACH) THEN
                    IF ABS(px - rx) < bestd THEN
                        bestd := ABS(px - rx); part := GVL_Cell.part_id[i]; found := TRUE;
                    END_IF
                END_IF
            END_IF
        END_IF
    END_FOR
    IF found THEN my_claim := part; phase := 1; END_IF

1:  // track -- follow the live part position (velocity match), descend, grip
    IF grip_confirm THEN
        cmd_x := gx; cmd_y := gy; cmd_z := PICK_HI; cmd_grip := TRUE; phase := 2;
    ELSE
        found := FALSE;
        FOR i := 0 TO 5 DO
            IF GVL_Cell.part_valid[i] AND (GVL_Cell.part_id[i] = part) THEN
                px := GVL_Cell.part_x[i]; py := GVL_Cell.part_y[i]; found := TRUE;
            END_IF
        END_FOR
        IF NOT found THEN phase := 0;                       // part gone
        ELSIF px > rx + WIN THEN phase := 0;                // passed my reach
        ELSE
            gx := px; gy := py;
            IF ABS(px - rx) < WIN THEN
                // ride the moving part (velocity-matched) for LOCK_TIME, THEN grip
                locktmr(IN := TRUE, PT := T#5S);
                cmd_x := px; cmd_y := py; cmd_z := PICK_Z;
                cmd_grip := (locktmr.ET >= LOCK_TIME);
            ELSE
                locktmr(IN := FALSE);
                IF px < rx THEN cmd_x := rx - WIN; ELSE cmd_x := rx + WIN; END_IF
                cmd_y := py; cmd_z := PICK_HI; cmd_grip := FALSE;             // hover at window edge
            END_IF
        END_IF
    END_IF

2:  // lift
    cmd_x := gx; cmd_y := gy; cmd_z := PICK_HI; cmd_grip := TRUE;
    IF elapsed > T#250MS THEN phase := 3; END_IF

3:  // transfer -- COMMIT to one tote (find the committed id; if gone, pick nearest)
    plctmr(IN := FALSE);                                    // keep place timer reset until place
    boxfound := FALSE;
    FOR i := 0 TO 3 DO
        IF GVL_Cell.box_valid[i] AND (GVL_Cell.box_id[i] = box_id) THEN
            bx := GVL_Cell.box_x[i]; bfill := GVL_Cell.box_fill[i]; boxfound := TRUE;
        END_IF
    END_FOR
    IF NOT boxfound THEN                                    // commit to the nearest tote
        bestd := 1.0E9;
        FOR i := 0 TO 3 DO
            IF GVL_Cell.box_valid[i] AND (GVL_Cell.box_x[i] <= rx + WIN + 500.0) THEN
                IF ABS(GVL_Cell.box_x[i] - rx) < bestd THEN
                    bestd := ABS(GVL_Cell.box_x[i] - rx); bx := GVL_Cell.box_x[i];
                    bfill := GVL_Cell.box_fill[i]; box_id := GVL_Cell.box_id[i]; boxfound := TRUE;
                END_IF
            END_IF
        END_FOR
    END_IF
    IF NOT boxfound THEN
        cmd_x := rx; cmd_y := BOX_Y; cmd_z := PLACE_HI; cmd_grip := TRUE;     // hold, wait for a tote
    ELSE
        IF (ABS(bx - rx) < WIN) AND (elapsed > T#150MS) THEN phase := 4; END_IF
        cmd_x := LIMIT(rx - WIN, bx, rx + WIN); cmd_y := BOX_Y; cmd_z := PLACE_HI; cmd_grip := TRUE;
    END_IF

4:  // place -- stay committed; descend timer runs ONLY while the tote is in-window
    boxfound := FALSE;
    FOR i := 0 TO 3 DO
        IF GVL_Cell.box_valid[i] AND (GVL_Cell.box_id[i] = box_id) THEN
            bx := GVL_Cell.box_x[i]; bfill := GVL_Cell.box_fill[i]; boxfound := TRUE;
        END_IF
    END_FOR
    IF NOT boxfound THEN                                    // committed tote gone -> re-pick nearest
        bestd := 1.0E9;
        FOR i := 0 TO 3 DO
            IF GVL_Cell.box_valid[i] AND (GVL_Cell.box_x[i] <= rx + WIN + 500.0) THEN
                IF ABS(GVL_Cell.box_x[i] - rx) < bestd THEN
                    bestd := ABS(GVL_Cell.box_x[i] - rx); bx := GVL_Cell.box_x[i];
                    bfill := GVL_Cell.box_fill[i]; box_id := GVL_Cell.box_id[i]; boxfound := TRUE;
                END_IF
            END_IF
        END_FOR
    END_IF
    IF NOT boxfound THEN
        plctmr(IN := FALSE);
        cmd_x := rx; cmd_y := BOX_Y; cmd_z := PLACE_HI; cmd_grip := TRUE;   // no tote -> HOLD, never abandon
    ELSIF ABS(bx - rx) < WIN THEN
        plctmr(IN := TRUE, PT := T#5S);                     // descend timer (only while tote in-window)
        cmd_x := bx; cmd_y := BOX_Y; cmd_z := STACK0 + DINT_TO_LREAL(bfill) * THICK;
        IF plctmr.ET < T#350MS THEN cmd_grip := TRUE; ELSE cmd_grip := FALSE; END_IF
        IF NOT grip_confirm THEN phase := 5; END_IF         // placed
    ELSE
        plctmr(IN := FALSE);                                // tote drifted out -> reset timer, hover
        cmd_x := LIMIT(rx - WIN, bx, rx + WIN); cmd_y := BOX_Y; cmd_z := PLACE_HI; cmd_grip := TRUE;
    END_IF

5:  // retract
    cmd_x := rx; cmd_y := 0.0; cmd_z := HOME_Z; cmd_grip := FALSE;
    IF elapsed > T#250MS THEN phase := 0; part := -1; my_claim := -1; box_id := -1; END_IF
END_CASE

// clamp into the reach envelope (P4) so no command is unreachable
dx := cmd_x - rx; dy := cmd_y; lat := SQRT(dx*dx + dy*dy);
IF lat > REACH THEN cmd_x := rx + dx * REACH / lat; cmd_y := dy * REACH / lat; END_IF
cmd_z := LIMIT(Z_MIN, cmd_z, Z_MAX);
```

## 3. MAIN

```iecst
PROGRAM MAIN
VAR
    rA : FB_CellRobot; rB : FB_CellRobot;
    claim_A : DINT := -1; claim_B : DINT := -1;
END_VAR

// A (upstream) runs first so B sees A's fresh claim
rA(rx := -700.0, upstream := TRUE,  grip_confirm := GVL_Cell.rA_confirm,
   other_claim := claim_B, my_claim := claim_A);
rB(rx := 700.0,  upstream := FALSE, grip_confirm := GVL_Cell.rB_confirm,
   other_claim := claim_A, my_claim := claim_B);

GVL_Cell.cA_tcp[0] := rA.cmd_x; GVL_Cell.cA_tcp[1] := rA.cmd_y; GVL_Cell.cA_tcp[2] := rA.cmd_z;
GVL_Cell.cA_grip := rA.cmd_grip;
GVL_Cell.cB_tcp[0] := rB.cmd_x; GVL_Cell.cB_tcp[1] := rB.cmd_y; GVL_Cell.cB_tcp[2] := rB.cmd_z;
GVL_Cell.cB_grip := rB.cmd_grip;
```

Run in the cyclic task (~1–4 ms for the eval-5 regime). Activate + Run, then on the
rig:

```
python scripts\run_twincat_cell.py <AMS_NET_ID>       # live PLC
python scripts\run_twincat_cell.py mock               # Python controller, no PLC (pipeline test)
```

`run_twincat_cell.py` closes the loop fast (recording snapshots + the ADS
round-trip latency), then renders the snapshots to `assets/render/twincat_cell.gif`.
The behaviour mirrors the laptop-verified `MockCellController`, so `mock` and the
live PLC should look the same under ideal conditions.
