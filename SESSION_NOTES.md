# ADVANCED ROBOT — session notes (resume point)

Last worked: 2026-06-19. (Prev: 2026-06-18.)

## NEW (2026-06-19): image -> fastest-plan generator  ** track_planner.py **
User's goal: "input a track image/conception, the robot analyses it and drives
the BEST/fastest path." Built `simulator/track_planner.py` — full pipeline:
image -> OpenCV binary mask -> Zhang-Suen skeleton -> spur prune -> graph (nodes
via CROSSING NUMBER, edges) -> route (pure loop, OR fastest start->end via
Dijkstra-over-time with turn penalty, labelling F/L/R) -> curvature-limited speed
profile (fwd/back accel passes) quantized to firmware levels 1(pwm140)/3(pwm220)
-> emits DRAFT `path[]` + `pathDistances[]` (cm) for a_pins_vars.ino -> validates
in robot_sim.simulate. Outputs `out/plan.svg` (line coloured by speed) + skeleton.png.
- Toolchain WIN: 32-bit Python HAS **OpenCV 4.13 + Pillow 9.0.1** (no scipy). 
- Test images via `simulator/make_test_track.py oval|junction` (10 px/cm baked in).
- Verified end-to-end: oval -> "B1s"; junction(start=100,600 end=800,350) ->
  "B1FLs" {0,0,22,50,52}, both junction decisions present; both pass sim validation.
- Decisions made with user: input = CLEAN digital drawings; build handles BOTH
  single-line and junctions. `--start/--end` are in PIXELS (col,row), use `=`.
- Docs: `simulator/TRACK_PLANNER.md`.
- DRAFT caveat: B/W colour mode + `C` conditions NOT inferable from a line image
  (default 'B', omitted) — user must review before flashing.

### Possible next steps for the planner
- Add photo-of-real-track support (perspective correction, shadow handling).
- Tune `PLAN` a_lat/a_long to the real robot once calibration numbers arrive.
- Smarter token emit (insert speed-change `1`/`3` mid-segment; map `C` conditions).
- Let user mark start/end on the image instead of typing pixel coords.

## NEW (2026-06-20): simple reusable follower ** SimpleLineFollower/ **
User wanted "a new code using the PID follower, simple to use and adapted to ANY
track." Built `SimpleLineFollower/SimpleLineFollower.ino` — ONE self-contained
sketch, no path string (pure reactive follower, runs any track unchanged). Reuses
exact motor pins/wiring + the fixed PID integral. Only dep = QTRSensors lib
(dropped MPU/Ultrasonic/FastLED/LCD). Features: auto-calibrate (sweep 8s) -> button
start -> PID follow; line-recovery pivot toward last-seen side on gap/sharp corner;
corner braking (TURN_SLOW); LINE_IS_BLACK flag for inverted tracks. 3 tuning knobs
(BASE_SPEED/KP/KD). Docs `SimpleLineFollower/README.md`. NOT a replacement for the
maze firmware — separate simpler sketch. Per-loop derivative (not /elapsedTime), so
default gains KP0.02/KD5 are starting points, not sim-matched yet.

### (2026-06-20 cont.) Tuned SimpleLineFollower gains in the simulator
Added a `PIDSimple` controller to robot_sim.py matching the sketch's EXACT form
(per-loop derivative, NOT /dt; clamped integral). New CLI flags `--form
simple|firmware` and `--dt <ms>` (overrides loop period). Autotuned form=simple,
dt=2ms, speed=150, hard track: wobble falls monotonically with Kp; grid edge
Kp0.038/Kd1.5=0.80cm. Picked the robust shoulder **Kp=0.030, Kd=1.2** (0.91cm,
margin vs real-world oscillation the sim under-penalizes). Sweep: stays <2cm to
PWM255. Beats firmware-form defaults on every track. scurve/chicane "lost line"
= open-track end + curvature (firmware form loses them worse), real sketch
recovers. Baked Kp0.030/Kd1.2 into the .ino as defaults + added Serial loop-time
print (KD is loop-rate dependent: KD = 1.2*(2/loop_ms)). Viz: out/anim_s150_kp0.03_kd1.2_hard.svg.
BASE_SPEED ceiling sweep (KP0.030/KD1.2, dt2): oval very-smooth(<1.5cm) to PWM255;
hard very-smooth to 205, on-line w/~2cm to 255. NEVER lost line at any speed
(TURN_SLOW brakes corners) -> BASE_SPEED ~= straight speed; corner speed=TURN_SLOW.
Recommend BASE_SPEED 220-240 open / 180 tight (sim optimistic, keep 15-20% margin).
Added `--turn` CLI override (TURNFACTOR/TURN_SLOW). KEY FINDING: TURN_SLOW is a
speed<->smoothness TRADE, not a speed boost. At BASE255/hard: turn1.0=4.34s@2.0cm,
turn1.5=5.81s@1.6cm, turn0.7=3.99s@2.4cm, turn3.5=12.2s@0.9cm. Never lost line even
turn0.3. Fastest lap = MAX base + LOWEST tolerable turn. So to go faster raise
BASE_SPEED, keep TURN_SLOW~1.0. Bumped sketch defaults: BASE_SPEED 150->200,
TURN_SLOW stays 1.0 (documented the trade in .ino + README table).

### (2026-06-20 cont.) Wrote GUIDE.md (top level)
Beginner guide: (0) big-picture two approaches table — reactive SimpleLineFollower
(NO image needed, follows any track live) vs programmed maze firmware (path string,
track_planner can gen it from image). (1) SimpleLineFollower code walkthrough
(flow, position/error, PID P/I/D plain-English, TURN_SLOW corner braking, recovery,
drive()). (2) image->data via track_planner (good-image rules, step-by-step cmds,
reading plan.svg/skeleton.png, DRAFT caveat, PLAN knobs). (3) which-path decision.
(4) cheat-sheet. NOTE flagged: feeding an arbitrary image-extracted track into the
sim for the REACTIVE follower is not built yet (offered as next feature).

### (2026-06-20 cont.) Added --image track loader to robot_sim.py
Now you can drive a REAL track from an image in the sim (tune reactive follower on
YOUR track). Added `track_from_image()` (lazy-imports track_planner, reuses its
vision pipeline: load_mask->zhang_suen->infer_px_per_cm->prune_spurs->build_graph->
choose_route->assemble_route -> cm centerline) + `_parse_px`. New CLI flags on all
subcommands: --image, --scale, --invert, --start, --end. Image filename becomes the
track label. Tested: oval.png -> loop (scale10 -> 186cm correct perimeter; inference
11.36px/cm slightly off due to AA, so pass --scale); junction.png --start/--end ->
routed branch; autotune on image works (oval BEST kp0.038/kd1.0). Open routes end by
"losing line" at finish (expected). Autotune verify-hint now image-aware. Docs
updated: simulator/README.md + GUIDE.md §2.7. Use loop images for clean lap stats.

## (still open) Original resume point: calibrate the simulator

## What this is
Working copy of the SUIVEUR LASSAAD line-follower/maze robot (Arduino Mega).
- Original (untouched): `C:\Users\HP\Desktop\02 - ROBOTICS\ALLIANCE\SUIVEUR LASSAAD\a_pins_vars`
- Working copy: `C:\Users\HP\Desktop\02 - ROBOTICS\ADVANCED ROBOT\a_pins_vars`

## Done so far
1. **Reviewed the firmware** and found 6 bugs; **fixed all 6** in the working copy:
   - #1 `bb_sensors.ino` localCountLines() missing `return` on recursion
   - #2 `fb_RR_sensors_cases.ino` `mode == 'N'` (compare) -> `mode = 'N'` (assign), 5 spots
   - #3 `d_wallFollow.ino` int Kp2/Kd2/localKp2 truncated float gains -> float
   - #4 `ea_PIDlineFollow.ino` `int i = i + prev_i` used uninitialised i -> `prev_i + error`
   - #5 `d_wallFollow.ino` `lasterror` local/uninit -> `static int lasterror = 0`
   - #6 `ba_util_fn.ino` compare() compared 12-char patterns vs 16 sensors -> uses strlen
   - Proven on PC in `_proof_tests/` (proof.cpp shows bugs, verify_fixed.cpp: 8/8 pass).

2. **Built a PID line-follower simulator** in `simulator/robot_sim.py` (Python+numpy,
   no other deps). Faithful port of pidfollow(); 16-sensor QTR model; differential
   drive with motor inertia + sub-stepping; sensor noise + control latency; tracks
   oval/hard/chicane/scurve. Modes:
   - `autotune` — grid-search Kp/Kd for smoothest run at a target speed
   - `sweep`    — max smooth speed at fixed gains
   - `plot`     — static path SVG
   - `animate`  — animated SVG (moving dot; matplotlib unusable -> Python is 32-bit)
   - Output SVGs in `simulator/out/`. Usage in `simulator/README.md`.
   - Early finding: auto-tuner suggests Kp~0.030, Kd~6 beat the firmware's 0.019/5
     (on DEFAULT, un-calibrated constants).

## (2026-06-22) Calibration run — PARTIALLY DONE, CFG INCONSISTENT
User flashed the helper. NOTE: helper was NOT pasted into a tab (nothing printed
at first); FIX = created `a_pins_vars/zz_calib_helper_TEMP.ino` (own tab, auto-
compiled) + moved `calibrate_all();` in y_setup.ino to BETWEEN calibratesensors()
and waitForStartButton() (it was after, blocked behind the button wait).
>>> REMEMBER TO DELETE zz_calib_helper_TEMP.ino + that calibrate_all(); line. <<<

Measured 7 numbers: wheelbase=18, sensor_ahead=15-17(used 16), sensor_pitch=0.8,
line_width=4, loop_dt=3.2, v_at_pwm255=85, SetPoint=7500.
- SetPoint gotcha: first read 3500 = MIS-CENTERED. Re-measured centered (Dsensors
  0000000110000000, two middle sensors) = 7500, matches (16-1)*1000/2. Use 7500.
  (Firmware global SetPoint defaults to 9700 = off-centre; a_pins_vars.ino:108.)

### !!! CFG ONLY 4/7 WRITTEN — user interrupted with "SAVE AND CLOSE" !!!
robot_sim.py CFG currently has: wheelbase 18 / sensor_ahead 16 / sensor_pitch 0.8
/ line_width 4 WRITTEN. SetPoint already 7500. BUT these 3 are STILL OLD DEFAULTS:
  - v_at_pwm255_cmps = 230  -> SHOULD BE 85
  - loop_dt_ms       = 5.0  -> SHOULD BE 3.2
  - latency_ms       = 6.0  -> SHOULD BE 3.2
>>> DO NOT autotune until these 3 are fixed (v=230 is 2.7x too fast, loop_dt drives Kd). <<<

## >>> RESUME HERE <<<
1. Finish CFG: set v_at_pwm255_cmps=85, loop_dt_ms=3.2, latency_ms=3.2 in robot_sim.py.
2. Re-run `autotune` (now on real-robot constants) for calibrated Kp/Kd.
3. Remove the calib helper (zz_calib_helper_TEMP.ino + calibrate_all(); line).
- After: optionally tune wall-follow PID, simulate the maze path-string, flash the
  fixed firmware (Arduino IDE, board Mega 2560).
