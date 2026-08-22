# track_planner.py — image → fastest robot plan

Turn a **clean top-down track image** into the fastest plan the SUIVEUR LASSAAD
robot can actually drive, then prove it in `robot_sim.py`.

## What it does (pipeline)
1. **Vision** (OpenCV): image → binary line mask (auto dark/light) → 1px centerline
   (Zhang-Suen thinning) → spur pruning.
2. **Scale**: infers px/cm from the known line width (distance transform). Override
   with `--scale` for exact results (recommended for real drawings — you know it).
3. **Graph**: centerline → nodes (endpoints / junctions, detected by the **crossing
   number**, not raw neighbour count — that's what makes curves not look like
   junctions) + edges (polylines).
4. **Route**:
   - pure loop → drive the whole loop;
   - junctions → **fastest** start→end route (Dijkstra over *time*, with a turn
     penalty), labelling each junction `F`/`L`/`R` by heading change.
5. **Speed profile**: curvature-limited speed `v=sqrt(a_lat/κ)` with forward
   (accel) and backward (brake) passes, then quantized to the robot's discrete
   levels (`1`=pwm140, `3`=pwm220).
6. **Emit**: a DRAFT `path[]` + `pathDistances[]` (cm) to paste into
   `a_pins_vars.ino`. Speed-changes and turns are merged by arc-length;
   `pathDistances[i]` = cm to PID-follow *before* firing token `i`.
7. **Validate**: feeds the extracted centerline into `robot_sim.simulate` at the
   planned speed/gains and reports whether the robot keeps the line.

## Usage
```bash
PY=/c/PySchool/3.10-32-bit/python.exe          # 32-bit Python on this PC

# 1) make a sample image (or use your own clean PNG)
$PY make_test_track.py oval                    # single loop
$PY make_test_track.py junction                # a fork (route choice)

# 2) plan
$PY track_planner.py plan out/track_oval.png
$PY track_planner.py plan out/track_oval.png --scale 10        # force px/cm

# 3) plan a junction track: give START and END in PIXEL coords (col,row)
#    read straight off the image. NOTE: use '=' so the leading '-' isn't parsed.
$PY track_planner.py plan out/track_junction.png --start=100,600 --end=800,350

# 4) plan + validate in the simulator
$PY track_planner.py validate out/track_oval.png
```
Outputs: `out/plan.svg` (chosen line coloured green=fast→red=slow, blue=nodes,
turn letters, green/red start/end) and `out/skeleton.png` (debug).

## Important: the emitted path is a DRAFT
The geometry, route, turns and speeds are computed. But the firmware's `B`/`W`
**colour mode** and any `C` **distance/ultrasonic conditions** depend on the
physical track and are NOT inferable from a plain line drawing — they default to
`B` and are omitted. Review before flashing. The turn tokens assume the robot's
sensors detect each junction (which is how the firmware triggers `case_*`).

## Tuning (top of `track_planner.py`, `PLAN` dict)
- `line_width_cm` — your tape width (for scale inference).
- `a_lat_cmps2` — turn aggressiveness (lower = safer, slower in curves).
- `a_long_cmps2` — accel/decel limit.
- `v_at_pwm255`, `pwm_level1/3` — drivetrain & the two firmware speed levels.
- `turn_penalty_s` — how much each junction decision "costs" in routing.

## Requirements
OpenCV + Pillow + numpy (all present in `C:\PySchool\3.10-32-bit`). No scipy,
no matplotlib (output is SVG, same as the simulator).
