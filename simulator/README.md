# Line-follower PID Simulator

A 2D simulator for the SUIVEUR LASSAAD robot, to tune the line-following PID
(`Kp`/`Kd`) and find the maximum smooth speed **without risking the hardware**.
It ports `pidfollow()` from `ea_PIDlineFollow.ino` exactly, plus a 16-sensor
QTR model and differential-drive physics (with motor inertia + sub-stepping).

## Run it (Git Bash / PowerShell)

```bash
cd "C:/Users/HP/Desktop/02 - ROBOTICS/ADVANCED ROBOT/simulator"

# 1) AUTO-TUNE: grid-search Kp/Kd for the smoothest run at a target speed
python robot_sim.py autotune --speed 230 --track hard

# 2) Find the max smooth speed at given gains:
python robot_sim.py sweep --kp 0.019 --kd 5 --lo 100 --hi 255 --step 15 --maxwobble 3

# 3) Draw the robot's path for one setting (static .svg, open in a browser):
python robot_sim.py plot --speed 200 --kp 0.030 --kd 6 --track hard

# 4) WATCH IT MOVE: animated .svg (a dot loops the track) -- no matplotlib needed
python robot_sim.py animate --speed 200 --kp 0.030 --kd 6 --track hard
```

All output lands in `simulator/out/`. Open the `.svg` files in any browser
(double-click). `plot` = static path (green=start, red=end, grey=track,
blue=robot path). `animate` = a red dot that loops the track in real-ish time.
Tracks: `--track oval` (gentle) | `hard` (tight corners) | `chicane` | `scurve`.

## Drive a REAL track from an image (`--image`)

Instead of a built-in shape, load YOUR track from a photo/drawing and tune on it.
Reuses `track_planner.py`'s vision pipeline (needs OpenCV). Works for a loop, an
open line, or a fork (give `--start/--end` pixel coords to pick the branch):

```bash
# any mode accepts --image; pass --scale (px per cm) for accurate distances
python robot_sim.py plot     --form simple --image out/track_oval.png --scale 10 --speed 200 --kp 0.030 --kd 1.2
python robot_sim.py autotune --form simple --image out/track_oval.png --scale 10 --speed 150
python robot_sim.py sweep    --form simple --image my_track.png        --scale 10 --kp 0.030 --kd 1.2
# a fork: choose the route by start/end PIXELS (col,row). Use '=' for the minus.
python robot_sim.py plot     --form simple --image out/track_junction.png --scale 10 --start=100,600 --end=800,350
```

Image flags: `--scale` (px/cm; else inferred from line width — inference is
approximate, pass it if you know it), `--invert 1|0` (line is white|dark; else
auto), `--start/--end` (fork route, in pixels). Open routes end by "losing the
line" at the finish — that's the track ending, not a tuning failure.

> Note: live matplotlib animation isn't used because this PC's Python is 32-bit
> (no matplotlib wheels). The animated SVG replaces it and needs zero installs.

## IMPORTANT: calibrate to your real robot

The tuning only transfers to hardware if the constants in the `CFG` block of
`robot_sim.py` match reality. Measure and set:

| Constant            | What to measure on the real robot |
|---------------------|------------------------------------|
| `v_at_pwm255_cmps`  | drive a wheel at PWM 255 for 1 s, measure cm travelled |
| `wheelbase_cm`      | distance between the two drive wheels |
| `sensor_ahead_cm`   | distance from wheel axle to the sensor bar |
| `sensor_pitch_cm`   | spacing between adjacent QTR sensors |
| `loop_dt_ms`        | how often your control loop actually runs (Kd depends on this!) |
| `SetPoint`          | the `position` value your QTR reports when centred on the line |
| `motor_tau_s`       | how quickly the wheels reach commanded speed (inertia) |

## Status / known limitations

- Physics is validated: the robot completes full laps; lap time drops with speed.
- Currently *idealized*: no sensor noise, no control latency. So on a gentle
  track it follows almost perfectly at every speed. To find a realistic limit,
  use a harder track and/or enable noise/latency (planned).
- Only line-following is modelled. Wall-following (ultrasonic) and the maze
  path-string logic are not simulated yet.
