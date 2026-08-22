# GUIDE — Understanding the line follower & turning a track image into a plan

This explains, in plain language:
1. The **big picture** — the two different ways this robot can drive a track.
2. **How `SimpleLineFollower.ino` works**, part by part.
3. **How to turn a track image into data** (with `track_planner.py`), and exactly
   when you need that.

---

## 0. The big picture — two ways to drive a track

There are **two completely different approaches** in this project. Don't mix them up.

| | **Reactive** (SimpleLineFollower) | **Programmed** (maze firmware) |
|---|---|---|
| File | `SimpleLineFollower/SimpleLineFollower.ino` | `a_pins_vars/` (many .ino files) |
| How it knows the track | It **doesn't** — it just follows whatever line is under the sensors, live | You give it a **path string** like `"B1CCLLs"` + distances |
| Needs a track image? | **No.** Same upload runs on any track | Optional: `track_planner.py` can generate the path string from an image |
| Best for | Plain line-following on any track. Simple. | Mazes / forks where the robot must *choose* a route |

> **If your goal is "follow any track", you do NOT need a track image at all.**
> Flash `SimpleLineFollower`, calibrate, press the button — done.
>
> You use the **image tools** for two reasons:
> - to generate a *path string* for the advanced maze firmware, **or**
> - to **simulate your real track** and tune the best speed/gains for it before
>   you ever touch the hardware.

---

## 1. How `SimpleLineFollower.ino` works

The whole idea: 16 light sensors look at the floor. They tell us **where the line
is** relative to the robot's centre. A controller (PID) turns that into "steer
left/right" commands. That's it — repeated thousands of times a second.

### 1.1 The hardware it talks to
- **16 QTR reflectance sensors** in a bar across the front → read the line.
- **2 motors** (left + right) via an L298-style driver → steering = driving one
  wheel faster than the other.
- **1 button** → press to start.
- Library: **QTRSensors** (only dependency).

### 1.2 The program flow

```
power on
   │
   ▼
setup():  set up pins  →  CALIBRATE (sweep bar over line ~8s)  →  wait for button
   │
   ▼
loop():   read line position  →  on the line?
                                   │           │
                                  yes          no
                                   │           │
                                 PID steer   pivot to re-find line ("recovery")
   (repeat loop() forever, thousands of times per second)
```

### 1.3 Reading the line: "position"
`readPosition()` asks the QTR library for one number, the **position**:

```
  sensor bar:   [0]................[15]
  line under left edge   → position ≈ 0
  line under centre      → position ≈ 7500   (this is CENTER)
  line under right edge  → position ≈ 15000
```

So **`error = position - CENTER`** tells us how far off-centre the line is, and
which side: negative = line is left, positive = line is right.

`onLine()` checks "does any sensor actually see the line?" If none do, we've run
off the line (sharp corner or a gap) and switch to **recovery** instead of PID.

### 1.4 The PID controller (the brain)
PID combines three terms to decide how hard to steer (`pid`):

- **P (proportional) — `KP * error`**: the further off-centre, the harder it
  steers back. This does 90% of the work.
  - Too low → it reacts late and cuts corners.
  - Too high → it over-steers and **wobbles** (zig-zags).
- **D (derivative) — `KD * (error - lastError)`**: looks at how *fast* the error
  is changing and **damps** it, like shock absorbers. Raise KD to calm wobble.
- **I (integral) — `KI * sum_of_errors`**: fixes a steady drift to one side. We
  leave `KI = 0` (not needed for line following); it's there if you ever want it.

```
pid = KP*error + KI*integral + KD*derivative
leftSpeed  = base + pid      // line on the right → speed up the LEFT wheel → turn right
rightSpeed = base - pid
```

### 1.5 Corner braking — `TURN_SLOW`
`base = BASE_SPEED - TURN_SLOW * |pid|`

When steering hard (big `|pid|`, i.e. a corner), this **lowers the base speed** so
the robot slows into the turn and doesn't overshoot. On straights `pid≈0`, so it
runs at full `BASE_SPEED`.

### 1.6 Line recovery
If `onLine()` is false, the robot pivots **toward the side it last saw the line**
(`lastDir`) at `SEARCH_SPEED` until a sensor finds the line again. This is what
lets the same code survive **sharp corners, gaps, and any track shape** without
any pre-programming.

### 1.7 The motors — `drive(left, right)`
Takes a **signed** speed for each wheel (−255…255). Positive = forward, negative
= that wheel reverses (needed for sharp pivots). It sets the direction pins and
writes the PWM. `stopMotors()` cuts everything.

### 1.8 The knobs you tune (and what the simulator already found)

| Knob | Meaning | Tuned value | Notes |
|---|---|---|---|
| `KP` | steering strength | **0.030** | sim-tuned for smoothest follow |
| `KD` | wobble damping | **1.2** | depends on loop speed (see below) |
| `KI` | drift fix | 0 | leave off |
| `BASE_SPEED` | cruise PWM | **200** | push to 240–255 once trusted |
| `TURN_SLOW` | corner braking | **1.0** | speed↔smooth trade, *not* a speed boost |

Two findings from `robot_sim.py`:
- **Faster = raise `BASE_SPEED`, keep `TURN_SLOW` ≈ 1.0.** Raising `TURN_SLOW`
  only makes corners *slower* (it's a smoothness/safety knob).
- **`KD` depends on loop speed** because the derivative here is per-loop. The
  sketch prints its average loop time over Serial; if it isn't ~2 ms, rescale:
  `KD = 1.2 * (2 / your_loop_ms)`.

See `SimpleLineFollower/README.md` for the upload + tuning checklist.

---

## 2. Turning a track image into data (`track_planner.py`)

**Remember:** SimpleLineFollower does NOT consume an image. This section is for
(a) generating a path string for the maze firmware, or (b) extracting your real
track so you can simulate/tune it.

### 2.1 What the planner does (the pipeline)
```
clean top-down image
   │  OpenCV: make a black/white mask of the line
   ▼
1-pixel-wide centreline  (skeleton / "thinning")
   │  figure out px-per-cm from the known line width (or you pass --scale)
   ▼
a graph: straight bits (edges) joined at junctions/ends (nodes)
   │  pick the route: whole loop, OR fastest start→end through a fork
   ▼
speed profile: slow in tight curves, fast on straights
   │
   ▼
OUTPUT:  path[] + pathDistances[]  (for the maze firmware)  +  plan.svg picture
         and it can VALIDATE the shape in the simulator
```

### 2.2 What makes a *good* image
The vision step needs a **clean, top-down, high-contrast** picture:
- Line clearly darker (or lighter) than the floor; even lighting, no big shadows.
- Straight-down view (not angled) — it can't undo perspective yet.
- **Know your scale**: either keep a known px/cm, or pass `--scale` (px per cm).
  This is how distances come out in real cm.
- Photos of a real taped track *can* work but are riskier (shadows/angle); clean
  digital drawings are the sure thing.

### 2.3 Step-by-step

```bash
PY=/c/PySchool/3.10-32-bit/python.exe          # the 32-bit Python on this PC
cd "C:/Users/HP/Desktop/02 - ROBOTICS/ADVANCED ROBOT/simulator"

# A) Make a sample image to practise on (or skip and use your own clean PNG):
$PY make_test_track.py oval                     # simple loop  -> out/track_oval.png
$PY make_test_track.py junction                 # a fork       -> out/track_junction.png

# B) Plan a simple loop:
$PY track_planner.py plan out/track_oval.png
$PY track_planner.py plan out/track_oval.png --scale 10     # force 10 px = 1 cm

# C) Plan a fork — give START and END as PIXEL coordinates (column,row) read
#    straight off the image. Use '=' so the leading minus isn't misread:
$PY track_planner.py plan out/track_junction.png --start=100,600 --end=800,350

# D) Plan AND check it in the simulator:
$PY track_planner.py validate out/track_oval.png
```

### 2.4 Reading the output
- **`out/plan.svg`** — open in a browser. The chosen line is coloured
  **green = fast → red = slow**, junction nodes in blue, with `F`/`L`/`R` turn
  letters and green/red start/end markers.
- **`out/skeleton.png`** — debug view of the extracted 1-px centreline. If this
  looks broken/spurious, your image isn't clean enough.
- **Printed `path[]` + `pathDistances[]`** — the tokens to paste into
  `a_pins_vars.ino` (maze firmware only).

### 2.5 IMPORTANT — the path is a DRAFT
The planner computes geometry, route, turns and speeds. It **cannot** know the
firmware's `B`/`W` colour mode or `C` distance/ultrasonic conditions from a plain
line drawing — those default to `B` and are left out. **Review before flashing.**

### 2.6 Planner tuning knobs (top of `track_planner.py`, `PLAN` dict)
- `line_width_cm` — your tape width (used for scale).
- `a_lat_cmps2` — how aggressively it takes curves (lower = safer/slower).
- `a_long_cmps2` — accel/brake limit.
- `v_at_pwm255`, `pwm_level1/3` — drivetrain + the two firmware speed levels.
- `turn_penalty_s` — how much each junction decision "costs" when routing.

Full reference: `simulator/TRACK_PLANNER.md`.

### 2.7 Tune the reactive follower on YOUR track image (`--image`)
This is the link between an image and **SimpleLineFollower**: load your real track
into the simulator and find its best speed/gains *before* touching hardware. Same
vision pipeline as the planner, but it drives the reactive follower on the shape.

```bash
PY=/c/PySchool/3.10-32-bit/python.exe
cd "C:/Users/HP/Desktop/02 - ROBOTICS/ADVANCED ROBOT/simulator"

# best KP/KD on your track:
$PY robot_sim.py autotune --form simple --image my_track.png --scale 10 --speed 150
# max smooth BASE_SPEED on your track:
$PY robot_sim.py sweep    --form simple --image my_track.png --scale 10 --kp 0.030 --kd 1.2
# watch it drive your track:
$PY robot_sim.py animate  --form simple --image my_track.png --scale 10 --speed 200 --kp 0.030 --kd 1.2
# a fork: pick the branch with start/end PIXELS (col,row); '=' guards the minus:
$PY robot_sim.py plot     --form simple --image out/track_junction.png --scale 10 --start=100,600 --end=800,350
```
Tips: pass `--scale` (px per cm) for accurate distances — auto-inference is
approximate. `--invert 1|0` if the auto dark/light guess is wrong. An **open**
route (a line with two ends) finishes by "losing the line" at the end — that's
the track ending, not a tuning problem; use a **loop** image for clean lap stats.

---

## 3. Which path should I use?

- **"I just want it to follow my track fast and smooth."**
  → Use **SimpleLineFollower**. No image needed. Tune `BASE_SPEED` (and `KD` if
  the loop time isn't ~2 ms). Done.

- **"My track has forks and the robot must pick a route / do a maze."**
  → Use the **maze firmware** + `track_planner.py` to generate the path string
  from an image. Review the draft, set colour/conditions, flash.

- **"I want to know the best speed/gains for THIS specific track before testing."**
  → Load the track from its image straight into the simulator with `--image` and
  tune the reactive follower on it (see §2.7).

---

## 4. Command cheat-sheet

```bash
PY=/c/PySchool/3.10-32-bit/python.exe
cd "C:/Users/HP/Desktop/02 - ROBOTICS/ADVANCED ROBOT/simulator"

# Tune the reactive follower (SimpleLineFollower form):
$PY robot_sim.py autotune --form simple --dt 2 --speed 150 --track hard
$PY robot_sim.py sweep    --form simple --dt 2 --kp 0.030 --kd 1.2 --track hard
$PY robot_sim.py animate  --form simple --dt 2 --speed 200 --kp 0.030 --kd 1.2 --track hard

# Tune the reactive follower on a REAL track image:
$PY robot_sim.py autotune --form simple --image my_track.png --scale 10 --speed 150
$PY robot_sim.py animate  --form simple --image my_track.png --scale 10 --speed 200 --kp 0.030 --kd 1.2

# Image -> plan (maze firmware):
$PY make_test_track.py oval
$PY track_planner.py plan     out/track_oval.png --scale 10
$PY track_planner.py validate out/track_oval.png
```

Open any `out/*.svg` in a web browser (double-click) to see the result.
