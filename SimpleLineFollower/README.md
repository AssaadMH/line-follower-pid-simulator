# SimpleLineFollower

A one-file, reusable PID line follower for the SUIVEUR LASSAAD hardware
(Arduino Mega 2560 + 16-channel QTR array + L298-style dual driver).

Unlike the full firmware, this needs **no path string** — it just follows
whatever line is under the sensor bar, so the same upload runs on **any track**.

## Dependency
Install **QTRSensors** (by Pololu) from Arduino IDE → Library Manager. Nothing else.
(No MPU6050 / Ultrasonic / FastLED / LCD needed.)

## Use it
1. Open `SimpleLineFollower.ino`. Set `LINE_IS_BLACK` (true = dark line on light floor).
2. Upload (Board: *Arduino Mega 2560*).
3. On power-up the on-board LED blinks ~8 s = **calibration**: sweep the sensor bar
   left↔right across the line so every sensor sees both line and floor.
4. LED goes solid = ready. Put the robot on the line, **press the button** → it follows.

## Tuning (only 3 knobs usually matter)
| Setting | Raise it if… | Lower it if… |
|---|---|---|
| `BASE_SPEED` | too slow | flies off on corners |
| `KP` | cuts corners / reacts late | wobbles / oscillates |
| `KD` | wobbles even after lowering KP | jittery on straights |

Start at `BASE_SPEED 120`, get it stable, then raise speed and re-tune KP/KD.

### Default gains are simulator-tuned
`KP 0.030 / KD 1.2` were found with `robot_sim.py` for **this** controller form
(per-loop derivative) — smoothest run on the tight "hard" track, stayed under
2 cm wobble at every speed up to PWM 255. Reproduce / retune:

```
python robot_sim.py autotune --form simple --dt 2 --speed 150 --track hard
python robot_sim.py sweep    --form simple --dt 2 --kp 0.030 --kd 1.2 --track hard
python robot_sim.py animate  --form simple --dt 2 --speed 150 --kp 0.030 --kd 1.2 --track hard
```

### Smooth-speed ceiling (BASE_SPEED)
Sweeps at KP 0.030 / KD 1.2 (`--dt 2`):
- Oval (R=45 cm): very smooth (<1.5 cm wobble) all the way to **PWM 255**.
- Hard (R=18 cm corners): very smooth to **PWM 205**; up to 255 it stays on the
  line but wobble grows to ~2 cm.
- It never lost the line at any speed — `TURN_SLOW` brakes the corners — so
  `BASE_SPEED` mostly sets STRAIGHT-line speed; corner speed = `TURN_SLOW`.

Recommended starting `BASE_SPEED`: **220–240** on open tracks, **180** on tight
ones (sim is optimistic, so keep ~15–20 % margin on real hardware).

### TURN_SLOW is a speed↔smoothness trade (not a speed boost)
Sweeping `--turn` at BASE_SPEED 255 on the hard track showed higher TURN_SLOW =
smoother corners but **slower** laps. The fastest lap = max BASE_SPEED + the
*lowest* TURN_SLOW you can tolerate:

| Style | BASE_SPEED | TURN_SLOW | Lap | Wobble |
|---|---|---|---|---|
| Very smooth | 255 | 1.5 | 5.8 s | 1.6 cm |
| **Balanced (default)** | **255** | **1.0** | **4.3 s** | **2.0 cm** |
| Aggressive | 255 | 0.7 | 4.0 s | 2.4 cm |

It never lost the line even at TURN_SLOW 0.3, so 1.0 is a safe, fast default. To
go faster, raise BASE_SPEED — don't raise TURN_SLOW.

Reproduce: `python robot_sim.py plot --form simple --dt 2 --turn 1.0 --speed 255 --kp 0.030 --kd 1.2 --track hard`

**Loop-rate caveat:** this form does NOT divide the derivative by dt, so KD's
strength scales with the loop period. Tuned at ~2 ms/loop. The sketch prints its
average loop time over Serial — if yours differs, rescale:
`KD = 1.2 * (2 ms / your_loop_ms)`.

## What it does on tricky track
- **Sharp corner / gap / line lost:** pivots toward the side it last saw the line
  (`SEARCH_SPEED`) until it re-acquires — no path programming required.
- **Corner braking:** automatically slows the base speed the harder it has to steer
  (`TURN_SLOW`), so it doesn't overshoot tight turns.
- **Inverted tracks:** flip `LINE_IS_BLACK` to follow a white line on a dark floor.

## Notes
- This does **not** replace the maze firmware in `a_pins_vars/` — it's a separate,
  simpler sketch for plain line-following on arbitrary tracks.
- Default gains are sane starting points; for hardware-matched tuning, calibrate the
  simulator (`simulator/CALIBRATION.md`) and sweep KP/KD there first.
