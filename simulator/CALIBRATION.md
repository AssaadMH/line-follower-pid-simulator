# Calibrating the simulator to your real robot

The simulator's tuning only transfers to hardware if the `CFG` block in
`robot_sim.py` matches your real robot. There are **9 constants**. Most take a
ruler; three need a 30-second test on the robot (use `calib_helper.ino`).

Fill in the right-hand column, then give me the numbers and I'll update `CFG`.

## A) Ruler measurements (robot powered OFF) — ~2 min

| CFG constant      | How to measure                                                        | Your value |
|-------------------|-----------------------------------------------------------------------|-----------|
| `wheelbase_cm`    | Distance between the two drive wheels (center of one tyre to the other) |           |
| `sensor_ahead_cm` | From the wheel **axle line** forward to the **row of QTR sensors**      |           |
| `sensor_pitch_cm` | Measure across all 16 sensors (first to last), divide by 15. (Or from the QTR datasheet.) |           |
| `line_width_cm`   | Width of the actual track line/tape                                    |           |

## B) On-robot tests (use `calib_helper.ino`) — ~3 min

1. Open your project in Arduino IDE, open `calib_helper.ino` from this folder,
   copy the whole block, and paste it **above `setup()`** in `y_setup.ino`
   (or any `.ino`). Then add `calibrate_all();` as the **last line of `setup()`**.
2. Upload, open Serial Monitor at **115200 baud**, and follow the prompts.
3. Record:

| CFG constant        | Comes from            | Your value |
|---------------------|-----------------------|-----------|
| `loop_dt_ms`        | test [2] prints it    |           |
| `SetPoint`          | test [3]: the steady `position` when the robot is centered on the line |           |
| `v_at_pwm255_cmps`  | test [1]: cm the robot travels in 1.0 s at full speed |           |

4. **Remove the helper block and the `calibrate_all();` line** when done.

## C) Leave at defaults (fine to skip)

| CFG constant     | Default | Note |
|------------------|---------|------|
| `latency_ms`     | 6.0     | Set roughly equal to your `loop_dt_ms`. Minor. |
| `motor_tau_s`    | 0.06    | Motor inertia. Hard to measure without encoders; default is reasonable for small geared motors. |
| `sensor_noise`   | 25.0    | Qualitative. Optional: with the robot still on the line, watch how much `position` jitters in test [3]; bigger jitter -> raise this. |

## Then
Give me your filled-in values and I'll write them into `robot_sim.py`'s `CFG`,
then re-run `autotune` so the recommended Kp/Kd are tuned for YOUR robot.
