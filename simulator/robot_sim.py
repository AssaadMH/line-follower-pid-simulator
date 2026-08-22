"""
robot_sim.py  --  2D simulator for the SUIVEUR LASSAAD line-follower robot.

Goal: tune the line-following PID (Kp/Kd) and push the maximum speed at which
the robot still follows the line smoothly -- WITHOUT risking the real hardware.

It faithfully ports the controller from ea_PIDlineFollow.ino:

    p     = position - SetPoint
    i     = prev_i + error
    d     = (error - prev_error) / elapsedTime_ms
    pid   = Kp*p + Ki*i + Kd*d
    Lspd  = (maxSpeed - TURNFACTOR*|pid|) + pid     # -> ENA (left wheel)
    Rspd  = (maxSpeed - TURNFACTOR*|pid|) - pid     # -> ENB (right wheel)

and a 16-sensor QTR "readLineBlack" model (weighted position, 0..15000).

USAGE
    python robot_sim.py plot   --speed 180 --kp 0.019 --kd 5
    python robot_sim.py sweep                      # find max stable speed
    python robot_sim.py animate --speed 180 --kp 0.019 --kd 5

All physical constants are in the CONFIG block -- calibrate them to your real
robot for the tuning to transfer (see notes there).
"""

import argparse, math, os
import numpy as np

# ----------------------------------------------------------------------------
# CONFIG  --  >>> CALIBRATE THESE TO YOUR REAL ROBOT <<<
# ----------------------------------------------------------------------------
CFG = dict(
    # --- robot geometry (centimetres) ---
    wheelbase_cm   = 18.0,   # CALIBRATED 2026-06-22: measured on the real robot
    sensor_ahead_cm= 16.0,   # CALIBRATED: measured 15-17cm, midpoint used
    sensor_pitch_cm= 0.8,    # CALIBRATED: ruler across 16 sensors / 15
    line_width_cm  = 4.0,    # CALIBRATED: actual track tape width

    # --- drivetrain ---
    # Linear wheel speed (cm/s) at full PWM = 255. MEASURE this on the real robot:
    # run a wheel at PWM 255 for 1s and see how far it travels.
    v_at_pwm255_cmps = 230.0,

    # --- controller (matches the firmware) ---
    SetPoint   = 7500.0,     # centre of a 16-sensor bar = (16-1)*1000/2 = 7500
    #                          (firmware uses 9700 -> an intentional offset; set
    #                           this to whatever 'centred on the line' reads on
    #                           your robot so sim and reality agree)
    Ki         = 0.0,
    TURNFACTOR = 1.0,

    # --- simulation ---
    loop_dt_ms = 5.0,        # control-loop period (ms). Kd depends on this!
    sim_time_s = 30.0,       # max seconds to simulate before giving up
    off_track_cm = 9.0,      # if the line is farther than this from bar centre
    #                          for too long -> counted as "lost the line"
    motor_tau_s = 0.06,      # motor/inertia time constant: wheels approach the
    #                          commanded speed over ~this long (real, damps wobble)
    phys_substeps = 10,      # integrate physics this many times per control step
    #                          (stops one correction from over-rotating in a step)

    # --- realism (set to 0 for the old idealized behaviour) ---
    sensor_noise = 25.0,     # std-dev of per-sensor analog noise (0..1000 scale)
    latency_ms   = 6.0,      # delay between reading sensors and the motors reacting
    #                          (sensor read + ultrasonic/gyro polling + compute)
    noise_seed   = 1234,     # fixed seed -> repeatable sweeps/comparisons
)

PWM_MAX = 255
N_SENS  = 16

# ----------------------------------------------------------------------------
# TRACK  --  a closed loop: two straights joined by two semicircles (an oval),
#            plus an optional S-curve to stress the controller.
# ----------------------------------------------------------------------------
def make_track(kind="oval"):
    pts = []
    if kind == "oval":
        straight = 120.0      # cm
        radius   = 45.0       # cm
        # bottom straight (left -> right)
        for x in np.linspace(-straight/2, straight/2, 200):
            pts.append((x, -radius))
        # right semicircle (bottom -> top)
        for a in np.linspace(-math.pi/2, math.pi/2, 200):
            pts.append((straight/2 + radius*math.cos(a), radius*math.sin(a)))
        # top straight (right -> left)
        for x in np.linspace(straight/2, -straight/2, 200):
            pts.append((x, radius))
        # left semicircle (top -> bottom)
        for a in np.linspace(math.pi/2, 3*math.pi/2, 200):
            pts.append((-straight/2 + radius*math.cos(a), radius*math.sin(a)))
    elif kind == "scurve":
        for x in np.linspace(0, 400, 800):
            pts.append((x, 40.0*math.sin(x/40.0)))
    elif kind == "hard":
        # rounded rectangle with TIGHT corners (radius 18) -> stresses the PID
        W, H, R = 90.0, 55.0, 18.0
        def arc(cx, cy, a0, a1):
            for a in np.linspace(a0, a1, 120):
                pts.append((cx + R*math.cos(a), cy + R*math.sin(a)))
        for x in np.linspace(-W+R, W-R, 200): pts.append((x, -H))   # bottom
        arc(W-R, -H+R, -math.pi/2, 0)                               # BR
        for y in np.linspace(-H+R, H-R, 200): pts.append((W, y))    # right
        arc(W-R, H-R, 0, math.pi/2)                                 # TR
        for x in np.linspace(W-R, -W+R, 200): pts.append((x, H))    # top
        arc(-W+R, H-R, math.pi/2, math.pi)                          # TL
        for y in np.linspace(H-R, -H+R, 200): pts.append((-W, y))   # left
        arc(-W+R, -H+R, math.pi, 3*math.pi/2)                       # BL
    elif kind == "chicane":
        # straight with tightening S-bends
        for x in np.linspace(0, 300, 900):
            pts.append((x, 25.0*math.sin(x/25.0)))
    return resample(np.array(pts), 0.4)

def resample(pts, spacing_cm):
    """Re-space a polyline to ~uniform fine spacing (smooths the sensor model)."""
    seg = np.hypot(np.diff(pts[:, 0]), np.diff(pts[:, 1]))
    s = np.concatenate([[0], np.cumsum(seg)])
    n = max(2, int(s[-1] / spacing_cm))
    su = np.linspace(0, s[-1], n)
    x = np.interp(su, s, pts[:, 0]); y = np.interp(su, s, pts[:, 1])
    return np.column_stack([x, y])

def _parse_px(s):
    """'col,row' pixel string -> (col, row) floats, or None."""
    if not s:
        return None
    a, b = s.split(",")
    return (float(a), float(b))

def track_from_image(path, scale=None, invert=None, start=None, end=None):
    """Load a REAL track from an image and return its centreline as a cm polyline
    the simulator can drive -- so you can tune the reactive follower on YOUR track.

    Reuses track_planner's vision pipeline (OpenCV): image -> line mask ->
    1px skeleton -> graph -> ordered route. Works for a closed loop, an open
    line, or a fork (pass --start/--end pixel coords to pick the branch).
    Returns (track_cm, info_dict)."""
    try:
        import track_planner as tp          # lazy: only needs cv2 when --image used
    except Exception as e:
        raise SystemExit(f"--image needs track_planner.py + OpenCV: {e}")
    mask, gray = tp.load_mask(path, invert)
    skel = tp.zhang_suen(mask)
    sc = scale or tp.infer_px_per_cm(mask, skel, tp.PLAN["line_width_cm"])
    if not sc or sc <= 0:
        raise SystemExit("could not infer px/cm from the image; pass --scale")
    skel = tp.prune_spurs(skel, int(round(1.2 * tp.PLAN["line_width_cm"] * sc)))
    g = tp.build_graph(skel, tp.PLAN["node_merge_px"])
    seq, node_seq, kind = tp.choose_route(g, sc, start, end)
    route_cm = tp.assemble_route(seq, sc)
    track = resample(route_cm, 0.4)
    info = dict(kind=kind, scale=sc, nodes=len(g.nodes), edges=len(g.edges),
                length_cm=float(np.hypot(np.diff(track[:, 0]),
                                         np.diff(track[:, 1])).sum()))
    return track, info

# ----------------------------------------------------------------------------
# SENSOR MODEL  --  QTR readLineBlack equivalent
# ----------------------------------------------------------------------------
class SensorBar:
    WIN = 200                          # track points to search each side of the hint

    def __init__(self, track, cfg, rng=None):
        self.track = track
        self.N = len(track)
        self.cfg = cfg
        self.last_position = cfg["SetPoint"]
        self.rng = rng
        self.hint = 0                  # index of last nearest track point

    def read(self, bar_center, right_vec):
        cfg = self.cfg
        pitch = cfg["sensor_pitch_cm"]
        half_line = cfg["line_width_cm"] / 2.0
        idx = np.arange(N_SENS)
        offsets = (idx - (N_SENS - 1) / 2.0) * pitch          # left(-) .. right(+)
        sx = bar_center[0] + offsets * right_vec[0]
        sy = bar_center[1] + offsets * right_vec[1]
        # only search track points near the robot (windowed, wraps for loops)
        win = np.arange(self.hint - self.WIN, self.hint + self.WIN) % self.N
        tw = self.track[win]                                  # (W,2)
        # advance the hint to the track point nearest the bar centre
        bc = (tw[:, 0] - bar_center[0]) ** 2 + (tw[:, 1] - bar_center[1]) ** 2
        self.hint = int(win[bc.argmin()])
        # TRUE tracking error = distance from the line to the bar CENTRE (the
        # robot's reference point), not the nearest single sensor.
        center_err = float(math.sqrt(bc.min()))
        # distance from each sensor to nearest point on the line (within window)
        dx = tw[:, 0][None, :] - sx[:, None]
        dy = tw[:, 1][None, :] - sy[:, None]
        dmin = np.sqrt(dx * dx + dy * dy).min(axis=1)         # (16,)
        # analog "blackness": 1000 on the line, linear falloff to 0 one line-width out
        black = np.clip(1000.0 * (1.0 - (dmin - half_line) / half_line), 0, 1000)
        # realism: per-sensor analog noise
        if self.rng is not None and cfg["sensor_noise"] > 0:
            black = np.clip(black + self.rng.normal(0, cfg["sensor_noise"], N_SENS), 0, 1000)
        total = black.sum()
        on_line = dmin.min() < (half_line + pitch)            # any sensor sees it?
        if total < 50:                                        # line lost
            # QTR AUTO behaviour: assume line is just past the last-seen edge
            pos = 0.0 if self.last_position < cfg["SetPoint"] else (N_SENS - 1) * 1000.0
            return pos, on_line, center_err
        pos = float((black * idx * 1000.0).sum() / total)
        self.last_position = pos
        return pos, on_line, center_err

# ----------------------------------------------------------------------------
# PID CONTROLLER  --  faithful port of pidfollow()
# ----------------------------------------------------------------------------
class PID:
    def __init__(self, kp, kd, cfg):
        self.kp, self.kd = kp, kd
        self.ki = cfg["Ki"]
        self.setpoint = cfg["SetPoint"]
        self.turnfactor = cfg["TURNFACTOR"]
        self.dt_ms = cfg["loop_dt_ms"]
        self.prev_error = 0.0
        self.prev_i = 0.0

    def step(self, position, max_speed):
        p = position - self.setpoint
        error = p
        i = self.prev_i + error
        d = (error - self.prev_error) / self.dt_ms
        pid = self.kp * p + self.ki * i + self.kd * d
        self.prev_i = i
        self.prev_error = error
        base = max_speed - self.turnfactor * abs(pid)
        left  = base + pid          # ENA / left wheel
        right = base - pid          # ENB / right wheel
        left  = max(-PWM_MAX, min(PWM_MAX, left))
        right = max(-PWM_MAX, min(PWM_MAX, right))
        return left, right, pid

# ----------------------------------------------------------------------------
# PID CONTROLLER (SIMPLE)  --  matches SimpleLineFollower/SimpleLineFollower.ino
#   error      = position - CENTER
#   integral  += error            (clamped, anti-windup)
#   derivative = error - lastError    <-- PER-LOOP, NOT divided by dt
#   pid        = KP*error + KI*integral + KD*derivative
#   base       = max(0, max_speed - TURN_SLOW*|pid|)
# Because the derivative is per-loop (not /dt), KD here is on a DIFFERENT scale
# than the firmware's Kd, and it depends on the loop period -- tune at the loop
# rate the real sketch actually runs at (set via --dt).
# ----------------------------------------------------------------------------
class PIDSimple:
    def __init__(self, kp, kd, cfg):
        self.kp, self.kd = kp, kd
        self.ki = cfg["Ki"]
        self.setpoint = cfg["SetPoint"]
        self.turnfactor = cfg["TURNFACTOR"]      # = TURN_SLOW in the sketch
        self.prev_error = 0.0
        self.integral = 0.0

    def step(self, position, max_speed):
        error = position - self.setpoint
        self.integral += error
        self.integral = max(-20000.0, min(20000.0, self.integral))   # anti-windup
        derivative = error - self.prev_error
        pid = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.prev_error = error
        base = max_speed - self.turnfactor * abs(pid)
        if base < 0:
            base = 0
        left  = base + pid
        right = base - pid
        left  = max(-PWM_MAX, min(PWM_MAX, left))
        right = max(-PWM_MAX, min(PWM_MAX, right))
        return left, right, pid

# ----------------------------------------------------------------------------
# SIMULATION
# ----------------------------------------------------------------------------
def simulate(speed, kp, kd, cfg, track, record=False, form="firmware"):
    rng = np.random.default_rng(cfg["noise_seed"])
    bar = SensorBar(track, cfg, rng)
    pid = PIDSimple(kp, kd, cfg) if form == "simple" else PID(kp, kd, cfg)
    # control latency: motors react to a sensor reading from `lat_steps` loops ago
    lat_steps = max(0, int(round(cfg["latency_ms"] / cfg["loop_dt_ms"])))
    pos_delay = [cfg["SetPoint"]] * (lat_steps + 1)

    # start on the track at its first point, heading along the initial tangent
    th = math.atan2(track[1, 1] - track[0, 1], track[1, 0] - track[0, 0])
    x, y = float(track[0, 0]), float(track[0, 1])
    dt = cfg["loop_dt_ms"] / 1000.0
    vmax = cfg["v_at_pwm255_cmps"]
    L = cfg["wheelbase_cm"]

    steps = int(cfg["sim_time_s"] / dt)
    off_time = 0.0
    max_err = 0.0
    dist_travelled = 0.0
    start = np.array([x, y])
    left_start_zone = False
    trail = []

    vL_act = vR_act = 0.0                          # actual wheel speeds (cm/s)
    nsub = max(1, int(cfg["phys_substeps"]))
    sub_dt = dt / nsub
    tau = cfg["motor_tau_s"]

    for _ in range(steps):
        h = np.array([math.cos(th), math.sin(th)])
        right_vec = np.array([math.sin(th), -math.cos(th)])
        bar_center = np.array([x, y]) + cfg["sensor_ahead_cm"] * h

        position, on_line, derr = bar.read(bar_center, right_vec)
        max_err = max(max_err, derr)
        off_time = off_time + dt if derr > cfg["off_track_cm"] else 0.0
        if off_time > 0.25:                       # lost the line for >250ms
            return dict(ok=False, reason="lost line", time=_ * dt,
                        max_err=max_err, trail=np.array(trail) if record else None)

        # apply control latency: use the sensor reading from lat_steps ago
        pos_delay.append(position)
        delayed_pos = pos_delay.pop(0)
        lpwm, rpwm, _pid = pid.step(delayed_pos, speed)
        vL_cmd = (lpwm / PWM_MAX) * vmax
        vR_cmd = (rpwm / PWM_MAX) * vmax

        # integrate physics in sub-steps with first-order motor lag (inertia)
        for _s in range(nsub):
            k = sub_dt / max(tau, 1e-6)
            vL_act += (vL_cmd - vL_act) * min(k, 1.0)
            vR_act += (vR_cmd - vR_act) * min(k, 1.0)
            v = (vL_act + vR_act) / 2.0
            omega = (vR_act - vL_act) / L
            x += v * math.cos(th) * sub_dt
            y += v * math.sin(th) * sub_dt
            th += omega * sub_dt
            dist_travelled += abs(v) * sub_dt
        if record:
            trail.append((x, y))

        # lap detection: left the start zone, then came back to it
        d_start = math.hypot(x - start[0], y - start[1])
        if d_start > 60:
            left_start_zone = True
        if left_start_zone and d_start < 8 and dist_travelled > 100:
            return dict(ok=True, reason="lap complete", time=_ * dt,
                        max_err=max_err, trail=np.array(trail) if record else None)

    return dict(ok=True, reason="time up (stable)", time=cfg["sim_time_s"],
                max_err=max_err, trail=np.array(trail) if record else None)

def cfg_radius(track):
    # the oval's bottom straight y = -radius
    return abs(track[0][1])

# ----------------------------------------------------------------------------
# MODES
# ----------------------------------------------------------------------------
def out_dir():
    d = os.path.join(os.path.dirname(os.path.abspath(__file__)), "out")
    os.makedirs(d, exist_ok=True)
    return d

def write_svg(track, trail, fname, title, line_w_cm):
    """Dependency-free trajectory plot. Opens in any web browser."""
    allx = np.concatenate([track[:, 0], trail[:, 0]]) if len(trail) else track[:, 0]
    ally = np.concatenate([track[:, 1], trail[:, 1]]) if len(trail) else track[:, 1]
    pad = 15.0
    minx, maxx = allx.min() - pad, allx.max() + pad
    miny, maxy = ally.min() - pad, ally.max() + pad
    W, H = 900, 620
    sx = W / (maxx - minx); sy = H / (maxy - miny); s = min(sx, sy)
    def X(x): return (x - minx) * s
    def Y(y): return H - (y - miny) * s          # flip y so +y is up
    def poly(pts, color, width, op=1.0):
        d = " ".join(f"{X(px):.1f},{Y(py):.1f}" for px, py in pts)
        return (f'<polyline points="{d}" fill="none" stroke="{color}" '
                f'stroke-width="{width}" stroke-opacity="{op}" '
                f'stroke-linejoin="round"/>')
    svg = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{W}" height="{H+40}" '
           f'viewBox="0 0 {W} {H+40}" font-family="sans-serif">',
           f'<rect width="{W}" height="{H+40}" fill="white"/>',
           f'<text x="10" y="22" font-size="16">{title}</text>',
           f'<g transform="translate(0,30)">',
           poly(track, "#bbbbbb", max(2, line_w_cm * s), 0.9)]   # track
    if len(trail):
        svg.append(poly(trail, "#1f6feb", 2.0))                  # robot path
        svg.append(f'<circle cx="{X(trail[0,0]):.1f}" cy="{Y(trail[0,1]):.1f}" '
                   f'r="6" fill="green"/>')                      # start
        svg.append(f'<rect x="{X(trail[-1,0])-5:.1f}" y="{Y(trail[-1,1])-5:.1f}" '
                   f'width="10" height="10" fill="red"/>')       # end
    svg.append("</g></svg>")
    with open(fname, "w") as f:
        f.write("\n".join(svg))

def mode_plot(args, cfg, track):
    r = simulate(args.speed, args.kp, args.kd, cfg, track, record=True, form=args.form)
    trail = r["trail"] if r["trail"] is not None else np.empty((0, 2))
    title = (f"speed={args.speed} Kp={args.kp} Kd={args.kd}  ->  {r['reason']}  "
             f"(max wobble {r['max_err']:.1f}cm, t={r['time']:.1f}s)")
    path = os.path.join(out_dir(), f"path_s{args.speed}_kp{args.kp}_kd{args.kd}.svg")
    write_svg(track, trail, path, title, cfg["line_width_cm"])
    print(f"{r['reason']:>18} | speed={args.speed:<4} Kp={args.kp} Kd={args.kd} "
          f"| max wobble={r['max_err']:.2f}cm | t={r['time']:.2f}s")
    print(f"saved -> {path}")

def mode_sweep(args, cfg, track):
    print("Sweeping max speed (PWM) at fixed gains to find the smooth limit...")
    print(f"  Kp={args.kp}  Kd={args.kd}  (wobble threshold = {args.maxwobble}cm)\n")
    print(f"  {'speed':>5} | {'result':>18} | {'max wobble':>10} | {'time':>6}")
    print("  " + "-" * 50)
    best = None
    for speed in range(args.lo, args.hi + 1, args.step):
        r = simulate(speed, args.kp, args.kd, cfg, track, form=args.form)
        smooth = r["ok"] and r["max_err"] <= args.maxwobble
        tag = "OK SMOOTH" if smooth else ("on line" if r["ok"] else "LOST")
        print(f"  {speed:>5} | {r['reason']:>18} | {r['max_err']:>8.2f}cm | {r['time']:>5.1f}s  {tag}")
        if smooth:
            best = speed
    print("\n  " + "-" * 50)
    if best is not None:
        print(f"  >>> Max SMOOTH speed at these gains: PWM {best} "
              f"(~{best/PWM_MAX*cfg['v_at_pwm255_cmps']:.0f} cm/s)")
    else:
        print("  No speed stayed under the wobble threshold -- lower it or retune Kp/Kd.")

def mode_autotune(args, cfg, track):
    kps = np.linspace(args.kp_lo, args.kp_hi, args.kp_n)
    kds = np.linspace(args.kd_lo, args.kd_hi, args.kd_n)
    print(f"Auto-tuning Kp/Kd on '{args.track}' track at speed {args.speed} "
          f"({kps.size}x{kds.size} = {kps.size*kds.size} runs)...")
    print(f"  goal: complete the lap with the SMALLEST wobble (= smoothest)\n")
    results = []
    print("           " + " ".join(f"Kd{kd:<4.1f}" for kd in kds))
    for kp in kps:
        row = []
        for kd in kds:
            r = simulate(args.speed, float(kp), float(kd), cfg, track, form=args.form)
            results.append((kp, kd, r["ok"], r["max_err"], r["reason"]))
            row.append(f"{r['max_err']:5.1f}" if r["ok"] else "  X  ")
        print(f"  Kp={kp:5.3f} | " + " ".join(row))
    print("\n  (numbers = max wobble in cm; X = lost the line)\n")

    ok = [r for r in results if r[2]]
    ok.sort(key=lambda r: r[3])
    print("  Top 5 gain sets (lowest wobble at this speed):")
    print(f"    {'Kp':>6} {'Kd':>6} {'wobble':>8}")
    for kp, kd, _o, err, _r in ok[:5]:
        print(f"    {kp:>6.3f} {kd:>6.2f} {err:>7.2f}cm")
    if ok:
        best = ok[0]
        sel = (f"--image {args.image}" + (f" --scale {args.scale}" if args.scale else "")
               if getattr(args, "image", None) else f"--track {args.track}")
        extra = f" --form {args.form}" + (f" --dt {args.dt}" if args.dt else "")
        print(f"\n  >>> BEST at speed {args.speed} on '{args.track}': "
              f"Kp={best[0]:.3f}  Kd={best[1]:.2f}  (wobble {best[3]:.2f}cm)")
        print(f"      verify/animate it:  python robot_sim.py plot "
              f"--speed {args.speed} --kp {best[0]:.3f} --kd {best[1]:.2f} {sel}{extra}")
    else:
        print("  No gain set completed the lap -- lower the speed or widen the grid.")

def mode_animate(args, cfg, track):
    """Dependency-free 'live' view: an animated SVG (a moving dot following the
    track). Opens in any browser, loops forever -- no matplotlib needed."""
    r = simulate(args.speed, args.kp, args.kd, cfg, track, record=True, form=args.form)
    trail = r["trail"] if r["trail"] is not None else np.empty((0, 2))
    if len(trail) == 0:
        print("nothing to animate"); return
    title = (f"speed={args.speed} Kp={args.kp} Kd={args.kd}  ->  {r['reason']}  "
             f"(max wobble {r['max_err']:.1f}cm, t={r['time']:.1f}s)")
    path = os.path.join(out_dir(), f"anim_s{args.speed}_kp{args.kp}_kd{args.kd}_{args.track}.svg")

    # coordinate transform (shared with write_svg)
    pad = 15.0
    allx = np.concatenate([track[:, 0], trail[:, 0]])
    ally = np.concatenate([track[:, 1], trail[:, 1]])
    minx, maxx = allx.min() - pad, allx.max() + pad
    miny, maxy = ally.min() - pad, ally.max() + pad
    W, H = 900, 620
    s = min(W / (maxx - minx), H / (maxy - miny))
    def X(x): return (x - minx) * s
    def Y(y): return H - (y - miny) * s
    def pts_str(p): return " ".join(f"{X(a):.1f},{Y(b):.1f}" for a, b in p)
    tr = trail[:: max(1, len(trail) // 400)]                      # ~400 pts for the path
    motion = "M " + " L ".join(f"{X(a):.1f},{Y(b):.1f}" for a, b in tr)
    dur = max(1.5, r["time"])                                     # loop ~ real lap time

    svg = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{W}" height="{H+40}" '
           f'viewBox="0 0 {W} {H+40}" font-family="sans-serif">',
           f'<rect width="{W}" height="{H+40}" fill="white"/>',
           f'<text x="10" y="22" font-size="15">{title}</text>',
           f'<g transform="translate(0,30)">',
           f'<polyline points="{pts_str(track)}" fill="none" stroke="#bbbbbb" '
           f'stroke-width="{max(2, cfg["line_width_cm"]*s):.1f}"/>',
           f'<polyline points="{pts_str(tr)}" fill="none" stroke="#1f6feb" '
           f'stroke-width="1.5" stroke-opacity="0.5"/>',
           f'<circle r="7" fill="red">',
           f'  <animateMotion dur="{dur:.1f}s" repeatCount="indefinite" '
           f'path="{motion}"/>',
           f'</circle>',
           "</g></svg>"]
    with open(path, "w") as f:
        f.write("\n".join(svg))
    print(f"{r['reason']:>18} | speed={args.speed} Kp={args.kp} Kd={args.kd} "
          f"| wobble={r['max_err']:.2f}cm")
    print(f"animated SVG saved -> {path}")
    print("Open it in your browser (double-click) to watch the robot loop the track.")

# ----------------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser(description="Line-follower PID simulator")
    sub = ap.add_subparsers(dest="mode", required=True)

    common = dict()
    for name in ("plot", "animate"):
        p = sub.add_parser(name)
        p.add_argument("--speed", type=int, default=180)
        p.add_argument("--kp", type=float, default=0.019)
        p.add_argument("--kd", type=float, default=5.0)
        p.add_argument("--track", default="oval")

    p = sub.add_parser("sweep")
    p.add_argument("--kp", type=float, default=0.019)
    p.add_argument("--kd", type=float, default=5.0)
    p.add_argument("--lo", type=int, default=80)
    p.add_argument("--hi", type=int, default=255)
    p.add_argument("--step", type=int, default=10)
    p.add_argument("--maxwobble", type=float, default=4.0)
    p.add_argument("--track", default="oval")

    p = sub.add_parser("autotune")
    p.add_argument("--speed", type=int, default=230)
    p.add_argument("--track", default="hard")
    p.add_argument("--kp_lo", type=float, default=0.006)
    p.add_argument("--kp_hi", type=float, default=0.030)
    p.add_argument("--kp_n", type=int, default=6)
    p.add_argument("--kd_lo", type=float, default=0.0)
    p.add_argument("--kd_hi", type=float, default=10.0)
    p.add_argument("--kd_n", type=int, default=6)

    # shared options on every sub-command
    for name, p in sub.choices.items():
        p.add_argument("--form", choices=["firmware", "simple"], default="firmware",
                       help="'firmware' = Kd*(d/dt); 'simple' = SimpleLineFollower (Kd*per-loop d)")
        p.add_argument("--dt", type=float, default=None,
                       help="override control-loop period in ms (default %.1f)" % CFG["loop_dt_ms"])
        p.add_argument("--turn", type=float, default=None,
                       help="override TURNFACTOR / TURN_SLOW corner-braking (default %.1f)" % CFG["TURNFACTOR"])
        p.add_argument("--image", default=None,
                       help="drive a REAL track loaded from an image (PNG/JPG) instead of a built-in shape")
        p.add_argument("--scale", type=float, default=None,
                       help="image px per cm (else inferred from line width)")
        p.add_argument("--invert", type=int, default=None,
                       help="image: 1=line is white, 0=line is dark (else auto)")
        p.add_argument("--start", default=None,
                       help="image fork: start 'col,row' in PIXELS")
        p.add_argument("--end", default=None,
                       help="image fork: end 'col,row' in PIXELS")

    args = ap.parse_args()
    cfg = dict(CFG)
    if args.dt   is not None: cfg["loop_dt_ms"] = args.dt
    if args.turn is not None: cfg["TURNFACTOR"] = args.turn
    if getattr(args, "image", None):
        track, info = track_from_image(args.image, args.scale, args.invert,
                                       _parse_px(args.start), _parse_px(args.end))
        # use the image filename as the track label for output titles/filenames
        args.track = os.path.splitext(os.path.basename(args.image))[0]
        print(f"[image] '{args.track}': {info['kind']}, {info['scale']:.2f} px/cm, "
              f"{info['nodes']} nodes / {info['edges']} edges, "
              f"length {info['length_cm']:.0f} cm")
    else:
        track = make_track(getattr(args, "track", "oval"))
    if args.mode == "plot":    mode_plot(args, cfg, track)
    elif args.mode == "sweep": mode_sweep(args, cfg, track)
    elif args.mode == "autotune": mode_autotune(args, cfg, track)
    elif args.mode == "animate": mode_animate(args, cfg, track)

if __name__ == "__main__":
    main()
