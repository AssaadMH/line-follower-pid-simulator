"""
make_test_track.py -- generate clean digital track images to exercise
track_planner.py without needing the real competition image yet.

Produces black line on white background (the common case), at a known
scale so we can check the planner's px->cm inference.

    python make_test_track.py oval      out/track_oval.png
    python make_test_track.py junction  out/track_junction.png

Scale baked in: PX_PER_CM. Line width LINE_W_CM -> thickness in px.
"""
import sys, os, math
import numpy as np
import cv2

PX_PER_CM = 10.0          # 10 px = 1 cm
LINE_W_CM = 1.9           # tape width
W_PX, H_PX = 900, 700


def _canvas():
    return np.full((H_PX, W_PX), 255, np.uint8)   # white


def _draw_polyline(img, pts_cm):
    th = max(1, int(round(LINE_W_CM * PX_PER_CM)))
    pts = (np.array(pts_cm) * PX_PER_CM).astype(np.int32)
    pts[:, 0] += W_PX // 2          # centre origin
    pts[:, 1] = H_PX // 2 - pts[:, 1]
    cv2.polylines(img, [pts], False, 0, th, cv2.LINE_AA)


def oval():
    img = _canvas()
    pts = []
    straight, radius = 30.0, 20.0     # cm
    for x in np.linspace(-straight/2, straight/2, 120): pts.append((x, -radius))
    for a in np.linspace(-math.pi/2, math.pi/2, 120):
        pts.append((straight/2 + radius*math.cos(a), radius*math.sin(a)))
    for x in np.linspace(straight/2, -straight/2, 120): pts.append((x, radius))
    for a in np.linspace(math.pi/2, 3*math.pi/2, 120):
        pts.append((-straight/2 + radius*math.cos(a), radius*math.sin(a)))
    pts.append(pts[0])
    _draw_polyline(img, pts)
    return img


def junction():
    """A main line that forks: a short branch and a long branch rejoin.
    Tests route choice (the planner should prefer the faster branch)."""
    img = _canvas()
    # main stem bottom -> fork
    _draw_polyline(img, [(-35, -25), (-35, 0)])
    # left branch: long way around (up, across, down) -> slow
    _draw_polyline(img, [(-35, 0), (-35, 25), (0, 25)])
    # right branch: short diagonal -> fast
    _draw_polyline(img, [(-35, 0), (-10, 12), (0, 25)])
    # after rejoin: head right to the end
    _draw_polyline(img, [(0, 25), (35, 25), (35, 0)])
    return img


def serpentine():
    """Clean, planner-compatible version of the lf.png layout: a single
    continuous line snaking START(top-left) -> FINISH(bottom-right) over three
    horizontal runs joined by U-turns, the last run a smooth wave. NO dashes,
    NO crossings, NO parallel doubles, single width -> one connected skeleton."""
    img = _canvas()
    xL, xR = -30.0, 30.0
    y1, y2, y3 = 24.0, 2.0, -20.0
    r = (y1 - y2) / 2.0                      # U-turn radius = half the row gap
    pts = []

    # Run 1: top, left -> right
    for x in np.linspace(xL, xR, 200): pts.append((x, y1))
    # U-turn on the right, y1 -> y2 (bulges right)
    cy = (y1 + y2) / 2.0
    for a in np.linspace(math.pi/2, -math.pi/2, 120):
        pts.append((xR + r*math.cos(a), cy + r*math.sin(a)))
    # Run 2: middle, right -> left
    for x in np.linspace(xR, xL, 200): pts.append((x, y2))
    # U-turn on the left, y2 -> y3 (bulges left)
    cy = (y2 + y3) / 2.0
    for a in np.linspace(math.pi/2, 3*math.pi/2, 120):
        pts.append((xL + r*math.cos(a), cy + r*math.sin(a)))
    # Run 3: bottom WAVE, left -> right, ending at FINISH.
    # amp/period overridable from CLI (argv[3], argv[4]) for sweeping.
    # sim-swept for lowest tracking error at constant pwm140: amp/period below
    # follow stably at 7.4 cm max error (beats straight baseline + tight wave).
    amp    = float(sys.argv[3]) if len(sys.argv) > 3 else 2.5
    period = float(sys.argv[4]) if len(sys.argv) > 4 else 30.0
    for x in np.linspace(xL, xR, 360):
        pts.append((x, y3 + amp*math.sin(2*math.pi*(x - xL)/period)))
    _draw_polyline(img, pts)

    # report the START/END pixel coords for the planner (col,row)
    sx, sy = pts[0];  ex, ey = pts[-1]
    to_px = lambda cx, cy: (int(round(cx*PX_PER_CM + W_PX//2)),
                            int(round(H_PX//2 - cy*PX_PER_CM)))
    print(f"  START px {to_px(sx, sy)}   FINISH px {to_px(ex, ey)}")
    return img


def _lf_centerline():
    """The single continuous drivable centerline that traces lf.png feature-for-
    feature (START loop -> triangle -> [dashed->solid] -> bar -> right U-turn ->
    parallel '==' -> square -> left U-turn -> wave -> FINISH). Crossings smoothed
    to non-crossing curves; dashed gap closed. Returns list of (x_cm, y_cm)."""
    P = []
    def L(x0, y0, x1, y1, n=40):
        for t in np.linspace(0, 1, n): P.append((x0 + (x1-x0)*t, y0 + (y1-y0)*t))
    def A(cx, cy, r, a0, a1, n=60):
        for a in np.radians(np.linspace(a0, a1, n)): P.append((cx + r*math.cos(a), cy + r*math.sin(a)))
    def Wv(x0, x1, yc, amp, period, n=240):
        for x in np.linspace(x0, x1, n): P.append((x, yc + amp*math.sin(2*math.pi*(x-x0)/period)))

    # --- Row 1 (y~26): START -> right, teardrop LOOP, return left, drop to row 2
    L(-30, 26, 12, 26)                 # out (from START)
    A(16, 26, 5, 170, 170+310)         # teardrop loop (non-crossing, ~310 deg)
    L(13, 22, -30, 22)                 # return left (lower strand)
    L(-30, 22, -30, 8)                 # drop down to row 2 (left)
    # --- Row 2 (y~8) L->R: triangle, dashed(solid), bar, to right U-turn
    L(-30, 8, -24, 8)
    L(-24, 8, -20, 12); L(-20, 12, -16, 8)     # triangle (chevron)
    L(-16, 8, -2, 8)                   # dashed region -> drawn solid
    L(-2, 8, 8, 8)                     # bar zone (marker drawn on top)
    L(8, 8, 26, 8)
    A(26, -0.5, 8.5, 90, -90)          # right U-turn down to row 3
    # --- Row 3 (y~-9) R->L: parallel '==' switchback, square, to left U-turn
    L(26, -9, -4, -9)                  # main pass (upper of the '==')
    A(-4, -11.5, 2.5, 90, -90)         # dip down
    L(-4, -14, -16, -14)               # parallel lower line ('==')
    A(-16, -11.5, 2.5, -90, -270)      # back up
    L(-16, -9, -23, -9)
    L(-23, -9, -23, -14); L(-23, -14, -30, -14); L(-30, -14, -30, -9)   # square
    L(-30, -9, -32, -9)
    A(-32, -17.5, 8.5, 90, 270)        # left U-turn down to row 4
    # --- Row 4 (y~-26) L->R: wave to FINISH (bottom-right)
    Wv(-32, 22, -26, 2.5, 30)
    L(22, -26, 27, -26)
    return P


def lftrack(decor=True):
    """Clean, planner-compatible re-creation of lf.png. decor=True adds the
    START box, FINISH squares and the wide bar (for viewing); decor=False emits
    just the centerline (for the planner, so markers don't fragment the skeleton)."""
    img = _canvas()
    _draw_polyline(img, _lf_centerline())
    if decor:
        def box(xc, yc, w, h, fill):
            x0 = int(xc*PX_PER_CM + W_PX//2 - w*PX_PER_CM/2)
            y0 = int(H_PX//2 - yc*PX_PER_CM - h*PX_PER_CM/2)
            cv2.rectangle(img, (x0, y0), (x0+int(w*PX_PER_CM), y0+int(h*PX_PER_CM)),
                          0, -1 if fill else max(1, int(LINE_W_CM*PX_PER_CM)))
        box(-33, 26, 5, 5, False)          # START box outline (top-left)
        box(3, 8, 5, 2.2, True)            # wide bar marker (row 2)
        box(29.5, -26, 2, 2, True); box(33, -26, 2, 2, True)   # FINISH squares
    return img


def main():
    kind = sys.argv[1] if len(sys.argv) > 1 else "oval"
    out = sys.argv[2] if len(sys.argv) > 2 else f"out/track_{kind}.png"
    os.makedirs(os.path.dirname(out) or ".", exist_ok=True)
    if kind == "lf":
        cv2.imwrite("out/track_lf_view.png", lftrack(decor=True))
        cv2.imwrite("out/track_lf_plan.png", lftrack(decor=False))
        P = _lf_centerline()
        to_px = lambda c: (int(round(c[0]*PX_PER_CM + W_PX//2)), int(round(H_PX//2 - c[1]*PX_PER_CM)))
        print(f"wrote out/track_lf_view.png + out/track_lf_plan.png  ({W_PX}x{H_PX})")
        print(f"  START px {to_px(P[0])}   FINISH px {to_px(P[-1])}")
        return
    img = {"oval": oval, "junction": junction, "serpentine": serpentine}[kind]()
    cv2.imwrite(out, img)
    print(f"wrote {out}  ({W_PX}x{H_PX}, {PX_PER_CM} px/cm, line {LINE_W_CM}cm)")


if __name__ == "__main__":
    main()
