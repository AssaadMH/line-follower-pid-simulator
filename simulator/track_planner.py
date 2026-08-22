"""
track_planner.py -- turn a CLEAN top-down track image into the FASTEST plan
the SUIVEUR LASSAAD robot can actually drive, then validate it in robot_sim.

Pipeline:
  1. load image (OpenCV) -> binary line mask (auto dark/light line)
  2. skeletonize (Zhang-Suen) -> 1px centerline
  3. infer px/cm scale from the known line width (distance transform)
  4. build a graph: nodes = endpoints(deg 1)/junctions(deg>=3), edges = polylines
  5. pick the route:
       - pure loop            -> drive the whole loop
       - endpoints/junctions  -> fastest start->end route (Dijkstra over TIME),
                                 labelling each junction decision F / L / R
  6. speed profile: curvature-limited v(s) with accel/decel passes, then
     quantize to the robot's discrete speed levels -> place '1'/'3' tokens
  7. emit a DRAFT  path[]  +  pathDistances[]  (cm) to paste into a_pins_vars.ino
  8. render an SVG of the chosen line coloured by speed
  9. (validate) feed the cm centerline into robot_sim.simulate

USAGE
  python make_test_track.py oval                 # make a sample image first
  python track_planner.py plan out/track_oval.png
  python track_planner.py plan out/track_junction.png --start -35,-25 --end 35,0
  python track_planner.py plan IMG --scale 10    # force 10 px/cm (skip inference)
  python track_planner.py validate out/track_oval.png

Notes
  - "Clean digital drawing" assumed: solid line, plain background.
  - The emitted path-string is a DRAFT: speed tokens + turn decisions are filled
    in, but B/W colour mode and any 'C' distance/ultrasonic conditions depend on
    the physical track and must be reviewed by you. Everything is printed clearly.
"""
import argparse, os, sys, math, heapq
import numpy as np
import cv2

# ----------------------------------------------------------------------------
# CONFIG -- planning physics. Tune A_LAT/A_LONG to how aggressive you want it.
# ----------------------------------------------------------------------------
PLAN = dict(
    line_width_cm = 1.9,     # used to infer px/cm when --scale not given
    a_lat_cmps2   = 250.0,   # max lateral accel in a turn (~0.25 g). Lower = safer.
    a_long_cmps2  = 200.0,   # max accel/decel along the path
    v_at_pwm255   = 230.0,   # top linear speed at full PWM (cm/s) -- from robot_sim
    pwm_level1    = 140,      # firmware firstSpeed()
    pwm_level3    = 220,      # firmware thirdSpeed()
    turn_penalty_s= 0.15,     # time cost added per junction turn (decision overhead)
    resample_cm   = 0.5,      # centerline sampling for curvature/speed
    node_merge_px = 6,        # cluster junction pixels within this radius into 1 node
)

# ============================================================================
# 1-2. IMAGE -> BINARY MASK -> SKELETON
# ============================================================================
def load_mask(path, invert=None):
    img = cv2.imread(path, cv2.IMREAD_GRAYSCALE)
    if img is None:
        sys.exit(f"ERROR: cannot read image '{path}'")
    # Otsu split. The LINE is the minority class on a clean drawing.
    _, th = cv2.threshold(img, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    line_is_dark = (img[th == 0].size >= img[th == 255].size)  # fewer dark px = line dark
    if invert is None:
        # mask = 1 where the LINE is. Assume line is the minority colour.
        white_cnt = int((th == 255).sum()); dark_cnt = int((th == 0).sum())
        mask = (th == 0) if dark_cnt <= white_cnt else (th == 255)
    else:
        mask = (th == 0) if invert else (th == 255)
    m = mask.astype(np.uint8)
    # close tiny gaps from anti-aliasing so the skeleton is continuous
    m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))
    return m, img


def zhang_suen(mask):
    """Vectorized Zhang-Suen thinning -> 1px skeleton (bool array)."""
    img = (mask > 0).astype(np.uint8)
    def neighbours(z):
        P2 = np.roll(z, -1, 0); P6 = np.roll(z, 1, 0)
        P4 = np.roll(z, 1, 1);  P8 = np.roll(z, -1, 1)
        P3 = np.roll(P2, 1, 1); P5 = np.roll(P6, 1, 1)
        P7 = np.roll(P6, -1, 1); P9 = np.roll(P2, -1, 1)
        return P2, P3, P4, P5, P6, P7, P8, P9
    changed = True
    while changed:
        changed = False
        for step in (0, 1):
            P2, P3, P4, P5, P6, P7, P8, P9 = neighbours(img)
            seq = [P2, P3, P4, P5, P6, P7, P8, P9, P2]
            A = sum(((seq[i] == 0) & (seq[i + 1] == 1)).astype(np.uint8) for i in range(8))
            B = P2 + P3 + P4 + P5 + P6 + P7 + P8 + P9
            if step == 0:
                c1 = (P2 * P4 * P6 == 0); c2 = (P4 * P6 * P8 == 0)
            else:
                c1 = (P2 * P4 * P8 == 0); c2 = (P2 * P6 * P8 == 0)
            cond = (img == 1) & (B >= 2) & (B <= 6) & (A == 1) & c1 & c2
            if cond.any():
                img[cond] = 0
                changed = True
    return img.astype(bool)


def prune_spurs(skel, max_spur_px):
    """Remove short skeleton hairs (spurs) created by thinning a thick/AA line:
    a degree-1 leaf whose chain reaches a junction within max_spur_px is deleted.
    Real branches (longer than that) survive. Iterates until stable."""
    skel = skel.copy()
    while True:
        sset = set(map(tuple, np.argwhere(skel)))
        leaves = [p for p in sset if _crossing(p, sset) == 1]
        to_del = []
        for lf in leaves:
            chain = [lf]
            prev, cur = None, lf
            steps = 0
            while True:
                if _crossing(cur, sset) >= 3:     # reached a junction
                    break
                nxt = _step(cur, prev, _nbrs(cur, sset))
                if len(nxt) != 1:
                    break
                prev, cur = cur, nxt[0]
                chain.append(cur)
                steps += 1
                if steps > max_spur_px:
                    break
            if steps <= max_spur_px and _crossing(cur, sset) >= 3:
                to_del.extend(chain[:-1])         # keep the junction pixel itself
        if not to_del:
            break
        for p in set(to_del):
            skel[p[0], p[1]] = False
    return skel


def infer_px_per_cm(mask, skel, line_width_cm):
    """Half the line thickness = the distance-transform value along the centerline
    (skeleton ridge). Median over the ridge is robust to rounded caps/corners."""
    dt = cv2.distanceTransform(mask, cv2.DIST_L2, 5)
    vals = dt[skel]
    if vals.size == 0:
        return None
    half_w_px = float(np.median(vals))
    return float(2.0 * half_w_px / line_width_cm)


# ============================================================================
# 4. SKELETON -> GRAPH
# ============================================================================
N8 = [(-1, -1), (-1, 0), (-1, 1), (0, -1), (0, 1), (1, -1), (1, 0), (1, 1)]
# clockwise 8-ring starting North -- order matters for the crossing number
RING = [(-1, 0), (-1, 1), (0, 1), (1, 1), (1, 0), (1, -1), (0, -1), (-1, -1)]

def _nbrs(p, sset):
    r, c = p
    return [(r+dr, c+dc) for dr, dc in N8 if (r+dr, c+dc) in sset]

def _crossing(p, sset):
    """Number of 0->1 transitions around the 8-ring. 1=endpoint, 2=on a path,
    >=3=real junction. Robust to 8-connected staircase corners (unlike raw degree)."""
    r, c = p
    vals = [1 if (r+dr, c+dc) in sset else 0 for dr, dc in RING]
    vals.append(vals[0])
    return sum(1 for i in range(8) if vals[i] == 0 and vals[i+1] == 1)

def _step(cur, prev, nbrs):
    """Pick the next pixel when walking a path, skipping the redundant
    staircase neighbour that is 8-adjacent to where we just came from."""
    cands = [x for x in nbrs if x != prev]
    if prev is not None:
        nonadj = [x for x in cands if max(abs(x[0]-prev[0]), abs(x[1]-prev[1])) > 1]
        if nonadj:
            cands = nonadj
    return cands

class Graph:
    def __init__(self):
        self.nodes = {}     # id -> (r, c) centroid
        self.edges = []     # list of dict(a, b, path[list of (r,c)], length_px)

def build_graph(skel, merge_px):
    H, W = skel.shape
    sset = set(map(tuple, np.argwhere(skel)))
    # node pixels via crossing number: endpoints (1) or junctions (>=3)
    cross = {p: _crossing(p, sset) for p in sset}
    nodepix = {p for p, t in cross.items() if t == 1 or t >= 3}
    g = Graph()
    if not nodepix:
        # pure loop: order it by walking
        loop = _trace_loop(sset)
        g.edges.append(dict(a=None, b=None, path=loop, length_px=_plen(loop)))
        return g
    # cluster adjacent node pixels into single nodes
    nodemask = np.zeros((H, W), np.uint8)
    for (r, c) in nodepix:
        nodemask[r, c] = 1
    nodemask = cv2.dilate(nodemask, np.ones((merge_px, merge_px), np.uint8))
    n_cc, lbl = cv2.connectedComponents(nodemask, connectivity=8)
    pix2node = {}
    for nid in range(1, n_cc):
        ys, xs = np.where(lbl == nid)
        g.nodes[nid] = (float(ys.mean()), float(xs.mean()))
    for p in nodepix:
        pix2node[p] = int(lbl[p[0], p[1]])
    # trace edges out of node pixels through pass-through chains
    seen = set()
    for npx in nodepix:
        for nb in _nbrs(npx, sset):
            if nb in nodepix:        # internal cluster step
                continue
            if (npx, nb) in seen:
                continue
            path = [npx]
            prev, cur = npx, nb
            while cur not in nodepix:
                path.append(cur)
                nxt = _step(cur, prev, _nbrs(cur, sset))
                if not nxt:
                    break
                prev, cur = cur, nxt[0]
            path.append(cur)
            seen.add((npx, nb))
            if len(path) >= 2:
                seen.add((cur, path[-2]))
            a, b = pix2node.get(npx), pix2node.get(cur)
            if a is None or b is None:
                continue
            g.edges.append(dict(a=a, b=b, path=path, length_px=_plen(path)))
    return g

def _trace_loop(sset):
    start = min(sset)
    nb0 = _nbrs(start, sset)
    if not nb0:
        return [start]
    prev, cur = start, nb0[0]
    path = [start]
    guard = 0
    while cur != start and guard < len(sset) + 5:
        path.append(cur)
        nxt = _step(cur, prev, _nbrs(cur, sset))
        if not nxt:
            break
        prev, cur = cur, nxt[0]
        guard += 1
    path.append(start)
    return path

def _plen(path):
    a = np.array(path, float)
    return float(np.hypot(np.diff(a[:, 0]), np.diff(a[:, 1])).sum())


# ============================================================================
# 5. ROUTE CHOICE
# ============================================================================
def edge_polyline_cm(path, scale):
    """(r,c) px path -> (x,y) cm, x right, y up (image row down -> y up)."""
    a = np.array(path, float)
    x = a[:, 1] / scale
    y = -a[:, 0] / scale
    return np.column_stack([x, y])

def curvature_speed_limit(poly_cm):
    """Per-vertex curvature-limited speed (cm/s) for a cm polyline."""
    p = resample(poly_cm, PLAN["resample_cm"])
    if len(p) < 3:
        return p, np.full(len(p), _vmax())
    d = np.gradient(p, axis=0)
    dd = np.gradient(d, axis=0)
    num = np.abs(d[:, 0] * dd[:, 1] - d[:, 1] * dd[:, 0])
    den = (d[:, 0] ** 2 + d[:, 1] ** 2) ** 1.5 + 1e-9
    kappa = num / den
    v = np.sqrt(PLAN["a_lat_cmps2"] / np.maximum(kappa, 1e-6))
    return p, np.minimum(v, _vmax())

def edge_time(poly_cm):
    """Approx traversal time (s) of an edge under the curvature speed limit."""
    p, v = curvature_speed_limit(poly_cm)
    if len(p) < 2:
        return 0.0
    ds = np.hypot(np.diff(p[:, 0]), np.diff(p[:, 1]))
    vmid = np.maximum((v[:-1] + v[1:]) / 2.0, 1.0)
    return float((ds / vmid).sum())

def choose_route(g, scale, start_px, end_px):
    # pure loop
    if g.nodes == {}:
        loop = g.edges[0]
        return [(loop, False)], None, "loop"
    # build adjacency with per-edge time cost
    adj = {nid: [] for nid in g.nodes}
    for ei, e in enumerate(g.edges):
        poly = edge_polyline_cm(e["path"], scale)
        t = edge_time(poly) + 1e-6
        adj[e["a"]].append((e["b"], ei, t))
        adj[e["b"]].append((e["a"], ei, t))
    endpoints = [nid for nid in g.nodes if len(adj[nid]) == 1]
    centroids = {nid: edge_polyline_cm([g.nodes[nid]], scale)[0] for nid in g.nodes}
    def nearest_px(xy):                       # xy = (col, row) image pixels
        col, row = xy
        return min(g.nodes,
                   key=lambda nid: (g.nodes[nid][0]-row)**2 + (g.nodes[nid][1]-col)**2)
    if start_px is not None:
        s = nearest_px(start_px)
    elif endpoints:
        # farthest-apart endpoint pair -> start is the one nearer the top
        s = max(endpoints, key=lambda nid: centroids[nid][1])
    else:
        s = max(g.nodes, key=lambda nid: centroids[nid][1])
    if end_px is not None:
        t = nearest_px(end_px)
    elif endpoints:
        t = max(endpoints, key=lambda nid: np.hypot(*(centroids[nid] - centroids[s])))
    else:
        t = max(g.nodes, key=lambda nid: np.hypot(*(centroids[nid] - centroids[s])))
    # Dijkstra over time, with a turn penalty per hop
    dist = {nid: math.inf for nid in g.nodes}; dist[s] = 0.0
    prev = {}
    pq = [(0.0, s)]
    while pq:
        dcur, u = heapq.heappop(pq)
        if dcur > dist[u]:
            continue
        if u == t:
            break
        for v, ei, w in adj[u]:
            nd = dcur + w + PLAN["turn_penalty_s"]
            if nd < dist[v]:
                dist[v] = nd; prev[v] = (u, ei); heapq.heappush(pq, (nd, v))
    if t not in prev and s != t:
        # disconnected; fall back to longest single edge
        ei = max(range(len(g.edges)), key=lambda i: g.edges[i]["length_px"])
        return [(g.edges[ei], False)], None, "single-edge"
    # reconstruct, orient each edge a->b along travel direction
    seq, node_seq = [], [t]
    cur = t
    while cur != s:
        u, ei = prev[cur]
        e = g.edges[ei]
        flip = (e["a"] != u)          # path stored a->b; travelling u->cur
        seq.append((e, flip))
        node_seq.append(u)
        cur = u
    seq.reverse(); node_seq.reverse()
    return seq, node_seq, "routed"


def assemble_route(seq, scale):
    chunks = []
    for e, flip in seq:
        poly = edge_polyline_cm(e["path"], scale)
        if flip:
            poly = poly[::-1]
        chunks.append(poly)
    route = chunks[0]
    for c in chunks[1:]:
        route = np.vstack([route, c[1:]])
    return route

def label_decisions(seq, g, scale):
    """At each interior junction, classify the turn as F / L / R by heading change."""
    decisions = []
    for k in range(len(seq) - 1):
        e0, f0 = seq[k]; e1, f1 = seq[k + 1]
        p0 = edge_polyline_cm(e0["path"], scale); p0 = p0[::-1] if f0 else p0
        p1 = edge_polyline_cm(e1["path"], scale); p1 = p1[::-1] if f1 else p1
        h_in = p0[-1] - p0[-2]
        h_out = p1[1] - p1[0]
        ang = math.degrees(math.atan2(h_in[0]*h_out[1]-h_in[1]*h_out[0],
                                      h_in[0]*h_out[0]+h_in[1]*h_out[1]))
        d = 'F' if abs(ang) < 30 else ('L' if ang > 0 else 'R')
        decisions.append((d, round(ang, 1)))
    return decisions


# ============================================================================
# 6. SPEED PROFILE + 7. EMIT
# ============================================================================
def _vmax():
    return PLAN["v_at_pwm255"] * PLAN["pwm_level3"] / 255.0

def resample(pts, spacing):
    if len(pts) < 2:
        return pts
    seg = np.hypot(np.diff(pts[:, 0]), np.diff(pts[:, 1]))
    s = np.concatenate([[0], np.cumsum(seg)])
    if s[-1] < 1e-6:
        return pts
    n = max(2, int(s[-1] / spacing))
    su = np.linspace(0, s[-1], n)
    return np.column_stack([np.interp(su, s, pts[:, 0]), np.interp(su, s, pts[:, 1])])

def speed_profile(route_cm):
    p, vlim = curvature_speed_limit(route_cm)
    ds = np.hypot(np.diff(p[:, 0]), np.diff(p[:, 1]))
    a = PLAN["a_long_cmps2"]
    v = vlim.copy()
    for i in range(1, len(v)):                       # forward: accel limit
        v[i] = min(v[i], math.sqrt(v[i-1]**2 + 2*a*ds[i-1]))
    for i in range(len(v)-2, -1, -1):                # backward: brake limit
        v[i] = min(v[i], math.sqrt(v[i+1]**2 + 2*a*ds[i]))
    return p, v, ds

def quantize_segments(p, v, ds):
    """Map continuous v to discrete robot levels (1/3), merge into segments."""
    v1 = PLAN["v_at_pwm255"] * PLAN["pwm_level1"] / 255.0
    v3 = PLAN["v_at_pwm255"] * PLAN["pwm_level3"] / 255.0
    thr = (v1 + v3) / 2.0
    lvl = np.where(v[:-1] >= thr, 3, 1)              # level per segment-ds
    segs = []
    i = 0
    while i < len(lvl):
        j = i
        while j < len(lvl) and lvl[j] == lvl[i]:
            j += 1
        seg_len = float(ds[i:j].sum())
        segs.append(dict(level=int(lvl[i]), len_cm=round(seg_len, 1),
                         vmax=float(v[i:j+1].max())))
        i = j
    return segs, v1, v3

def _polylen_cm(poly):
    return float(np.hypot(np.diff(poly[:, 0]), np.diff(poly[:, 1])).sum())

def emit_path(segs, decisions, dec_dist):
    """Build a DRAFT path[] + pathDistances[] by merging two distance-keyed
    streams along the route: speed-changes (at segment boundaries) and junction
    decisions R/L/F (at junction arc-lengths). pathDistances[i] = cm the robot
    PID-follows BEFORE firing token i. Colour mode defaults to 'B' (review!)."""
    seg_start, acc = [], 0.0
    for s in segs:
        seg_start.append(acc); acc += s["len_cm"]
    total = acc
    init_lvl = segs[0]["level"] if segs else 1
    events = []                                  # (distance_cm, token)
    last = init_lvl
    for s, st in list(zip(segs, seg_start))[1:]:
        if s["level"] != last:
            events.append((st, '3' if s["level"] == 3 else '1'))
            last = s["level"]
    for (d, ang), dd in zip(decisions, dec_dist):
        events.append((dd, d))                   # R / L / F at the junction
    events.sort(key=lambda e: e[0])
    tokens = ['B', '3' if init_lvl == 3 else '1']   # path[0]=colour, path[1]=speed
    dists = [0, 0]
    prev = 0.0
    for d, tok in events:
        tokens.append(tok); dists.append(int(round(d - prev))); prev = d
    tokens.append('s'); dists.append(int(round(total - prev)))   # stop after final span
    return ''.join(tokens), dists


# ============================================================================
# 8. SVG
# ============================================================================
def _spd_color(t):           # t in [0,1] slow->fast : red->green
    r = int(255 * (1 - t)); g = int(200 * t)
    return f"#{r:02x}{g:02x}40"

def write_svg(out, img_gray, route_px, v, g, start_end, decisions, node_seq):
    H, W = img_gray.shape
    vmax = max(v.max(), 1e-6)
    parts = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{W}" height="{H}" '
             f'viewBox="0 0 {W} {H}">',
             f'<rect width="{W}" height="{H}" fill="#111"/>']
    # faint full skeleton/graph in grey
    for e in g.edges:
        pts = " ".join(f"{c},{r}" for r, c in e["path"])
        parts.append(f'<polyline points="{pts}" fill="none" stroke="#333" stroke-width="2"/>')
    # the chosen route, coloured by speed
    for i in range(len(route_px) - 1):
        (r0, c0), (r1, c1) = route_px[i], route_px[i + 1]
        t = v[min(i, len(v)-1)] / vmax
        parts.append(f'<line x1="{c0:.1f}" y1="{r0:.1f}" x2="{c1:.1f}" y2="{r1:.1f}" '
                     f'stroke="{_spd_color(t)}" stroke-width="5"/>')
    # nodes
    for nid, (r, c) in g.nodes.items():
        parts.append(f'<circle cx="{c:.1f}" cy="{r:.1f}" r="5" fill="#08f"/>')
    # decisions
    if node_seq:
        for k, (d, ang) in enumerate(decisions):
            nid = node_seq[k + 1]
            r, c = g.nodes[nid]
            parts.append(f'<text x="{c+7:.1f}" y="{r-7:.1f}" fill="#ff0" '
                         f'font-size="20" font-family="monospace">{d}</text>')
    # start/end
    if start_end:
        (sr, sc), (er, ec) = start_end
        parts.append(f'<circle cx="{sc:.1f}" cy="{sr:.1f}" r="8" fill="none" stroke="#0f0" stroke-width="3"/>')
        parts.append(f'<circle cx="{ec:.1f}" cy="{er:.1f}" r="8" fill="none" stroke="#f00" stroke-width="3"/>')
    parts.append(f'<text x="10" y="{H-12}" fill="#aaa" font-size="16" '
                 f'font-family="monospace">green=fast  red=slow  blue=node  '
                 f'O=start(green)/end(red)</text>')
    parts.append('</svg>')
    with open(out, "w") as f:
        f.write("\n".join(parts))


# ============================================================================
# DRIVER
# ============================================================================
def cm_to_px(poly_cm, scale):
    return np.column_stack([-poly_cm[:, 1] * scale, poly_cm[:, 0] * scale])

def run_plan(args):
    mask, gray = load_mask(args.image, args.invert)
    skel = zhang_suen(mask)
    scale = args.scale or infer_px_per_cm(mask, skel, PLAN["line_width_cm"])
    if not scale or scale <= 0:
        sys.exit("ERROR: could not infer px/cm; pass --scale")
    # prune spurs shorter than ~1.2 line-widths (kills thinning hairs, keeps real branches)
    skel = prune_spurs(skel, int(round(1.2 * PLAN["line_width_cm"] * scale)))
    g = build_graph(skel, PLAN["node_merge_px"])
    print(f"[vision] {int(mask.sum())} line px | scale {scale:.2f} px/cm | "
          f"{len(g.nodes)} nodes, {len(g.edges)} edges")

    start_px = _parse_xy(args.start); end_px = _parse_xy(args.end)
    seq, node_seq, kind = choose_route(g, scale, start_px, end_px)
    route_cm = assemble_route(seq, scale)
    decisions = label_decisions(seq, g, scale) if node_seq else []

    p_cm, v, ds = speed_profile(route_cm)
    segs, v1, v3 = quantize_segments(p_cm, v, ds)
    total_len = float(ds.sum())
    total_t = float((ds / np.maximum((v[:-1]+v[1:])/2, 1.0)).sum())
    # arc-length of each interior junction along the route (for decision placement)
    if node_seq:
        elens = [_polylen_cm(edge_polyline_cm(e["path"], scale)) for e, _ in seq]
        cum = np.cumsum(elens)
        dec_dist = [float(cum[k]) for k in range(len(seq) - 1)]
    else:
        dec_dist = []
    path, dists = emit_path(segs, decisions, dec_dist)

    # report
    print(f"[route]  kind={kind}  length={total_len:.1f} cm  "
          f"est. time={total_t:.2f} s  Vmax={v.max():.0f} cm/s")
    if decisions:
        print("[turns]  " + "  ".join(f"{d}({a:+.0f}deg)" for d, a in decisions))
    print(f"[speed]  level1={v1:.0f}cm/s(pwm{PLAN['pwm_level1']})  "
          f"level3={v3:.0f}cm/s(pwm{PLAN['pwm_level3']})  "
          f"-> {len(segs)} segments")
    print("\n=== DRAFT for a_pins_vars.ino  (REVIEW colour mode & C-conditions) ===")
    print(f'const char path[] = "{path}";')
    print(f'const int pathDistances[] = {{{", ".join(map(str, dists))}}};')
    print("=" * 70)

    os.makedirs(args.outdir, exist_ok=True)
    route_px = cm_to_px(p_cm, scale)
    start_end = None
    if node_seq:
        s_px = cm_to_px(p_cm[:1], scale)[0]; e_px = cm_to_px(p_cm[-1:], scale)[0]
        start_end = (s_px, e_px)
    svg = os.path.join(args.outdir, "plan.svg")
    write_svg(svg, gray, route_px, v, g, start_end, decisions, node_seq)
    cv2.imwrite(os.path.join(args.outdir, "skeleton.png"),
                (skel * 255).astype(np.uint8))
    print(f"[out]    {svg}  +  skeleton.png")
    plan_pwm = PLAN["pwm_level3"] if any(s["level"] == 3 for s in segs) else PLAN["pwm_level1"]
    return route_cm, v, plan_pwm

def run_validate(args):
    route_cm, v, pwm = run_plan(args)
    try:
        import robot_sim as rs
    except Exception as e:
        sys.exit(f"cannot import robot_sim: {e}")
    cfg = dict(rs.CFG)
    cfg["v_at_pwm255_cmps"] = PLAN["v_at_pwm255"]
    track = rs.resample(route_cm, 0.4)
    # gains matching the planned speed level (firmware firstSpeed/thirdSpeed)
    kp, kd = (0.016, 4.9) if pwm == PLAN["pwm_level3"] else (0.01, 2.89)
    res = rs.simulate(pwm, kp, kd, cfg, track, record=True)
    status = "FOLLOWS the line" if res["ok"] else "LOST the line"
    print("\n[validate] planned line driven through robot_sim "
          f"@ pwm{pwm} (kp{kp} kd{kd}):")
    print(f"   -> {status}  ({res['reason']})")
    print(f"   max tracking error = {res['max_err']:.2f} cm  |  sim time {res['time']:.2f} s")
    if not res["ok"]:
        print("   NOTE: too fast for these gains/curvature -> lower a_lat_cmps2 in PLAN "
              "or use level1 in the tight parts.")

def _parse_xy(s):
    if not s:
        return None
    x, y = s.split(",")
    return (float(x), float(y))


def main():
    ap = argparse.ArgumentParser(description="Track image -> fastest robot plan")
    sub = ap.add_subparsers(dest="cmd", required=True)
    for name in ("plan", "validate"):
        p = sub.add_parser(name)
        p.add_argument("image")
        p.add_argument("--scale", type=float, default=None, help="px per cm (else inferred)")
        p.add_argument("--invert", type=int, default=None, help="1=line is white, 0=line is dark")
        p.add_argument("--start", default=None, help="start col,row in PIXELS (junction tracks)")
        p.add_argument("--end", default=None, help="end col,row in PIXELS (junction tracks)")
        p.add_argument("--outdir", default="out")
    args = ap.parse_args()
    if args.cmd == "plan":
        run_plan(args)
    else:
        run_validate(args)


if __name__ == "__main__":
    main()
