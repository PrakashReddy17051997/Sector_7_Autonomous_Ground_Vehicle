"""Closed-loop simulation of WallFollow.cpp (same fit + controller) with a simulated LiDAR.
Scene: wall on the LEFT (x = -1.3 m) from y = 0 to 14 m, with a 1 m door gap at y = 5..6,
a box (0.5 x 0.5 m) on the path at y = 9, then the wall ends (facade end).
Rover model from log_003: 1.1 m/s and 62 deg/s per unit command. LiDAR: 90 rays of 4 deg,
1 cm noise, 35 % no-return (as outdoors), new scan 4x/s. Speed command 0.2 (slider)."""
import math, numpy as np, sys
rng = np.random.default_rng(int(sys.argv[1]) if len(sys.argv) > 1 else 0)
SEG = [((-1.3, 0), (-1.3, 5)), ((-1.3, 6), (-1.3, 14)),                 # wall with door gap
       ((-0.05, 9), (0.45, 9)), ((0.45, 9), (0.45, 9.5)), ((0.45, 9.5), (-0.05, 9.5)), ((-0.05, 9.5), (-0.05, 9))]  # box
def ray(px, py, hdg):  # hdg: clockwise from +y (north)
    dx, dy = math.sin(hdg), math.cos(hdg); best = 0
    for (ax, ay), (bx, by) in SEG:
        ex, ey = bx - ax, by - ay; den = dx * ey - dy * ex
        if abs(den) < 1e-9: continue
        t = ((ax - px) * ey - (ay - py) * ex) / den; u = ((ax - px) * dy - (ay - py) * dx) / den
        if t > 0 and 0 <= u <= 1 and (best == 0 or t < best): best = t
    return best
def scan(x, y, h):
    out = np.zeros(90)
    for i in range(90):
        r = ray(x, y, h + math.radians(i * 4 + 2))
        if 0.2 < r < 12 and rng.random() > 0.35: out[i] = round((r + rng.normal(0, 0.01)) * 100)
    return out
def fitline(x, y):
    mx, my = x.mean(), y.mean(); sxx = ((x - mx) ** 2).sum(); syy = ((y - my) ** 2).sum(); sxy = ((x - mx) * (y - my)).sum()
    phi = 0.5 * math.atan2(2 * sxy, sxx - syy); ux, uy = math.cos(phi), math.sin(phi)
    return (mx, my, ux, uy) if ux >= 0 else (mx, my, -ux, -uy)
def measure(sc, side, prior=None):  # side -1 left, +1 right ; returns (dist, angle) or None
    a = np.arange(90) * 4 + 2.0; c = 270 if side < 0 else 90
    off = np.abs((a - c + 540) % 360 - 180); ok = (sc >= 20) & (sc <= 300) & (off <= 60)
    r = sc / 100; px = r * np.cos(np.radians(a)); py = r * np.sin(np.radians(a)) * side
    if prior:   # tracking: points near the last wall line (bridges gaps in front of the rover)
        th = math.radians(prior[1]); m0 = ok & (np.abs(-math.sin(th) * px + math.cos(th) * py - prior[0]) < 0.15)
        if m0.sum() < 5: return None
        mx, my, ux, uy = fitline(px[m0], py[m0])
    else:
        core = ok & (off <= 20)
        if core.sum() < 4: return None
        mx, my, ux, uy = fitline(px[core], py[core])
    for _ in range(2):
        m = ok & (np.abs(-uy * (px - mx) + ux * (py - my)) < 0.10)
        if m.sum() < 5: return None
        mx, my, ux, uy = fitline(px[m], py[m])
    al = px[m] * ux + py[m] * uy
    if al.max() - al.min() < 0.6: return None
    if px[m].max() < -0.1: return None      # wall must reach at least the rover's side (not only behind)
    return abs(-uy * mx + ux * my), math.degrees(math.atan2(uy, ux))
clamp = lambda v, lo, hi: max(lo, min(hi, v))
HALF_W, LOOK = 0.35, 1.5          # rover half width + margin, look-ahead (m)
def corridor(sc, phi):            # free length (m) of a 2*HALF_W wide corridor in direction phi (deg, + = right)
    a = np.radians(np.arange(90) * 4 + 2.0) - math.radians(phi); r = sc / 100
    ok = sc > 0; lat = r * np.sin(a); al = r * np.cos(a)
    blk = ok & (np.abs(lat) < HALF_W) & (al > 0) & (al < LOOK)
    return al[blk].min() if blk.any() else LOOK
def best_dir(sc):                 # VFH-lite: freest direction, preferring straight ahead
    cands = [(corridor(sc, p), p) for p in range(-60, 61, 10) if p <= 0]   # left wall: never away from it (allowAwayFromWall = false)
    good = [(abs(p), p, c) for c, p in cands if c >= 1.2]
    if good: _, p, c = min(good); return p, c
    c, p = max(cands); return p, c
# WallFollow parameters (same defaults as WallFollow.h)
kDist, maxApp, kAng, turnFrac, fSlow, fMin, jump, bridge = 40, 20, 0.008, 0.6, 0.9, 0.35, 0.35, 2.0
side, spd = -1, 0.2
x, y, h = -0.2, 1.0, math.radians(-15)        # start 1.1 m from the wall, nose 15 deg towards it
sc = scan(x, y, h); w0 = measure(sc, side); wall = w0; dSet = clamp(w0[0], 0.35, 2.0); last = w0[0]; lastT = 0.0
state = "follow"; t = 0; dt = 0.02; nextScan = 0.25; L = R = 0.0; log = []
while t < 120 and state != "lost":
    if t >= nextScan: sc = scan(x, y, h); nextScan += 0.25
    w = measure(sc, side, wall if (t - lastT < 2.0 and last > 0) else None)
    if w and last > 0 and abs(w[0] - last) > jump: w = None
    if w: wall = w; last = w[0]; lastT = t
    if state != "avoid":
        if w: state = "follow"
        elif t - lastT < bridge: state = "bridge"
        else: state = "lost"; L = R = 0; break
    front = corridor(sc, 0)
    if state != "avoid" and front < 0.9: state = "avoid"
    if state == "avoid":
        phi, clr = best_dir(sc)
        if phi == 0 and front > 1.2: state = "follow"; last = -1   # passed: back to the wall (no jump gate)
        else:
            v = spd * clamp((clr - 0.35) / 0.55, 0, 1) * max(0.3, math.cos(math.radians(phi)))
            wc = clamp(0.01 * phi, -turnFrac * spd, turnFrac * spd)
    if state != "avoid":
        v = spd; wt = 0
        if state == "follow": wt = kAng * (wall[1] + clamp(kDist * (wall[0] - dSet), -maxApp, maxApp))
        else: v *= 0.7
        wt = clamp(wt, -turnFrac * spd, turnFrac * spd)
        wc = wt if side > 0 else -wt                                 # + = right (as in WallFollow.cpp)
    tl, tr = clamp(v + wc, -.5, .5), clamp(v - wc, -.5, .5)
    if front < 0.30 and tl + tr > 0: vv, ww = 0, (tl - tr) / 2; tl, tr = vv + ww, vv - ww   # SafetyGuard
    step = 1.0 * dt                                              # acceleration ramp (1/s), braking instant
    L = tl if abs(tl) <= abs(L) else L + clamp(tl - L, -step, step)
    R = tr if abs(tr) <= abs(R) else R + clamp(tr - R, -step, step)
    vt, wtrue = 1.1 * (L + R) / 2, math.radians(62 * (L - R) / 2)
    h += wtrue * dt; x += vt * dt * math.sin(h); y += vt * dt * math.cos(h); t += dt
    # true clearance to wall and box
    dwall = x + 1.3 if (y < 5 or y > 6) and y < 14 else np.nan
    dbox = math.hypot(max(-0.05 - x, 0, x - 0.45), max(9 - y, 0, y - 9.5))
    log.append((t, x, y, dwall, dbox, state, math.degrees(h)))
lg = np.array([(a, b, c, d, e) for a, b, c, d, e, *_ in log], dtype=float)
st = [s for *_, s, _ in log]
ok = ~np.isnan(lg[:, 3]) & (lg[:, 2] > 2) & (lg[:, 2] < 8.5)
print(f"dSet {dSet:.2f} m | final state {state} at y = {y:.1f} m, t = {t:.0f} s | "
      f"wall distance (y 2..8.5 m): mean {np.nanmean(lg[ok,3]):.2f}, min {np.nanmin(lg[ok,3]):.2f}, max {np.nanmax(lg[ok,3]):.2f} | "
      f"min clearance to box {lg[:,4].min():.2f} m | bridge passes {sum(1 for i in range(1,len(st)) if st[i]=='bridge' and st[i-1]!='bridge')}")
if len(sys.argv) > 2:
    import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
    fig, ax = plt.subplots(figsize=(6, 9))
    for (a, b) in SEG: ax.plot([a[0], b[0]], [a[1], b[1]], "k", lw=3)
    ax.plot(lg[:, 1], lg[:, 2], color="tab:blue", lw=2, label="simulated rover (center)")
    ax.axvline(-1.3 + dSet, color="g", ls="--", lw=1, label=f"set distance {dSet:.2f} m")
    ax.set_aspect("equal"); ax.set_xlim(-2, 2); ax.grid(alpha=.3); ax.legend(loc="lower right", fontsize=8)
    ax.set_title(f"WallFollow sim: door gap y=5-6, box y=9, wall ends y=14\nstop state: {state} at y={y:.1f} m")
    ax.set_xlabel("x (m)"); ax.set_ylabel("y (m), driving direction")
    fig.tight_layout(); fig.savefig(sys.argv[2], dpi=110)
