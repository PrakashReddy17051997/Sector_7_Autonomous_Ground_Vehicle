#!/usr/bin/env python3
"""Closed-loop simulation of RouteFollow.cpp (same state machine, numbers and LiDAR maths) on a
synthetic copy of legs 1-5 of log_009 (facade B, straight on, to ramp D, wall at D, around the
corner of D). Checks the signs (turn directions, wall side), the turns, the leg ends and
obstacle handling (a box on the facade path).

Rover model from the logs: forward 0.62 m/s per unit wheel command (log_009: effort 36.4 on the
22.7 m facade), on-the-spot turn 31 deg/s at +-0.5 (log_009), arc turn rate 40 deg/s per unit of
wheel difference; MotorShaper (dead band 0.18, pivot min 0.5) applied. LiDAR 90 rays, 1 cm noise,
35 % dropouts, 4 Hz. Gyro: 0.5 deg/min drift. Writes sim_route.png.

    python3 sim_route.py [seed]
"""
import math
import sys

import numpy as np

from pathlib import Path
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import navlib as nl  # noqa: E402

rng = np.random.default_rng(int(sys.argv[1]) if len(sys.argv) > 1 else 1)
# legs 1-5 of kRoute (RouteFollow.cpp): kind, side, end, endArg, ref, dSet, effort, gate, noRight
LEGS = [("WALL", -1, "LOST", 0, -37.8, 1.45, 36.4, 0.85, 1),
        ("HOLD", -1, "EFFORT", 0, -36.4, 0, 9.7, 0.75, 1),
        ("HOLD", 1, "FRONT", 2.5, -122.9, 0, 30.2, 0.90, 0),
        ("WALL", -1, "LOST", 0, -216.2, 1.33, 4.7, 0.75, 0),
        ("WALL", -1, "LOST", 0, -125.3, 1.39, 12.0, 0.75, 0)]
C = 90 - (-37.8)   # world angle (deg, CCW from +x) = routeYaw + C ; facade runs along +y
M_PER_EFF = 0.62

# ---- world (metres): facade on the left of +y; at its end open; then leg 3 runs east (+x) to
# a wall across (west face of D, x = XD); leg 4 south along it; corner; leg 5 east along the
# south face of D.
fac_len = 36.4 * M_PER_EFF                      # 22.6 m
y3 = fac_len + 9.7 * M_PER_EFF                   # where leg 3 starts (after the turn)
XD = 30.2 * M_PER_EFF + 2.3                      # west face of D (front landmark 2.5 m)
yS = y3 - 1.33 - 4.7 * M_PER_EFF                 # south face of D (y)
SEG = [((-1.45, -1), (-1.45, fac_len)),          # B facade
       ((XD, y3 + 6), (XD, yS)),                 # D west face
       ((XD, yS), (XD + 7.5, yS)),               # D south face
       ((0.0, 14.0), (0.4, 14.0)), ((0.4, 14.0), (0.4, 14.4)),   # box 0.4 m on the path
       ((0.4, 14.4), (0.0, 14.4)), ((0.0, 14.4), (0.0, 14.0))]   # (gap to the wall 1.45 m)


def ray(px, py, ang):
    dx, dy = math.cos(ang), math.sin(ang)
    best = 0
    for (ax, ay), (bx, by) in SEG:
        ex, ey = bx - ax, by - ay
        den = dx * ey - dy * ex
        if abs(den) < 1e-9:
            continue
        t = ((ax - px) * ey - (ay - py) * ex) / den
        u = ((ax - px) * dy - (ay - py) * dx) / den
        if t > 0 and 0 <= u <= 1 and (best == 0 or t < best):
            best = t
    return best


def scan(x, y, th):
    out = np.zeros(90)
    for i in range(90):  # LiDAR index i: 4i+2 deg CLOCKWISE from the nose
        r = ray(x, y, th - math.radians(i * 4 + 2))
        if 0.2 < r < 12 and rng.random() > 0.35:
            out[i] = round((r + rng.normal(0, 0.01)) * 100)
    return out


def shape(l, r):  # MotorShaper without the kick
    z = lambda v: 0 if abs(v) < 0.02 else v
    l, r = z(l), z(r)
    if l == 0 and r == 0:
        return 0, 0
    if l * r < 0:
        m = max(abs(l), abs(r))
        if m < 0.5:
            l, r = l * 0.5 / m, r * 0.5 / m
        return l, r
    f = lambda v: 0 if v == 0 else math.copysign(0.18 + 0.32 * min(abs(v), 0.5) / 0.5, v)
    return f(l), f(r)


def wrap(a):
    return (a + 180) % 360 - 180


class Route:  # port of RouteFollow::_step (DRIVE / TURN / AVOID / end detection)
    kHead, kI, iMax, pivotDeg, turnDone, kDist = 0.015, 0.0008, 0.06, 35, 8, 40

    def __init__(s, sc, yaw):
        L = LEGS[0]
        w = nl.measure(sc, L[1])
        s.off = wrap(L[4] - (yaw + w["ang"]))
        s.i, s.state, s.eff, s.I = 0, "DRIVE", 0.0, 0.0
        s.passE, s.tries = -99, 0
        s.seen = s.lostN = s.foundN = 0
        s.fr = []
        s.l = s.r = 0
        s.log = []

    def heading(s, yd, ry, v):
        e = wrap(yd - ry)
        if abs(e) > s.pivotDeg:
            s.l = -0.5 if e > 0 else 0.5
            s.r = -s.l
            s.I = 0
            return
        s.I = np.clip(s.I + s.kI * e, -s.iMax, s.iMax)
        w = np.clip(s.kHead * e + s.I, -v, v)
        s.l, s.r = v - w, v + w

    def wall(s, sc, side, ry):
        w = nl.measure(sc, side)
        if not w["ok"] or w["pts"] < 6:
            return None
        return w["d"], wrap(ry + w["ang"] if side < 0 else ry - w["ang"]), w["rms"], w["pts"]

    def step(s, sc, yaw, t):
        if s.state == "DONE":
            s.l = s.r = 0
            return
        kind, side, end, arg, ref, dset, eff, gate, noRight = LEGS[s.i]
        ry = wrap(yaw + s.off)
        if s.state == "TURN":
            e = wrap(ref - ry)
            if abs(e) < s.turnDone:
                s.state, s.l, s.r = "DRIVE", 0, 0
            else:
                s.l = -0.5 if e > 0 else 0.5
                s.r = -s.l
            return
        fr = nl.corridor(sc, 0, half=0.35, look=1.5, minr=0.25)
        if s.state == "AVOID":
            if s.ph == 0:                       # wait 3 s
                s.l = s.r = 0
                if fr > 1.0:
                    s.state = "DRIVE"
                    return
                if t - s.t0 < 3:
                    return
                best = None
                for a in range(20, 61, 10):
                    for sg in (-1, 1):
                        phi = sg * a
                        if phi > 0 and noRight:
                            continue
                        if nl.corridor(sc, phi, 0.35, 1.5, 0.25) >= 1.2:
                            best = phi
                            break
                    if best is not None:
                        break
                if best is None:
                    s.tries += 1
                    if s.tries >= 3:
                        s.log.append((t, "PAUSE blocked"))
                        s.state = "DONE"
                    s.t0 = t
                    return
                s.tgt, s.ph = wrap(ry - best), 1
                s.log.append((t, f"avoid {'left' if best < 0 else 'right'} {abs(best)}"))
                return
            if s.ph == 1:                       # turn out
                e = wrap(s.tgt - ry)
                if abs(e) < s.turnDone:
                    s.ph, s.e0, s.l, s.r = 2, s.eff, 0, 0
                    return
                s.l = -0.5 if e > 0 else 0.5
                s.r = -s.l
                return
            if s.ph == 2:                       # sidestep until the leg direction is free
                if fr < 0.7 or s.eff - s.e0 > 3.0:
                    s.ph, s.t0, s.l, s.r = 0, t, 0, 0
                    return
                back = nl.corridor(sc, -wrap(ref - ry), 0.35, 1.5, 0.25)
                if s.eff - s.e0 > 0.4 and back >= 1.45:
                    s.ph, s.l, s.r = 3, 0, 0
                    return
                s.heading(s.tgt, ry, SPEED)
                return
            e = wrap(ref - ry)                  # ph 3: turn back parallel to the leg
            if abs(e) < s.turnDone:
                s.state, s.passE, s.l, s.r = "DRIVE", s.eff, 0, 0
                return
            s.l = -0.5 if e > 0 else 0.5
            s.r = -s.l
            return
        # DRIVE
        if fr < 0.7:
            s.state, s.ph, s.t0, s.l, s.r = "AVOID", 0, t, 0, 0
            return
        v = SPEED * np.clip((fr - 0.4) / 0.6, 0.5, 1)
        w = s.wall(sc, side, ry)
        valid = w is not None and abs(wrap(w[1] - ref)) < 20
        yd = ref
        passing = s.eff - s.passE < 2.5
        if kind == "WALL" and valid and passing:
            yd = w[1]                           # just passed an obstacle: parallel, no approach
        elif kind == "WALL" and valid:
            d, wy, rms, pts = w
            app = np.clip(s.kDist * (d - dset), -20, 20)
            yd = wy + (app if side < 0 else -app)
            if rms < 0.04 and pts >= 8:
                s.off = wrap(s.off + np.clip(0.05 * wrap(ref - wy), -0.5, 0.5))
        s.heading(yd, ry, v)
        g = s.eff >= gate * eff
        fire = False
        if end == "LOST":
            if valid:
                s.seen, s.lostN = 1, 0
            else:
                s.lostN += 1
            fire = g and s.seen and s.lostN >= 6
        elif end == "FRONT":
            s.fr.append(nl.corridor(sc, 0, 0.35, 3.0, 0.25))
            fire = g and len(s.fr) >= 4 and np.median(s.fr[-4:]) < arg
        elif end == "EFFORT":
            fire = s.eff >= 0.97 * eff
        if fire:
            s.log.append((t, f"leg {s.i + 1} end, effort {s.eff / eff:.2f}"))
            s.i += 1
            if s.i >= len(LEGS):
                s.state = "DONE"
                return
            s.eff, s.I, s.seen, s.lostN, s.fr = 0, 0, 0, 0, []
            s.passE, s.tries = -99, 0
            s.state = "TURN"
        if s.eff > 1.6 * eff + 4:
            s.log.append((t, f"leg {s.i + 1}: PAUSE end not found"))
            s.state = "DONE"


SPEED = 0.20
x, y, th = 0.0, 0.0, math.radians(90 + 4)    # placed 4 deg towards the open side
yaw = math.degrees(th) - 37.0                 # today's gyro frame: arbitrary offset
R = Route(scan(x, y, th), yaw)
dt, t, k = 0.02, 0.0, 0
trace = []
while t < 900 and R.state != "DONE":
    k += 1
    if k % 12 == 0:   # 4 Hz step (every 12 x 20 ms = 240 ms)
        yaw_meas = math.degrees(th) - 37.0 + 0.5 * t / 60
        R.step(scan(x, y, th), yaw_meas, t)
    l, r = shape(R.l, R.r)
    v = 0.62 * (l + r) / 2
    if l * r < 0:
        om = math.copysign(31, r - l) if abs(r - l) >= 0.99 else 0
    else:
        om = 40 * (r - l)
    th += math.radians(om) * dt
    x += v * math.cos(th) * dt
    y += v * math.sin(th) * dt
    R.eff += max(0, (l + r) / 2) * dt
    trace.append((t, x, y, R.i, R.state))
    t += dt

import os
if os.environ.get("DEBUG"):
    a, b = map(float, os.environ["DEBUG"].split(","))
    for tt, xx, yy, ii, st in trace[::25]:
        if a <= tt <= b:
            print(f"{tt:6.1f} x{xx:5.2f} y{yy:6.2f} leg{ii + 1} {st}")
for e in R.log:
    print(f"{e[0]:6.1f} s  {e[1]}")
tr = np.array([(a, b, c) for a, b, c, _, _ in trace])
# wall distance along the facade and the south face of D
fac = tr[(tr[:, 2] > 2) & (tr[:, 2] < fac_len - 2) & (tr[:, 1] < 1)]
print(f"facade: distance to wall {np.min(fac[:, 1] + 1.45):.2f}..{np.max(fac[:, 1] + 1.45):.2f} m (set 1.45)")
print(f"min distance to box: {min(math.hypot(px - np.clip(px, 0.0, 0.4), py - np.clip(py, 14, 14.4)) for _, px, py in tr):.2f} m")
print(f"min x on the facade (open edge side is +x): max {fac[:, 1].max():.2f} m right of start line")
south = tr[tr[:, 1] > XD + 2]
if len(south):
    print(f"south face of D: distance {np.min(yS - south[:, 2]):.2f}..{np.max(yS - south[:, 2]):.2f} m (set 1.39)")
print(f"end: state {R.state}, leg {R.i + 1}, t {t:.0f} s")
try:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(figsize=(7, 9))
    for (ax0, ay0), (bx, by) in SEG:
        ax.plot([ax0, bx], [ay0, by], "k-", lw=2)
    ax.plot(tr[:, 1], tr[:, 2], "-", color="tab:blue", lw=1.5, label="simulated rover")
    ax.set_aspect("equal")
    ax.grid(alpha=.3)
    ax.legend()
    ax.set_title("RouteFollow simulation, legs 1-5 of log_009")
    fig.tight_layout()
    fig.savefig(HERE / "sim_route.png", dpi=110)
except ImportError:
    pass
