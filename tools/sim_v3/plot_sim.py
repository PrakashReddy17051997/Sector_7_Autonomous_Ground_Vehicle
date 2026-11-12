#!/usr/bin/env python3
"""plot_sim.py - one PNG from a rover_sim run (matplotlib, no interaction).

    ./rover_sim --model rover7 --start-off 180 --legs 1 --time 70 --csv out/s180
    python3 plot_sim.py out/s180            -> out/s180.png

Left:  map (walls of the teaching run, grey), taught path (blue), obstacles (red), the rover footprint every
       2 s coloured by time (light -> dark), its track coloured by state (TURN orange, DRIVE blue, AVOID red,
       BACKUP purple, PAUSED grey), nose = short line.
Right: heading error to the current target (deg); wheel commands asked by RouteFollow (dashed) and what the
       wheels got after the MotorShaper (solid); state band at the bottom.
"""
import csv, sys
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon, Circle
import math

pre = sys.argv[1]
F = list(csv.DictReader(open(pre + "_frames.csv")))
W = list(csv.DictReader(open(pre + "_walls.csv")))
P = list(csv.DictReader(open(pre + "_path.csv")))
O = list(csv.DictReader(open(pre + "_obst.csv")))
title = open(pre + "_events.txt").readline().lstrip("# ").strip()
f = lambda r, k: float(r[k])
ST = {1: ("TURN", "#f79009"), 2: ("DRIVE", "#2f6fde"), 3: ("AVOID", "#d92d20"), 4: ("BACKUP", "#7a5af8"),
      5: ("PAUSED", "#98a2b3"), 6: ("DONE", "#98a2b3"), 0: ("OFF", "#98a2b3")}

fig = plt.figure(figsize=(15, 7.5))
ax = fig.add_axes([0.04, 0.08, 0.46, 0.84])
xs = [f(r, "x_m") for r in F]; ys = [f(r, "y_m") for r in F]
for o in O: xs += [f(o, "x_m")]; ys += [f(o, "y_m")]
pad = 1.5
x0, x1, y0, y1 = min(xs) - pad, max(xs) + pad, min(ys) - pad, max(ys) + pad
wx = [f(r, "x_m") for r in W if x0 - 1 < f(r, "x_m") < x1 + 1 and y0 - 1 < f(r, "y_m") < y1 + 1]
wy = [f(r, "y_m") for r in W if x0 - 1 < f(r, "x_m") < x1 + 1 and y0 - 1 < f(r, "y_m") < y1 + 1]
ax.scatter(wx, wy, s=6, marker="s", color="#9aa4b2", linewidths=0, label="walls (teaching run map)")
legs = sorted(set(int(r["leg"]) for r in P))
for k in legs:
    px = [f(r, "x_m") for r in P if int(r["leg"]) == k]; py = [f(r, "y_m") for r in P if int(r["leg"]) == k]
    ax.plot(px, py, color="#2f6fde", lw=1.2, alpha=0.7, label="taught path" if k == legs[0] else None)
    if x0 < px[0] < x1 and y0 < py[0] < y1: ax.annotate("L%d" % k, (px[0], py[0]), color="#2f6fde", fontsize=8)
for o in O:
    ax.add_patch(Circle((f(o, "x_m"), f(o, "y_m")), f(o, "r_m"), color="#d92d20", alpha=0.5))
    t1 = f(o, "t_to_s")
    ax.annotate("obstacle" + ("" if t1 < 0 else " %.0f-%.0f s" % (f(o, "t_from_s"), t1)), (f(o, "x_m"), f(o, "y_m")), color="#d92d20", fontsize=8)
for i in range(1, len(F)):
    ax.plot([xs[i - 1], xs[i]], [ys[i - 1], ys[i]], color=ST[int(F[i]["state"])][1], lw=2)
n = len(F)
cmap = plt.get_cmap("Greys")
for i in range(0, n, 10):
    x, y, th = f(F[i], "x_m"), f(F[i], "y_m"), math.radians(f(F[i], "heading_deg"))
    c, s = math.cos(th), math.sin(th)
    pts = [(x + c * u - s * v, y + s * u + c * v) for u, v in ((0.17, 0.13), (0.17, -0.13), (-0.17, -0.13), (-0.17, 0.13))]
    col = cmap(0.25 + 0.7 * i / max(1, n - 1))
    ax.add_patch(Polygon(pts, closed=True, fill=False, ec=col, lw=0.8))
    ax.plot([x, x + 0.27 * c], [y, y + 0.27 * s], color=col, lw=0.8)
ax.plot(xs[0], ys[0], "o", color="green", ms=7, label="start")
ax.plot(f(F[-1], "x_m"), f(F[-1], "y_m"), "s", color="black", ms=7, label="end")
for k, (nm, col) in ST.items():
    if k in (1, 2, 3, 4): ax.plot([], [], color=col, lw=3, label=nm)
ax.set_xlim(x0, x1); ax.set_ylim(y0, y1); ax.set_aspect("equal")
ax.set_xlabel("x (m)"); ax.set_ylabel("y (m)"); ax.grid(alpha=0.3); ax.legend(loc="best", fontsize=8)
ax.set_title("rover footprint every 2 s (light = early, dark = late)", fontsize=10)

t = [f(r, "t_s") for r in F]
a1 = fig.add_axes([0.56, 0.56, 0.42, 0.36])
a1.plot(t, [f(r, "head_err_deg") for r in F], color="#e08a1e")
a1.axhline(0, color="k", lw=0.6); a1.set_ylabel("heading error (deg)"); a1.grid(alpha=0.3)
a1.set_title(title, fontsize=9)
a2 = fig.add_axes([0.56, 0.14, 0.42, 0.36], sharex=a1)
a2.plot(t, [f(r, "wheel_l") for r in F], color="#2f6fde", label="left wheel (to rover)")
a2.plot(t, [f(r, "wheel_r") for r in F], color="#d92d20", label="right wheel (to rover)")
a2.plot(t, [f(r, "ask_l") for r in F], color="#2f6fde", ls="--", lw=0.8, alpha=0.6, label="left asked")
a2.plot(t, [f(r, "ask_r") for r in F], color="#d92d20", ls="--", lw=0.8, alpha=0.6, label="right asked")
a2.set_ylim(-0.6, 0.65); a2.set_ylabel("wheel command"); a2.grid(alpha=0.3); a2.legend(fontsize=8, ncol=2, loc="upper right")
a3 = fig.add_axes([0.56, 0.08, 0.42, 0.03], sharex=a1)
for i in range(1, n):
    a3.axvspan(t[i - 1], t[i], color=ST[int(F[i]["state"])][1], lw=0)
a3.set_yticks([]); a3.set_xlabel("time (s)   - band: state")
plt.setp(a1.get_xticklabels(), visible=False); plt.setp(a2.get_xticklabels(), visible=False)
fig.savefig(pre + ".png", dpi=110)
print("written", pre + ".png")
