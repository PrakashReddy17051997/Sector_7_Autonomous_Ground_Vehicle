#!/usr/bin/env python3
"""dr_check.py - where did the rover REALLY go? Dead reckoning (IMU yaw + wheel command, 1.03 m/s per unit,
crab -9 deg) started at a locked localizer pose, against where the localizer THOUGHT it was (lp_x/lp_y).
   python3 dr_check.py LocalPath.h log.csv t_start t_end out.png"""
import csv, math, re, sys
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
lp_h, log, t0, t1, out = sys.argv[1], sys.argv[2], float(sys.argv[3]), float(sys.argv[4]), sys.argv[5]
h = open(lp_h).read()
legs = re.findall(r"\{(\d+), (kPath\d+), [\d.]+f, (-?\d+), (-?\d+), \d+, \d+, kGrid\d+, [-\d.]+f, [-\d.]+f, [-\d.]+f, ([-\d.]+)f, ([-\d.]+)f, ([-\d.]+)f\}", h)
paths = {m[0]: [int(v) for v in m[1].split(",")] for m in re.findall(r"static const int16_t (kPath\d+)\[\] = \{([^}]*)\}", h)}
O = [(float(m[4]), float(m[5]), float(m[6])) for m in legs]; P = [paths[m[1]] for m in legs]
def g(o, x, y): c, s = math.cos(o[2]), math.sin(o[2]); return o[0] + c * x - s * y, o[1] + s * x + c * y
rows = [r for r in csv.DictReader(open(log))]; f = lambda r, k: float(r[k] or 0)
sel = [r for r in rows if t0 <= f(r, "t_s") <= t1]
st = next(r for r in sel if f(r, "lp_score") >= 0.6 and r["route_leg"] not in ("", "0"))   # first locked pose
k = int(st["route_leg"]) - 1
x, y = g(O[k], f(st, "lp_x"), f(st, "lp_y")); th = O[k][2] + math.radians(f(st, "lp_th")); yaw0 = f(st, "yaw_iso_deg")
dr = [(x, y)]; prev = st
for r in sel[sel.index(st) + 1:]:
    dt = f(r, "t_s") - f(prev, "t_s")
    v = 0.5 * (f(prev, "out_l") + f(prev, "out_r")) * 1.03
    hd = th + math.radians(f(r, "yaw_iso_deg") - yaw0) + (math.radians(-9) if v > 0 else 0)
    x += v * dt * math.cos(hd); y += v * dt * math.sin(hd); dr.append((x, y)); prev = r
fig, ax = plt.subplots(figsize=(10, 9))
for i in range(len(P)):
    pts = [g(O[i], P[i][2 * j] / 100, P[i][2 * j + 1] / 100) for j in range(len(P[i]) // 2)]
    ax.plot([p[0] for p in pts], [p[1] for p in pts], color="#2f6fde", lw=2); ax.annotate("L%d" % (i + 1), pts[0], color="#2f6fde")
ax.plot([p[0] for p in dr], [p[1] for p in dr], color="#e08a1e", lw=2.5, label="dead reckoning (IMU + wheels) = roughly where it went")
for r in sel:
    if f(r, "lp_score") >= 0.5 and r["route_leg"] not in ("", "0"):
        kk = int(r["route_leg"]) - 1; q = g(O[kk], f(r, "lp_x"), f(r, "lp_y"))
        ax.plot(q[0], q[1], ".", ms=3, color="green")
ax.plot([], [], ".", color="green", label="localizer belief (score >= 0.5)")
ax.plot(*dr[0], "ko"); ax.annotate("start %.0f s (leg %s)" % (f(st, "t_s"), st["route_leg"]), dr[0])
ax.set_aspect("equal"); ax.grid(alpha=.3); ax.legend(); ax.set_title("%s  %.0f-%.0f s" % (log.split("/")[-1], t0, t1))
fig.tight_layout(); fig.savefig(out, dpi=120); print("written", out)