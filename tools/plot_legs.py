#!/usr/bin/env python3
"""plot_legs.py - a mission log (where the box's LiDAR localizer placed the rover) over the taught path of
log_027 (LocalPath.h), for chosen legs. No GPS needed.
   python3 plot_legs.py LocalPath.h log_037.csv 6 9 out.png
Left: taught path (blue) + rover positions from lp_x/lp_y (coloured by leg, only while the match score >= 0.5).
Right: per time - match score, steering error to the carrot (carrot_err) and heading error (ctrl_err)."""
import csv, math, re, sys
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
lp_h, log, a, b, out = sys.argv[1], sys.argv[2], int(sys.argv[3]), int(sys.argv[4]), sys.argv[5]
h = open(lp_h).read()
legs = re.findall(r"\{(\d+), (kPath\d+), [\d.]+f, (-?\d+), (-?\d+), \d+, \d+, kGrid\d+, [-\d.]+f, [-\d.]+f, [-\d.]+f, ([-\d.]+)f, ([-\d.]+)f, ([-\d.]+)f\}", h)
paths = {m[0]: [int(v) for v in m[1].split(",")] for m in re.findall(r"static const int16_t (kPath\d+)\[\] = \{([^}]*)\}", h)}
def g(o, x, y): c, s = math.cos(o[2]), math.sin(o[2]); return o[0] + c * x - s * y, o[1] + s * x + c * y
O = [(float(m[4]), float(m[5]), float(m[6])) for m in legs]
P = [paths[m[1]] for m in legs]
rows = list(csv.DictReader(open(log))); f = lambda r, k: float(r[k] or 0)
fig, (ax, ay) = plt.subplots(1, 2, figsize=(15, 7), gridspec_kw={"width_ratios": [1, 1.3]})
cols = plt.get_cmap("tab10")
for k in range(a - 1, b):
    pts = [g(O[k], P[k][2 * i] / 100, P[k][2 * i + 1] / 100) for i in range(len(P[k]) // 2)]
    ax.plot([p[0] for p in pts], [p[1] for p in pts], color="#2f6fde", lw=2)
    ax.annotate("L%d" % (k + 1), pts[0], color="#2f6fde")
    sel = [r for r in rows if r["route_leg"] == str(k + 1) and f(r, "lp_score") >= 0.5]
    q = [g(O[k], f(r, "lp_x"), f(r, "lp_y")) for r in sel]
    ax.scatter([p[0] for p in q], [p[1] for p in q], s=6, color=cols(k % 10), label="log leg %d" % (k + 1))
ax.set_aspect("equal"); ax.grid(alpha=.3); ax.legend(fontsize=8); ax.set_title("taught path (blue) vs localizer positions")
sel = [r for r in rows if a <= int(r["route_leg"] or 0) <= b]
t = [f(r, "t_s") for r in sel]
ay.plot(t, [f(r, "carrot_err") for r in sel], lw=.8, label="carrot_err (deg, path ahead)")
ay.plot(t, [f(r, "ctrl_err") for r in sel], lw=.8, label="ctrl_err (deg, heading controller)")
ay.plot(t, [100 * f(r, "lp_score") for r in sel], lw=.8, color="grey", label="match score x100")
for r0, r1 in zip(sel, sel[1:]):
    if r0["route_leg"] != r1["route_leg"]: ay.axvline(f(r1, "t_s"), color="k", lw=.5); ay.text(f(r1, "t_s"), 95, "L" + r1["route_leg"], fontsize=8)
ay.set_ylim(-90, 100); ay.grid(alpha=.3); ay.legend(fontsize=8); ay.set_xlabel("time (s)")
fig.tight_layout(); fig.savefig(out, dpi=110); print("written", out)