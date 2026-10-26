#!/usr/bin/env python3
"""
compare_runs.py - compare two drive logs of the same route in ONE common GPS frame.

Typical use: the reference is a manual teaching run (log_009), the test is a later run on
the same physical path (log_010). The plot shows how far apart GPS places the two runs and
when the box drove on its own.

Usage:
    python3 compare_runs.py log_009.bin log_010.bin
    python3 compare_runs.py log_009.csv log_010.csv --annotate "92:stuck in AVOID" --annotate "176:GPS leg off route"
    python3 compare_runs.py log_009.bin log_010.bin --shift-from 250 --show-shift

Inputs: .bin (decoded with analyze_log.py, which must sit next to this file) or the .csv that
analyze_log.py writes. Needed CSV columns: t_s, lat, lon, gps_fix, cmd_l, cmd_r.

Output:
    <test>_vs_<ref>.png    next to the test log (or --out)
    printed table          distance of the test run to the reference track, per time window
    printed shift          best constant east/north shift that moves the test onto the reference

How it works:
  1. Real GPS fixes only: gps_fix true AND lat/lon changed (GPS updates at 1 Hz, the log at
     4 Hz, so each fix appears about 4 times).
  2. One common frame for BOTH runs, origin = first route waypoint of the reference. Metres by
     the equirectangular approximation (exact to cm over a few hundred metres):
         E = (lon - lon0) * 111194.93 * cos(lat0)      N = (lat - lat0) * 111194.93
     (111194.93 m = R_earth * pi / 180 with R = 6371 km.) A shared origin is essential: with
     each run centred on its own start, a constant GPS offset between days would be hidden.
  3. Route waypoints from the reference, the same way as make_route.py: 5-fix moving average,
     then one point every --spacing metres along the track, plus the end point.
  4. Distance: for every --step-th test fix, the distance to the NEAREST point of the
     reference track; median / max per --window seconds.
  5. Shift: a simple ICP (iterative closest point) for translation only: match each test fix
     to its nearest reference point, move by the mean difference, repeat. A small residual
     after the shift means: same path shape, whole track displaced = GPS frame offset.
  6. Autonomous records: the drive pad only sends multiples of 0.05 with both wheels equal
     (straight) or opposite (turn on the spot). Any other moving command came from the
     controller -> orange dot. Heuristic: a controller command that happens to look like a
     pad command counts as manual (rare; it can only make the orange part smaller).
"""
import argparse
import csv
import math
import subprocess
import sys
from pathlib import Path

import numpy as np

K = 111194.93  # metres per degree of latitude


# ---------------------------------------------------------------------------------------- input
def to_csv(path: Path) -> Path:
    """Return the CSV for a log. A .bin is decoded with analyze_log.py (writes <log>.csv)."""
    path = path.expanduser()
    if not path.exists():
        sys.exit(f"file not found: {path}\n(current folder: {Path.cwd()}) - check the path, "
                 f"or use the full path, e.g. ~/Documents/.../log_009.bin")
    if path.suffix.lower() == ".csv":
        return path
    if path.suffix.lower() != ".bin":
        sys.exit(f"{path}: expected a .bin or .csv file")
    out = path.with_suffix(".csv")
    if out.exists() and out.stat().st_mtime >= path.stat().st_mtime:
        return out  # already decoded and up to date
    analyzer = Path(__file__).resolve().parent / "analyze_log.py"
    if not analyzer.exists():
        sys.exit(f"analyze_log.py not found next to {Path(__file__).name}; pass the .csv instead")
    print(f"decoding {path.name} with analyze_log.py ...")
    subprocess.run([sys.executable, str(analyzer), str(path)], check=True)
    if not out.exists():
        sys.exit(f"analyze_log.py did not write {out.name}")
    return out


def load(path: Path) -> dict:
    """Columns as numpy arrays: t, lat, lon, fix, cmd_l, cmd_r."""
    need = ("t_s", "lat", "lon", "gps_fix", "cmd_l", "cmd_r")
    cols = {k: [] for k in need}
    with open(to_csv(path), newline="") as f:
        rd = csv.DictReader(f)
        missing = [k for k in need if k not in (rd.fieldnames or [])]
        if missing:
            sys.exit(f"{path.name}: missing columns {missing}")
        for row in rd:
            for k in need:
                v = row[k]
                if k == "gps_fix":
                    cols[k].append(v.strip().lower() in ("1", "true", "yes"))
                else:
                    try:
                        cols[k].append(float(v))
                    except ValueError:
                        cols[k].append(float("nan"))
    d = {k: np.array(v) for k, v in cols.items()}
    return {"t": d["t_s"], "lat": d["lat"], "lon": d["lon"], "fix": d["gps_fix"].astype(bool),
            "cmd_l": d["cmd_l"], "cmd_r": d["cmd_r"], "name": path.stem}


def real_fixes(d: dict):
    """Indices of real GPS updates: fix and position changed since the previous record."""
    lat, lon = d["lat"], d["lon"]
    changed = np.r_[True, (np.diff(lat) != 0) | (np.diff(lon) != 0)]
    ok = d["fix"] & changed & np.isfinite(lat) & np.isfinite(lon) & (lat != 0) & (lon != 0)
    return np.flatnonzero(ok)


# ---------------------------------------------------------------------------------------- maths
class Frame:
    """Local east/north metres around (lat0, lon0)."""

    def __init__(self, lat0, lon0):
        self.lat0, self.lon0 = lat0, lon0
        self.c = math.cos(math.radians(lat0))

    def en(self, lat, lon):
        return np.c_[(np.asarray(lon) - self.lon0) * K * self.c, (np.asarray(lat) - self.lat0) * K]


def smooth(x, n=5):
    """Centred moving average over n samples, edges padded with the first / last value."""
    if len(x) < n:
        return np.asarray(x, float)
    h = n // 2
    p = np.r_[np.full(h, x[0]), x, np.full(h, x[-1])]
    return np.convolve(p, np.ones(n) / n, "valid")


def route_waypoints(lat, lon, spacing):
    """Like make_route.py: smooth, then one point every `spacing` m along the track + end."""
    lat_s, lon_s = smooth(lat), smooth(lon)
    f = Frame(lat_s[0], lon_s[0])
    P = f.en(lat_s, lon_s)
    idx = [0]
    for i in range(1, len(P)):
        if np.hypot(*(P[i] - P[idx[-1]])) >= spacing:
            idx.append(i)
    if np.hypot(*(P[-1] - P[idx[-1]])) > spacing / 3:
        idx.append(len(P) - 1)
    return lat_s[idx], lon_s[idx]


def nearest_dist(X, Y, chunk=2000):
    """For each point of X (n,2): distance to the nearest point of Y (m,2), and its index."""
    dist = np.empty(len(X))
    idx = np.empty(len(X), dtype=int)
    for a in range(0, len(X), chunk):
        D = np.hypot(X[a:a + chunk, None, 0] - Y[None, :, 0], X[a:a + chunk, None, 1] - Y[None, :, 1])
        idx[a:a + chunk] = D.argmin(axis=1)
        dist[a:a + chunk] = D[np.arange(len(D)), idx[a:a + chunk]]
    return dist, idx


def best_shift(X, Y, iters=30):
    """Translation-only ICP: shift s so that X + s lies on Y. Returns s, residual distances."""
    s = np.zeros(2)
    for _ in range(iters):
        _, j = nearest_dist(X + s, Y)
        step = np.mean(Y[j] - (X + s), axis=0)
        s += step
        if np.hypot(*step) < 0.01:
            break
    res, _ = nearest_dist(X + s, Y)
    return s, res


def autonomous_mask(cmd_l, cmd_r):
    """True where the wheel command did not come from the drive pad (see module docstring)."""
    L, R = np.round(cmd_l, 3), np.round(cmd_r, 3)
    on_grid = np.abs(L * 20 - np.round(L * 20)) < 1e-3
    pad = ((L == R) | (L == -R)) & on_grid & (L != 0)
    moving = (L != 0) | (R != 0)
    return moving & ~pad & np.isfinite(L) & np.isfinite(R)


# ---------------------------------------------------------------------------------------- main
def main():
    ap = argparse.ArgumentParser(description="Compare two drive logs in one common GPS frame.")
    ap.add_argument("ref", type=Path, help="reference log (e.g. the teaching run), .bin or .csv")
    ap.add_argument("test", type=Path, help="log to compare against it, .bin or .csv")
    ap.add_argument("--spacing", type=float, default=4.0, help="waypoint spacing in m (default 4)")
    ap.add_argument("--step", type=int, default=4, help="use every n-th test fix for distances (default 4)")
    ap.add_argument("--window", type=float, default=50.0, help="time window for the table in s (default 50)")
    ap.add_argument("--shift-from", type=float, default=0.0,
                    help="estimate the GPS shift only from test fixes after this time in s (default 0)")
    ap.add_argument("--show-shift", action="store_true", help="also draw the test track moved by the shift")
    ap.add_argument("--annotate", action="append", default=[], metavar="T:TEXT",
                    help='label the test track at time T seconds, e.g. "92:stuck in AVOID" (repeatable)')
    ap.add_argument("--out", type=Path, help="output PNG (default: <test>_vs_<ref>.png next to the test log)")
    a = ap.parse_args()

    ref, test = load(a.ref), load(a.test)
    ir, it = real_fixes(ref), real_fixes(test)
    if len(ir) < 5 or len(it) < 5:
        sys.exit("not enough GPS fixes in one of the logs")

    # route waypoints of the reference; their first point is the common origin
    wlat, wlon = route_waypoints(ref["lat"][ir], ref["lon"][ir], a.spacing)
    fr = Frame(wlat[0], wlon[0])
    W = fr.en(wlat, wlon)
    A = fr.en(ref["lat"][ir], ref["lon"][ir])      # reference track (real fixes)
    B = fr.en(test["lat"][it], test["lon"][it])    # test track (real fixes)
    tb = test["t"][it]

    # ---- distance of the test run to the reference track
    sel = np.arange(0, len(B), max(1, a.step))
    dist, _ = nearest_dist(B[sel], A)
    print(f"\n{test['name']} vs {ref['name']}: distance to the reference track "
          f"({len(sel)} fixes, every {a.step}th)")
    print(f"  {'window':>13}  {'fixes':>5}  {'median m':>8}  {'max m':>6}")
    for t0 in np.arange(0, tb[-1] + a.window, a.window):
        m = (tb[sel] >= t0) & (tb[sel] < t0 + a.window)
        if m.any():
            print(f"  {t0:5.0f}-{t0 + a.window:5.0f} s  {m.sum():5d}  {np.median(dist[m]):8.1f}  {dist[m].max():6.1f}")
    print(f"  {'whole run':>13}  {len(sel):5d}  {np.median(dist):8.1f}  {dist.max():6.1f}")
    print(f"  end points: {test['name']} E{B[-1, 0]:.1f} N{B[-1, 1]:.1f} m, "
          f"{ref['name']} E{A[-1, 0]:.1f} N{A[-1, 1]:.1f} m, difference {np.hypot(*(B[-1] - A[-1])):.1f} m")

    # ---- best constant shift (GPS frame offset)
    late = sel[tb[sel] >= a.shift_from]
    shift = None
    if len(late) >= 5:
        shift, res = best_shift(B[late], A)
        before, _ = nearest_dist(B[late], A)
        print(f"\nbest shift for t >= {a.shift_from:.0f} s: E{shift[0]:+.1f} m, N{shift[1]:+.1f} m "
              f"(|{np.hypot(*shift):.1f}| m)")
        print(f"  distance before shift: median {np.median(before):.1f} m, max {before.max():.1f} m")
        print(f"  distance after shift:  median {np.median(res):.2f} m, max {res.max():.1f} m")
        print("  small residual after the shift = same path shape, displaced as a whole (GPS offset)")
    else:
        print(f"\nshift: fewer than 5 fixes after {a.shift_from:.0f} s, skipped")

    # ---- autonomous records of the test run
    auto = autonomous_mask(test["cmd_l"], test["cmd_r"])
    pos_ok = test["fix"] & np.isfinite(test["lat"]) & (test["lat"] != 0)
    P_all = fr.en(test["lat"], test["lon"])
    print(f"\nautonomous records in {test['name']}: {int((auto & pos_ok).sum())} of {len(auto)} "
          f"({100 * (auto & pos_ok).mean():.0f} %)")

    # ---- plot
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, ax = plt.subplots(figsize=(8, 10))
    ax.plot(A[:, 0], A[:, 1], color="0.7", lw=3, label=f"{ref['name']} (reference)")
    ax.plot(W[:, 0], W[:, 1], "o", color="0.4", ms=5, label=f"route waypoints (from {ref['name']})")
    for i, (x, y) in enumerate(W):
        if i % 3 == 0 or i == len(W) - 1:
            ax.annotate(f"WP{i}", (x, y), xytext=(-28, 0), textcoords="offset points", fontsize=8, color="0.3")
    ax.plot(B[:, 0], B[:, 1], color="tab:blue", lw=1.2, label=f"{test['name']}")
    m = auto & pos_ok
    if m.any():
        ax.scatter(P_all[m, 0], P_all[m, 1], s=10, color="tab:orange", zorder=5,
                   label=f"{test['name']} autonomous")
    if a.show_shift and shift is not None:
        Bs = B[tb >= a.shift_from] + shift   # only the part the shift was estimated from
        ax.plot(Bs[:, 0], Bs[:, 1], "--", color="tab:blue", lw=0.8, alpha=0.6,
                label=f"{test['name']} after {a.shift_from:.0f} s, moved by "
                      f"E{shift[0]:+.1f} N{shift[1]:+.1f} m")
    for spec in a.annotate:
        try:
            t_s, txt = spec.split(":", 1)
            t_s = float(t_s)
        except ValueError:
            print(f"ignoring --annotate '{spec}' (format T:TEXT)")
            continue
        i = int(np.argmin(np.abs(tb - t_s)))
        ax.annotate(txt.replace("\\n", "\n"), tuple(B[i]), xytext=(25, 10), textcoords="offset points",
                    fontsize=9, color="crimson", arrowprops=dict(arrowstyle="->", color="crimson"))

    lo = np.percentile(dist, 10) if len(dist) else 0
    hi = np.percentile(dist, 90) if len(dist) else 0
    ax.set_title(f"{test['name']} vs {ref['name']}: distance to reference "
                 f"{lo:.0f}-{hi:.0f} m (10th-90th percentile)")
    ax.set_aspect("equal", adjustable="datalim")
    ax.grid(alpha=0.3)
    ax.legend(fontsize=8, loc="lower left")
    ax.set_xlabel(f"east (m)  - origin WP0 of {ref['name']}")
    ax.set_ylabel("north (m)")
    fig.tight_layout()
    out = a.out or a.test.with_name(f"{test['name']}_vs_{ref['name']}.png")
    fig.savefig(out, dpi=120)
    print(f"\nwrote {out}")


if __name__ == "__main__":
    main()