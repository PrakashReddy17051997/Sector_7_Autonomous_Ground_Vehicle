#!/usr/bin/env python3
"""Turn a manual teaching run into the waypoint route for the box (GpsNav kDefaultRoute).

Usage:
    python3 make_route.py log_009.bin            # or the CSV written by analyze_log.py
    python3 make_route.py log_009.csv --spacing 4

Prints the C++ array to paste into GpsNav.cpp (replace kDefaultRoute), and writes
<log>_route.csv and <log>_route.png next to the log. Steps:
  1. take only real GPS updates (1 Hz), smooth over 5 fixes (~5 s) against GPS jitter
  2. keep one point every --spacing metres along the track (default 4 m), plus the end point
Check the PNG: points where the rover wandered (e.g. manoeuvring at a corner) can be deleted.
"""
import argparse
import math
import sys
from pathlib import Path

import numpy as np

K = 111194.93  # metres per degree


def load(path):
    p = Path(path)
    if p.suffix == ".bin":
        sys.path.insert(0, str(Path(__file__).parent))
        from analyze_log import load as load_bin, derive
        rec, _ = load_bin(p)
        d = derive(rec, 4.0)
        return d["t"], d["lat"], d["lon"], d["fix"]
    import csv
    t, la, lo, fx = [], [], [], []
    with open(p) as f:
        for row in csv.DictReader(f):
            t.append(float(row["t_s"])); la.append(float(row["lat"])); lo.append(float(row["lon"]))
            fx.append(row["gps_fix"] in ("1", "True", "true"))
    return np.array(t), np.array(la), np.array(lo), np.array(fx)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("log")
    ap.add_argument("--spacing", type=float, default=4.0, help="metres between waypoints")
    a = ap.parse_args()
    t, lat, lon, fix = load(a.log)
    keep = fix & np.r_[True, (np.diff(lat) != 0) | (np.diff(lon) != 0)]
    t, lat, lon = t[keep], lat[keep], lon[keep]
    if len(t) < 5:
        sys.exit("not enough GPS fixes")
    kern = np.ones(5) / 5
    pad = lambda x: np.r_[np.full(2, x[0]), x, np.full(2, x[-1])]
    lat_s = np.convolve(pad(lat), kern, "valid")
    lon_s = np.convolve(pad(lon), kern, "valid")
    c = math.cos(math.radians(lat_s[0]))
    E = (lon_s - lon_s[0]) * K * c
    N = (lat_s - lat_s[0]) * K
    idx = [0]
    for i in range(1, len(E)):
        if math.hypot(E[i] - E[idx[-1]], N[i] - N[idx[-1]]) >= a.spacing:
            idx.append(i)
    if math.hypot(E[-1] - E[idx[-1]], N[-1] - N[idx[-1]]) > a.spacing / 3:
        idx.append(len(E) - 1)

    base = Path(a.log).with_suffix("")
    print(f"// {len(idx)} waypoints from {Path(a.log).name}, every ~{a.spacing:g} m")
    print("static const GpsNav::Wp kDefaultRoute[] = {")
    with open(f"{base}_route.csv", "w") as f:
        f.write("wp,t_s,lat,lon,east_m,north_m\n")
        for j, i in enumerate(idx):
            print(f"  {{{lat_s[i]:.7f}, {lon_s[i]:.7f}}},  // WP{j:02d} t={t[i] - t[0]:.0f}s E{E[i]:+.1f} N{N[i]:+.1f}")
            f.write(f"{j},{t[i] - t[0]:.1f},{lat_s[i]:.7f},{lon_s[i]:.7f},{E[i]:.2f},{N[i]:.2f}\n")
    print("};")
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(figsize=(8, 9))
        ax.plot(E, N, color="0.6", lw=1, label="GPS track (smoothed)")
        ax.plot(E[idx], N[idx], "o-", color="tab:blue", label="waypoints")
        for j, i in enumerate(idx):
            ax.annotate(f"{j}", (E[i], N[i]), xytext=(4, 4), textcoords="offset points", fontsize=8)
        ax.set_aspect("equal", adjustable="datalim"); ax.grid(alpha=.3); ax.legend()
        ax.set_xlabel("east (m)"); ax.set_ylabel("north (m)"); ax.set_title(f"Route from {Path(a.log).name}")
        fig.tight_layout(); fig.savefig(f"{base}_route.png", dpi=120)
        print(f"// written {base}_route.csv and {base}_route.png", file=sys.stderr)
    except ImportError:
        pass


if __name__ == "__main__":
    main()
