#!/usr/bin/env python3
"""Believed position vs the taught map, for a route log (what the Route map card shows live).

    python3 replay_route.py log_013.csv log_0XX.csv     # teaching log, test log
        -> log_0XX_replay.png + table per leg

For every record of the test log in which the route ran (route_leg > 0):
  believed position  = the box's own estimate: leg + effort fraction (effort as on the box:
                       forward command * battery V / 11.6 V, summed per leg) -> point on that leg's
                       part of the taught path; heading = test-log gyro yaw + the offset that makes
                       the leg start parallel to the taught leg direction
  agreement          = share of the live LiDAR points (0.3 - 6 m, rear sector left out) that lie
                       within 0.3 m of a wall of the taught map when the scan is put at the believed
                       position. ~70-90 % = the box is where it thinks; < 40 % = it is not.
No GPS anywhere. The map is the IMU + LiDAR map of the teaching log (lidar_map.py).
"""
import argparse
import sys
from pathlib import Path

import numpy as np
import pandas as pd
from scipy.spatial import cKDTree

sys.path.insert(0, str(Path(__file__).parent))
import lidar_map as LMAP  # noqa: E402
import make_legs as ML  # noqa: E402
import make_webmap as WM  # noqa: E402

TOL = 0.30


def teach_map(teach_csv):
    J, _, _ = WM.build(teach_csv)
    A = pd.read_csv(teach_csv)
    S = A[[c for c in A.columns if c.startswith("lidar_")]].values.astype(float)
    T = LMAP.run(A, S, verbose=False)
    pts = [LMAP.to_world(LMAP.body_points(S[i]), T[i, 0], T[i, 1], T[i, 2]) for i in range(0, len(A), 2) if T[i, 3]]
    W = np.vstack(pts)
    P = np.array(J["p"], float).reshape(-1, 2) / 10.0
    return J, P, W


def path_pt(P, f):
    f = np.clip(f, 0, len(P) - 1)
    i = int(np.floor(f)); j = min(len(P) - 1, i + 1); t = f - i
    return P[i] + (P[j] - P[i]) * t


def believed(J, P, B, legs):
    """per record: x, y, th (map frame) like the web card, from route_leg + effort"""
    leg = B.route_leg.values.astype(int) - 1
    v = B.battery_v.values
    eq = np.where((v > 9) & (v < 13.5), v / 11.6, 1.0)
    vf = np.clip((B.out_l.values + B.out_r.values) / 2, 0, None) * eq * 0.25
    yaw = np.degrees(np.unwrap(np.radians(B.yaw_iso_deg.values)))
    out = np.full((len(B), 3), np.nan)
    eff, cur, off = 0.0, -1, 0.0
    for i in range(len(B)):
        k = leg[i]
        if k < 0 or k >= len(J["legs"]):
            continue
        if k != cur:                       # leg start: rover parallel to the taught leg direction
            cur, eff = k, 0.0
            off = legs[k]["ref"] - yaw[i]
        eff += vf[i]
        p0, p1, rot, _ = J["legs"][k]
        frac = min(1.2, eff / max(legs[k]["eff"], 1e-3))
        x, y = path_pt(P, p0 + frac * (p1 - p0))
        out[i] = x, y, np.radians(yaw[i] + off + rot)
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("teach"); ap.add_argument("test")
    a = ap.parse_args()
    J, P, W = teach_map(a.teach)
    tree = cKDTree(W)
    A2, S2 = ML.load(a.teach)
    legs, _ = ML.build(A2, S2)
    B = pd.read_csv(a.test)
    SB = B[[c for c in B.columns if c.startswith("lidar_")]].values.astype(float)
    if "route_leg" not in B or not (B.route_leg > 0).any():
        sys.exit("the test log has no route run (route_leg is 0 everywhere)")
    X = believed(J, P, B, legs)
    agree = np.full(len(B), np.nan)
    for i in range(len(B)):
        if np.isnan(X[i, 0]):
            continue
        Q = LMAP.to_world(LMAP.body_points(SB[i]), X[i, 0], X[i, 1], X[i, 2])
        Q = Q[np.hypot(*(Q - X[i, :2]).T) < 6.0]
        if len(Q) >= 8:
            d, _ = tree.query(Q)
            agree[i] = np.mean(d < TOL)
    print(f"{'leg':<40} {'records':>7} {'agreement':>9}")
    for k in sorted(set(B.route_leg[B.route_leg > 0].astype(int))):
        m = (B.route_leg.values == k) & ~np.isnan(agree)
        name = J["legs"][k - 1][3] if k - 1 < len(J["legs"]) else "?"
        print(f"{k:>2} {name:<37} {m.sum():>7} {100 * np.nanmean(agree[m]) if m.any() else float('nan'):8.0f} %")
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(figsize=(11, 11))
    ax.scatter(W[::3, 0], W[::3, 1], s=1, color="#9aa4b2", alpha=0.3, label=f"walls of the teaching run ({Path(a.teach).stem})")
    ax.plot(P[:, 0], P[:, 1], "-", color="#2f81f7", lw=2, label="taught path")
    for i in range(0, len(B), 4):
        if np.isnan(X[i, 0]):
            continue
        Q = LMAP.to_world(LMAP.body_points(SB[i]), X[i, 0], X[i, 1], X[i, 2])
        ax.scatter(Q[:, 0], Q[:, 1], s=2, color=plt.cm.RdYlGn(np.nan_to_num(agree[i])), alpha=0.6)
    ok = ~np.isnan(X[:, 0])
    ax.plot(X[ok, 0], X[ok, 1], ".", color="#d62a1e", ms=3, label="believed position (leg + effort)")
    ax.set_aspect("equal"); ax.grid(alpha=0.3)
    ax.set_title(f"{Path(a.test).stem}: believed position vs the taught map (live scans coloured by agreement, green = fits)")
    ax.legend(loc="best", fontsize=9)
    out = Path(a.test).with_name(Path(a.test).stem + "_replay.png")
    plt.tight_layout(); plt.savefig(out, dpi=100)
    print(f"-> {out}")


if __name__ == "__main__":
    main()
