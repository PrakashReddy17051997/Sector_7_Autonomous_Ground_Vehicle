#!/usr/bin/env python3
"""Route map for the box web page: the taught path, the walls, the Marks and the legs.

    python3 make_webmap.py log_013.csv          # -> ../cas_box/RouteWebMap.h
    python3 make_webmap.py log_013.csv --local log_013_local.npz   # path = the straightened legs the box follows

Made from the SAME teaching log as make_legs.py / make_segments.py (re-make all three together).
The map is the IMU + LiDAR (+ ultrasonic) map of lidar_map.py - no GPS:
  path    the teaching drive, smoothed, a point every 0.5 m (decimetres in the JSON)
  walls   LiDAR points of the matched scans, one per 40 cm cell, within 5 m of the path
  marks   where every Mark was pressed (path index)
  legs    for every leg of kRoute: first / last path index and the turn from route yaw (gyro frame
          of the teaching run, what RouteFollow steers in) to the map frame
On the page (Mission card "Route map") the box draws its BELIEVED position: leg + effort fraction
-> a point between the leg's first and last path index, heading = route yaw + turn, and the live
LiDAR scan around it. If the live walls lie on the map walls, the box is where it thinks it is.
"""
import argparse
import json
import sys
from pathlib import Path

import numpy as np
import pandas as pd

sys.path.insert(0, str(Path(__file__).parent))
import lidar_map as LMAP  # noqa: E402
import make_legs as ML  # noqa: E402
import marktags as mt  # noqa: E402

STEP = 0.5
CELL = 0.40      # one wall point per 40 cm cell ...
NEAR = 5.0       # ... and only within 5 m of the path (the page stays fast, the JSON ~15 kB)


def wrap(a):
    return (a + 180.0) % 360.0 - 180.0


def build(csv, local=None):
    A = pd.read_csv(csv)
    S = A[[c for c in A.columns if c.startswith("lidar_")]].values.astype(float)
    T = LMAP.run(A, S, verbose=False)
    x, y, th, ok = T[:, 0], T[:, 1], T[:, 2], T[:, 3].astype(bool)
    # path: smoothed, resampled by distance
    sx = pd.Series(x).rolling(9, center=True, min_periods=1).mean().values
    sy = pd.Series(y).rolling(9, center=True, min_periods=1).mean().values
    d = np.r_[0, np.cumsum(np.hypot(np.diff(sx), np.diff(sy)))]
    s = np.arange(0, d[-1], STEP)
    px, py = np.interp(s, d, sx), np.interp(s, d, sy)
    rec2p = lambda i: int(min(len(s) - 1, np.searchsorted(s, d[i])))
    # walls: matched scans, one point per cell
    pts = []
    for i in range(0, len(A), 2):
        if ok[i]:
            pts.append(LMAP.to_world(LMAP.body_points(S[i]), x[i], y[i], th[i]))
    W = np.vstack(pts)
    cells = np.unique(np.round(W / CELL).astype(int), axis=0) * CELL
    from scipy.spatial import cKDTree
    dpath, _ = cKDTree(np.c_[px, py]).query(cells)
    cells = cells[dpath < NEAR]
    # legs (same build as make_legs / make_segments)
    A2, S2 = ML.load(csv)
    legs, _ = ML.build(A2, S2)
    yaw_unw = A2.yaw.values                      # unwrapped gyro yaw = route yaw frame of kRoute
    out_legs, prev_end = [], rec2p(int(np.argmax(np.abs(A.out_l.values) + np.abs(A.out_r.values) > 0.05)))
    for L in legs:
        p0, p1 = prev_end, rec2p(L["i1"])
        seg = slice(L["i0"], L["i1"] + 1)
        rot = float(np.degrees(np.angle(np.mean(np.exp(1j * np.radians(np.degrees(th[seg]) - yaw_unw[seg]))))))
        out_legs.append([p0, max(p1, p0 + 1), round(rot, 1), L["name"]])
        prev_end = p1
    marks = [[m, rec2p(i)] for i, m, t in mt.presses(A.mark.values, A.mark_tag.values)] if "mark_tag" in A else []
    if local:   # the path the box FOLLOWS: the straightened legs of localpath.py, put into the map frame
        import localpath as LP
        model = LP.load(local)
        if len(model) != len(out_legs):
            sys.exit(f"{local}: {len(model)} legs, the route has {len(out_legs)} - re-make localpath.py build")
        G, starts = [], []
        for L in model:
            o = L["origin"]; c, sn = np.cos(o[2]), np.sin(o[2])
            Q = L["path"]
            G.append(np.c_[o[0] + c * Q[:, 0] - sn * Q[:, 1], o[1] + sn * Q[:, 0] + c * Q[:, 1]])
        newp, bounds = [], []
        for g in G:   # each leg resampled every STEP, then joined
            dd = np.r_[0, np.cumsum(np.hypot(*np.diff(g, axis=0).T))]
            ss = np.r_[np.arange(0, dd[-1], STEP), dd[-1]]
            q = np.c_[np.interp(ss, dd, g[:, 0]), np.interp(ss, dd, g[:, 1])]
            bounds.append((len(newp), len(newp) + len(q) - 1))
            newp += list(q)
        newp = np.array(newp)
        for k, (b0, b1) in enumerate(bounds):
            out_legs[k][0], out_legs[k][1] = b0, max(b1, b0 + 1)
        # marks: nearest point of the new path to where the teaching run was at the press
        mk = []
        for i, m, t in (mt.presses(A.mark.values, A.mark_tag.values) if "mark_tag" in A else []):
            k = next((j for j, L in enumerate(model) if L["r0"] <= i <= L["r1"]), len(model) - 1)
            b0, b1 = bounds[k]
            dq = np.hypot(newp[b0:b1 + 1, 0] - x[i], newp[b0:b1 + 1, 1] - y[i])
            mk.append([m, int(b0 + np.argmin(dq))])
        marks = mk
        px, py = newp[:, 0], newp[:, 1]
    dm = lambda v: [int(round(q * 10)) for q in v]
    J = dict(src=Path(csv).stem, step=STEP,
             p=[c for xy in zip(dm(px), dm(py)) for c in xy],
             w=[c for xy in zip(dm(cells[:, 0]), dm(cells[:, 1])) for c in xy],
             legs=out_legs, marks=marks)
    return J, len(px), len(cells)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("teach", help="teaching log CSV (analyze_log.py), the same as for make_legs / make_segments")
    ap.add_argument("--local", help="log_013_local.npz from localpath.py build: draw the STRAIGHTENED path the box follows")
    ap.add_argument("--out", default=str(Path(__file__).resolve().parent.parent / "cas_box" / "RouteWebMap.h"))
    a = ap.parse_args()
    J, npath, nwall = build(a.teach, a.local)
    txt = json.dumps(J, separators=(",", ":"))
    with open(a.out, "w") as f:
        f.write("#pragma once\n")
        f.write(f"// RouteWebMap - route map for the web page. GENERATED by tools/make_webmap.py from {Path(a.teach).name}.\n")
        f.write("// Do not edit: re-run the tool together with make_legs.py and make_segments.py (same log).\n")
        f.write("// JSON: p = path x,y (dm, every 0.5 m), w = wall points x,y (dm), legs = [first path index,\n")
        f.write("// last path index, turn route yaw -> map (deg), name], marks = [mark, path index]. No GPS.\n")
        f.write(f"static const uint8_t kWebMapLegs = {len(J['legs'])};\n")
        f.write(f'static const char kRouteWebMapJson[] PROGMEM = R"rwm({txt})rwm";\n')
    print(f"{len(J['legs'])} legs, path {npath} points, walls {nwall} points, {len(txt)} bytes -> {a.out}")


if __name__ == "__main__":
    main()
