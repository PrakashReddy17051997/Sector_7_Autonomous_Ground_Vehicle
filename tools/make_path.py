#!/usr/bin/env python3
"""Smooth continuous reference path from a teaching log (LiDAR + IMU frame, no GPS on the box).

    python3 make_path.py log_012.csv                 # -> log_012_path.csv + log_012_path.png

WHAT THE PATH IS
  A line through the taught drive, every 0.5 m:  s (m along the path), x, y (m), heading (deg, route
  yaw = gyro frame of the teaching run, + = left), curvature (1/m), and what the LiDAR saw there:
  wall distance left / right (m, from the 90-point scan, nan = no wall) and the free clearance.
  Frame: x along the rover at the start, y to the left (the same frame RouteFollow steers in).

HOW IT IS MADE
  1. shape    = IMU heading integrated over the distance driven. The distance comes from the GPS
                track length of the TEACHING run only (wheel effort varies 0.5-2.8 m per cmd*s on
                this rover, and the 90-point LiDAR cannot see progress along a straight corridor).
                log_012: this shape matches the GPS track to 2.7 m median over 141 m.
  2. smoothing = Gaussian along s (sigma 1.0 m), corners stay corners, zig-zags of the manual drive
                and GPS jitter go. Resampled every 0.5 m.
  3. landmarks = every 4th LiDAR scan placed with the pose on the path -> walls, railings, ramp
                edges in the path frame; clearance = closest point beside the rover in its own scan. The rear
                sector (130-230 deg) is left out: in a manual teaching drive that is the operator.
  Records where the rover stood still or was tilted > 5 deg (lifted / stuck on the ramp) add no
  distance and no landmarks.

The PNG shows the path and the landmarks in the path frame, turned so that north is about up (the
turn is fitted to the GPS track - drawing only). --gps also draws the GPS track: on log_012 it differs
by several metres / tens of degrees in places (building shadow at the start: 5 satellites, HDOP 2.7),
while the LiDAR walls stay sharp and parallel in the path frame - the path frame is the consistent one.
"""
import argparse
import sys
from pathlib import Path

import numpy as np
import pandas as pd

sys.path.insert(0, str(Path(__file__).parent))
import make_segments as MS  # noqa: E402
import marktags as mt  # noqa: E402
import navlib as nl  # noqa: E402

STEP = 0.5        # m between path points
SIGMA = 1.0       # m, smoothing
ANG = -np.radians(np.arange(90) * 4 + 2.0)
_deg = np.arange(90) * 4 + 2.0
REAR_OK = ~((_deg > 130) & (_deg < 230))   # log_012: an object at ~1.3 m behind the whole drive = the
                                           # operator walking behind; it would sit right on the path


def gauss_smooth(v, sigma_pts):
    k = np.arange(-int(3 * sigma_pts), int(3 * sigma_pts) + 1)
    w = np.exp(-0.5 * (k / sigma_pts) ** 2)
    w /= w.sum()
    pad = np.r_[[v[0]] * len(k), v, [v[-1]] * len(k)]
    return np.convolve(pad, w, "same")[len(k):-len(k)]


def build(B, S):
    s_gps = MS.path_length(B)
    yaw = np.unwrap(np.radians(B.yaw_iso_deg.values))
    yaw -= yaw[0]
    tilt = np.abs(B.pitch_deg.values - np.median(B.pitch_deg.values)) > 5
    ds = np.diff(s_gps, prepend=s_gps[0])
    ds[tilt] = 0.0
    x = np.cumsum(ds * np.cos(yaw))
    y = np.cumsum(ds * np.sin(yaw))
    s = np.cumsum(ds)
    # uniform in s, then smooth
    su = np.arange(0, s[-1], STEP / 2)
    xu, yu = np.interp(su, s, x), np.interp(su, s, y)
    sp = SIGMA / (STEP / 2)
    xs, ys = gauss_smooth(xu, sp), gauss_smooth(yu, sp)
    # re-parametrise by the smoothed length, every STEP
    L = np.r_[0, np.cumsum(np.hypot(np.diff(xs), np.diff(ys)))]
    sp_ = np.arange(0, L[-1], STEP)
    px, py = np.interp(sp_, L, xs), np.interp(sp_, L, ys)
    hd = np.unwrap(np.arctan2(np.gradient(py), np.gradient(px)))
    kap = np.gradient(hd) / STEP
    # record -> path index (by distance driven)
    ri = np.clip(np.searchsorted(sp_, s * (sp_[-1] / max(s[-1], 1e-6))), 0, len(sp_) - 1)
    # landmarks: LiDAR points placed with the raw pose (x, y, yaw) of the record
    lm, lm_i = [], []
    for i in range(0, len(B), 4):
        if tilt[i]:
            continue
        r = S[i] / 100.0
        ok = (r > 0.25) & (r < 6.0) & REAR_OK     # rear sector = the person walking behind the rover
        a = yaw[i] + ANG[ok]
        lm.append(np.c_[x[i] + r[ok] * np.cos(a), y[i] + r[ok] * np.sin(a)])
        lm_i.append(np.full(ok.sum(), ri[i]))
    LM, LMI = np.vstack(lm), np.concatenate(lm_i)
    # per path point: clearance = closest point BESIDE the rover in its own scan (|along| < 0.3 m,
    # rear sector excluded), min over the records at that point - no map smear
    clear = np.full(len(sp_), np.nan)
    per = [[] for _ in sp_]
    for i in range(len(B)):
        if tilt[i] or ds[i] <= 0:
            continue
        r = S[i] / 100.0
        ok = (r > 0.25) & REAR_OK
        al, la = r[ok] * np.cos(ANG[ok]), r[ok] * np.sin(ANG[ok])
        side = np.sort(np.abs(la[np.abs(al) < 0.3]))
        if len(side) >= 2:
            per[ri[i]].append(side[1])            # 2nd closest: one stray point (a foot) does not count
    for k in range(len(sp_)):
        if per[k]:
            clear[k] = float(np.median(per[k]))  # median over the scans taken at this path point
    from scipy.spatial import cKDTree  # noqa: F401
    dl = np.full(len(sp_), np.nan); dr = np.full(len(sp_), np.nan)
    for i in range(len(B)):
        if tilt[i]:
            continue
        for side, arr in ((-1, dl), (1, dr)):
            w = nl.measure(S[i], side)
            if w["ok"] and w["pts"] >= 6:
                arr[ri[i]] = w["d"] if np.isnan(arr[ri[i]]) else 0.5 * (arr[ri[i]] + w["d"])
    P = pd.DataFrame(dict(s_m=sp_, x_m=px, y_m=py, heading_deg=np.degrees(hd), curvature_1pm=kap,
                          wall_left_m=dl, wall_right_m=dr, clearance_m=clear))
    return P, dict(x=x, y=y, s=s, ri=ri, LM=LM, yaw=yaw)


def plot(path_png, B, P, raw, title, show_gps=False):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    E, N = B.east_m.values, B.north_m.values
    good = (B.gps_fix == 1) & (B.hdop < 1.6) & (B.t_s > 120)
    X = np.c_[raw["x"], raw["y"]]
    a, b = X[good], np.c_[E, N][good]
    ca, cb = a.mean(0), b.mean(0)
    U, _, Vt = np.linalg.svd((a - ca).T @ (b - cb))
    R = (U @ Vt).T
    f = lambda Q: (Q - ca) @ R.T + cb
    LM, Pp = f(raw["LM"]), f(P[["x_m", "y_m"]].values)
    fig, ax = plt.subplots(figsize=(11, 11))
    ax.scatter(LM[:, 0], LM[:, 1], s=1, color="#7a9cc6", alpha=0.25, label="landmarks: LiDAR points (walls, railings, ramp edges)")
    if show_gps:
        ax.plot(E, N, color="#c8c8c8", lw=6, alpha=0.7, label="GPS track (not used; buildings shift it by metres)")
    ax.plot(Pp[:, 0], Pp[:, 1], "--", color="#e0301e", lw=2.6, label="reference path: IMU heading x distance, smoothed (0.5 m points)")
    low = P.clearance_m.values < 0.35
    if low.any():
        ax.scatter(Pp[low, 0], Pp[low, 1], s=30, facecolors="none", edgecolors="k", lw=1.2, zorder=5,
                   label="clearance < 0.35 m")
    for k in range(0, len(P), 20):
        ax.text(Pp[k, 0], Pp[k, 1], f" {P.s_m.iloc[k]:.0f} m", fontsize=8, color="#e0301e")
    m = mt.presses(B.mark.values, B.mark_tag.values) if "mark_tag" in B.columns else []
    for i, mm, t in m:
        k = raw["ri"][i]
        ax.plot(*Pp[k], "o", color="k", ms=4, zorder=6)
        ax.text(Pp[k, 0], Pp[k, 1], f"  M{mm}", fontsize=7, color="k")
    ax.set_aspect("equal"); ax.grid(alpha=0.3)
    ax.set_xlabel("about East (m)"); ax.set_ylabel("about North (m)"); ax.set_title(title)
    ax.legend(loc="lower right", fontsize=9)
    plt.tight_layout(); plt.savefig(path_png, dpi=100)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("log", help="teaching log CSV (analyze_log.py)")
    ap.add_argument("--gps", action="store_true", help="also draw the GPS track in the PNG")
    a = ap.parse_args()
    B, S = MS.load(a.log)
    P, raw = build(B, S)
    base = Path(a.log).with_suffix("")
    P.to_csv(f"{base}_path.csv", index=False, float_format="%.3f")
    plot(f"{base}_path.png", B, P, raw, f"{Path(a.log).stem}: smooth reference path through the landmarks\n"
         "(LiDAR + IMU frame; rotated so north is about up)", a.gps)
    c = P.clearance_m
    print(f"path {P.s_m.iloc[-1]:.1f} m, {len(P)} points every {STEP} m, max |curvature| "
          f"{np.nanmax(np.abs(P.curvature_1pm)):.2f} 1/m, clearance min {c.min():.2f} m, "
          f"< 0.35 m at {(c < 0.35).sum()} points -> {base}_path.csv / .png")


if __name__ == "__main__":
    main()
