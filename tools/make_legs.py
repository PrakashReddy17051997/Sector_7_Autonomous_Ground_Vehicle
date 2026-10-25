#!/usr/bin/env python3
"""Teach & repeat: turn a teaching log (with Mark presses) into the leg table for RouteFollow.

Usage:
    python3 make_legs.py log_009.csv            # CSV written by analyze_log.py
    python3 make_legs.py log_009.csv --check    # also replays the end-of-leg detectors on the log

The PLAN below says which Mark segments form one leg and how the leg is tracked / ended. For each
leg the script measures from the log (gyro frame of the teaching run = "route yaw"):
    ref     wall direction (yaw + wall angle) for WALL legs, median yaw for HOLD legs
    dSet    median wall distance (WALL legs)
    effort  sum of the forward wheel command * dt  (what it took the rover to drive this leg)
    end     GPS position at the end of the leg (zone check only), and whether GPS moved at all
Prints the C++ table to paste into RouteFollow.cpp (kRoute).
"""
import argparse
import math
import sys
from pathlib import Path

import numpy as np
import pandas as pd

sys.path.insert(0, str(Path(__file__).parent))
import navlib as nl  # noqa: E402

# kind: WALL (follow wall on side), HOLD (keep heading). end: LOST (wall of the leg side ends),
# FOUND_R / FOUND_L (a wall appears on that side closer than endArg m), FRONT (something ahead
# closer than endArg m), EFFORT (taught effort reached). Turns between legs are added on the box.
# marks = (first, last) Mark segment numbers of the teaching log, inclusive.
PLAN = [
# gate = landmark accepted only after this fraction of the taught effort (an obstacle by the wall,
# the ramp surface ahead etc. look like the landmark earlier on the leg).
    # name                     marks     kind   side   end        endArg  noRight gpsAssist gate
    ("B facade",               (1, 1),   "WALL", "L", "LOST",     0,      1, 0, 0.85),
    ("after facade",           (2, 2),   "HOLD", "L", "EFFORT",   0,      1, 0, 0.75),
    ("to ramp D",              (3, 8),   "HOLD", "R", "FRONT",    2.5,    0, 0, 0.90),
    ("wall at ramp D",         (9, 10),  "WALL", "L", "LOST",     0,      0, 0, 0.75),
    ("around corner D",        (11, 13), "WALL", "L", "LOST",     0,      0, 0, 0.75),
    ("open stretch",           (14, 18), "HOLD", "R", "FOUND_R",  0.8,    0, 0, 0.75),
    ("narrow passage",         (19, 20), "WALL", "R", "LOST",     0,      0, 0, 0.80),
    ("diagonal",               (21, 24), "HOLD", "L", "EFFORT",   0,      0, 1, 0.75),
    ("wall to H",              (25, 29), "WALL", "L", "EFFORT",   0,      0, 0, 0.75),
    ("last stretch H",         (31, 32), "WALL", "L", "EFFORT",   0,      0, 0, 0.75),
]
EFFORT_END = 0.97  # EFFORT legs end at this fraction (the teaching run zig-zagged a little)
KIND = {"WALL": 0, "HOLD": 1}
END = {"LOST": 0, "FOUND_L": 1, "FOUND_R": 2, "FRONT": 3, "EFFORT": 4}


def load(path):
    A = pd.read_csv(path)
    lid = [c for c in A.columns if c.startswith("lidar")]
    S = A[lid].values.astype(float)
    A = A[[c for c in A.columns if not c.startswith("lidar")]].copy()
    A["yaw"] = np.degrees(np.unwrap(np.radians(A.yaw_iso_deg)))
    A["vf"] = ((A.out_l + A.out_r) / 2).clip(lower=0)
    return A, S


def wall_features(S, side):
    d = np.full(len(S), np.nan)
    a = np.full(len(S), np.nan)
    for i in range(len(S)):
        w = nl.measure(S[i], side)
        if w["ok"] and w["pts"] >= 6:
            d[i], a[i] = w["d"], w["ang"]
    return d, a


def wrap(x):
    return (x + 180) % 360 - 180


def build(A, S):
    Ld, La = wall_features(S, -1)
    Rd, Ra = wall_features(S, 1)
    legs = []
    for name, (m0, m1), kind, side, end, arg, noRight, gpsA, gate in PLAN:
        idx = np.where((A.mark >= m0) & (A.mark <= m1))[0]
        yaw = A.yaw.values[idx]
        if kind == "WALL":
            dd, aa = (Ld, La) if side == "L" else (Rd, Ra)
            wy = yaw + aa[idx] if side == "L" else yaw - aa[idx]
            ok = ~np.isnan(wy)
            ref = float(np.median(wy[ok]))
            dset = float(np.median(dd[idx][ok]))
        else:
            ref = float(np.median(yaw))
            dset = 0.0
        # the leg starts where the yaw first comes within 20 deg of ref (previous turn finished)
        # and ends where it last is within 20 deg (next turn starts)
        near = np.where(np.abs(wrap(yaw - ref)) < 20)[0]
        i0, i1 = idx[near[0]], idx[near[-1]]
        eff = float(A.vf.values[i0:i1 + 1].sum() * 0.25)
        g = A.iloc[i0:i1 + 1]
        gd = math.hypot(g.east_m.iloc[-1] - g.east_m.iloc[0], g.north_m.iloc[-1] - g.north_m.iloc[0])
        blind = gd < 0.25 * eff  # GPS hardly moved although the rover drove: GPS blind zone
        legs.append(dict(name=name, kind=kind, side=side, end=end, arg=arg, ref=ref, dset=dset,
                         eff=eff, i0=i0, i1=i1, t0=A.t_s.values[i0], t1=A.t_s.values[i1],
                         lat=A.lat.values[i1], lon=A.lon.values[i1], blind=blind, gpsd=gd,
                         noRight=noRight, gpsA=gpsA, gate=gate))
    return legs, (Ld, La, Rd, Ra)


def print_table(legs, src):
    print(f"// {len(legs)} legs from {src} (tools/make_legs.py). ref = route yaw (gyro frame of the")
    print("// teaching run, deg, + = left), dSet m, effort = sum(forward wheel cmd * s), gate, end GPS = zone only")
    print("static const RouteFollow::Leg kRoute[] = {")
    for k, L in enumerate(legs):
        print(f"  {{RouteFollow::{L['kind']}, {(-1 if L['side'] == 'L' else 1):2d}, RouteFollow::END_{L['end']}, "
              f"{L['arg']:.2f}f, {L['ref']:7.1f}f, {L['dset']:.2f}f, {L['eff']:5.1f}f, "
              f"{L['gate']:.2f}f, {L['lat']:.7f}, {L['lon']:.7f}, {int(L['blind'])}, {L['noRight']}, {L['gpsA']}, \"{L['name']}\"}},"
              f"  // {k + 1}: t {L['t0']:.0f}-{L['t1']:.0f} s, GPS {L['gpsd']:.1f} m")
    print("};")


def check(A, S, legs, feats):
    """Replay of the end detectors (same rules as RouteFollow.cpp) on the teaching log itself:
    each leg starts at its taught start; where does the detector fire, and at which effort?"""
    Ld, La, Rd, Ra = feats
    yaw = A.yaw.values
    vf = A.vf.values
    print("\nreplay of the end-of-leg detectors on the teaching log:")
    print("leg  end        taught end   fires at   effort/taught")
    for k, L in enumerate(legs):
        eff = 0.0
        seen = 0
        lostN = foundN = 0
        fronts = []
        fire = None
        for i in range(L["i0"], min(len(A), L["i1"] + 120)):
            eff += vf[i] * 0.25
            side = -1 if L["side"] == "L" else 1
            dd, aa = (Ld[i], La[i]) if side < 0 else (Rd[i], Ra[i])
            wy = yaw[i] + aa if side < 0 else yaw[i] - aa
            valid = not math.isnan(dd) and abs(wrap(wy - L["ref"])) < 20
            gate = eff >= L["gate"] * L["eff"]
            if L["end"] == "LOST":
                seen += valid
                lostN = 0 if valid else lostN + 1
                if gate and seen and lostN >= 6:
                    fire = i
            elif L["end"] in ("FOUND_R", "FOUND_L"):
                fd, fa = (Rd[i], Ra[i]) if L["end"] == "FOUND_R" else (Ld[i], La[i])
                ok = not math.isnan(fd) and fd < L["arg"]
                foundN = foundN + 1 if ok else 0
                if gate and foundN >= 4:
                    fire = i
            elif L["end"] == "FRONT":
                fronts.append(nl.corridor(S[i], 0, look=3.0, minr=0.25))
                if gate and len(fronts) >= 4 and np.median(fronts[-4:]) < L["arg"]:
                    fire = i
            else:
                if eff >= EFFORT_END * L["eff"]:
                    fire = i
            if fire is not None:
                break
        if fire is None:
            print(f"{k + 1:3d}  {L['end']:9s}  {L['t1']:6.0f} s   NOT FIRED  (max {eff / L['eff']:.2f})")
        else:
            print(f"{k + 1:3d}  {L['end']:9s}  {L['t1']:6.0f} s   {A.t_s.values[fire]:6.0f} s   {eff / L['eff']:.2f}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("log")
    ap.add_argument("--check", action="store_true")
    a = ap.parse_args()
    A, S = load(a.log)
    legs, feats = build(A, S)
    print_table(legs, Path(a.log).name)
    if a.check:
        check(A, S, legs, feats)


if __name__ == "__main__":
    main()
