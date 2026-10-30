#!/usr/bin/env python3
"""Local-coordinate path tracking: every leg in its own frame (no GPS, no global map).

    python3 localpath.py build  log_013.csv                  # -> log_013_local.npz (+ PNG of all legs)
    python3 localpath.py replay log_013_local.npz log_012.csv # localise another drive leg by leg

LEG FRAME   origin = where the leg starts in the teaching run, x = the rover's heading there,
            y = to the left (metres). Inside one leg (2-15 m) the IMU + LiDAR trajectory of the
            teaching run is accurate to a few cm - the metre errors of a global map come from
            adding up 140 m and do not exist here.
PER LEG     path   the taught path in the leg frame, a point every 0.25 m, s_end = its length
            grid   likelihood field of the walls in the leg frame (10 cm cells, uint8): the scans of
                   this leg + 2 m before + 3 m of the next leg (so the end is covered)
            next   pose of the next leg's origin in this frame (hand-over at the leg end)
LOCALISER   (the same on the box later, see LocalPath.h)
  predict   heading: gyro change; distance: forward command * 1.0 m/s (battery-compensated);
            0 when standing, pivoting or when the scan has not changed for 1 s
  correct   the live scan (0.3-6 m, rear sector = operator left out) is matched against the leg
            grid near the prediction: +-0.4 m in 5 cm steps, -4..+4 deg, then 2 cm / 1 deg;
            a weak prior keeps the prediction where the walls say nothing (along a corridor)
  lost      8 unmatched scans in a row, or > 1 m off the path: search along the taught path (3 m
            back to 3 m ahead, every 0.5 m, path direction +-20 deg) - the distance along the leg is
            the uncertain direction (a straight corridor looks the same everywhere)
  leg end   when the position along the path passes s_end - 0.25 m -> next leg, pose handed over
"""
import argparse
import sys
from pathlib import Path

import numpy as np
import pandas as pd
from scipy.ndimage import distance_transform_edt

sys.path.insert(0, str(Path(__file__).parent))
import lidar_map as LMAP  # noqa: E402
import make_legs as ML  # noqa: E402

RES = 0.10          # grid cell (m)
SIG = 0.15          # likelihood blur (m): softer = a point on a cell border changes the score less
STEP = 0.25         # path point spacing (m)
K_SPEED = 1.0       # m/s per forward command (measured with the LiDAR, log_012)
BEFORE_M, AFTER_M = 2.0, 3.0
ANG = -np.radians(np.arange(90) * 4 + 2.0)
DEG = np.arange(90) * 4 + 2.0
REAR = (DEG > 130) & (DEG < 230)


MAP_MARGIN = 7.0    # m around the path box of a leg that is mapped (log_013: 290 kB for 17 legs)
RMAX = 10.0         # LiDAR points used (m); log_017: in the open area half the points are 6-11 m away


def scan_pts(cm, rmax=RMAX):
    r = np.asarray(cm, float) / 100.0
    ok = (r > 0.3) & (r < rmax) & ~REAR
    return np.c_[r[ok] * np.cos(ANG[ok]), r[ok] * np.sin(ANG[ok])]


def to_frame(P, o):
    """world points P -> frame with origin pose o = (x, y, th)"""
    c, s = np.cos(-o[2]), np.sin(-o[2])
    d = P - o[:2]
    return np.c_[c * d[:, 0] - s * d[:, 1], s * d[:, 0] + c * d[:, 1]]


def pose_to_frame(p, o):
    q = to_frame(np.array([p[:2]]), o)[0]
    return np.array([q[0], q[1], wrap(p[2] - o[2])])


def wrap(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


def T(P, x, y, th):
    c, s = np.cos(th), np.sin(th)
    return np.c_[x + c * P[:, 0] - s * P[:, 1], y + s * P[:, 0] + c * P[:, 1]]


# ------------------------------------------------------------------------------------------ build
def clean_path(path, near=0.2, win=10):
    """taught path without the back-and-forth (you backed up / shuffled on the spot, SLAM jitter on the
    ramp): from each kept point skip to the LAST point within the next win that is still within near
    m of it (a loop or a reversal comes back there), then smooth and resample every STEP m"""
    keep, i = [0], 0
    while i < len(path) - 1:
        j = i + 1
        for q in range(i + 2, min(len(path), i + win + 1)):
            if np.hypot(*(path[q] - path[i])) < near:
                j = q
        keep.append(j)
        i = j
    P = path[keep]
    if len(P) >= 3:
        P = np.r_[P[:1], (P[:-2] + P[1:-1] + P[2:]) / 3, P[-1:]]
    d = np.r_[0, np.cumsum(np.hypot(*np.diff(P, axis=0).T))]
    sp = np.arange(0, max(d[-1], STEP) + 1e-9, STEP)
    return np.round(np.c_[np.interp(sp, d, P[:, 0]), np.interp(sp, d, P[:, 1])] * 100) / 100, sp


STRAIGHTEN = True     # build: straight pieces instead of the driven wiggles (see straighten)
STRAIGHT_TOL = 0.45   # m: the straight path may leave the driven one by this much ...
MIN_CLEAR = 0.25      # ... but must keep this clearance to the mapped walls (or what the drive had)
HAIRPIN_DEG = 100     # a corner sharper than this between two short pieces = a manual back-and-forth


def straighten(path, clear):
    """The driven path -> straight pieces between corners (2026-10-30, log_013: the manual drive is full
    of small left corrections against the rover's ~6 deg/s right drift, back-ups and shuffles; the
    rover can only drive straight and pivot, so straight lines with a few corners are the path it can
    follow best).
    Douglas-Peucker with STRAIGHT_TOL, but a straight piece is only accepted if it keeps
    min(MIN_CLEAR, the clearance the drive itself had there - 3 cm) to the mapped walls everywhere
    (clear(xy) = distance to the nearest LiDAR wall point, m). Then hairpins between short pieces
    are cut out (same clearance test), and the pieces are resampled every STEP m."""
    P = np.asarray(path, float)
    n = len(P)
    if n < 3:
        return path, np.arange(n) * STEP
    c_orig = clear(P)

    def seg_ok(a, b):
        A, B = P[a], P[b]
        L = float(np.hypot(*(B - A)))
        m = max(2, int(L / 0.05) + 1)
        q = A + (B - A) * np.linspace(0, 1, m)[:, None]
        need = min(MIN_CLEAR, float(c_orig[a:b + 1].min()) - 0.03)
        if clear(q).min() < need:
            return False, None
        if b - a < 2:
            return True, None
        if L < 1e-6:
            d = np.hypot(*(P[a + 1:b] - A).T)
        else:
            u = (B - A) / L
            d = np.abs((P[a + 1:b, 0] - A[0]) * u[1] - (P[a + 1:b, 1] - A[1]) * u[0])
        k = int(np.argmax(d))
        return bool(d[k] <= STRAIGHT_TOL), a + 1 + k

    keep = {0, n - 1}
    stack = [(0, n - 1)]
    while stack:
        a, b = stack.pop()
        good, k = seg_ok(a, b)
        if good or b - a < 2:
            continue
        if k is None:                                   # clearance failed: split in the middle
            k = (a + b) // 2
        keep.add(k)
        stack += [(a, k), (k, b)]
    V = sorted(keep)
    changed = True
    while changed and len(V) > 2:                       # cut hairpins (back-and-forth) out
        changed = False
        for j in range(1, len(V) - 1):
            a, m_, b = V[j - 1], V[j], V[j + 1]
            v1, v2 = P[m_] - P[a], P[b] - P[m_]
            l1, l2 = np.hypot(*v1), np.hypot(*v2)
            if l1 < 1e-6 or l2 < 1e-6:
                ang = 180.0
            else:
                ang = np.degrees(np.arccos(np.clip(np.dot(v1, v2) / (l1 * l2), -1, 1)))
            if ang > HAIRPIN_DEG and min(l1, l2) < 0.8:
                A, B = P[a], P[b]
                q = A + (B - A) * np.linspace(0, 1, max(2, int(np.hypot(*(B - A)) / 0.05) + 1))[:, None]
                if clear(q).min() >= min(MIN_CLEAR, float(c_orig[a:b + 1].min()) - 0.03):
                    V.pop(j); changed = True
                    break
    Q = P[V]
    d = np.r_[0, np.cumsum(np.hypot(*np.diff(Q, axis=0).T))]
    sp = np.arange(0, max(d[-1], STEP) + 1e-9, STEP)
    out = np.round(np.c_[np.interp(sp, d, Q[:, 0]), np.interp(sp, d, Q[:, 1])] * 100) / 100
    out[-1] = np.round(Q[-1] * 100) / 100                # end exactly at the leg end
    return out, sp


def build(teach_csv):
    A = pd.read_csv(teach_csv)
    S = A[[c for c in A.columns if c.startswith("lidar_")]].values.astype(float)
    Tw = LMAP.run(A, S, verbose=False)                      # x, y, th (rad), ok, score
    A2, S2 = ML.load(teach_csv)
    legs, _ = ML.build(A2, S2)
    xs = pd.Series(Tw[:, 0]).rolling(5, center=True, min_periods=1).mean().values
    ys = pd.Series(Tw[:, 1]).rolling(5, center=True, min_periods=1).mean().values
    dist = np.r_[0, np.cumsum(np.hypot(np.diff(xs), np.diff(ys)))]
    first = int(np.argmax(np.abs(A.out_l.values) + np.abs(A.out_r.values) > 0.05))
    out = []
    r0 = first
    for k, L in enumerate(legs):
        r1 = L["i1"]
        o = np.array([xs[r0], ys[r0], Tw[r0, 2]])
        # path
        P = to_frame(np.c_[xs[r0:r1 + 1], ys[r0:r1 + 1]], o)
        d = np.r_[0, np.cumsum(np.hypot(*np.diff(P, axis=0).T))]
        sp = np.arange(0, max(d[-1], STEP) + 1e-9, STEP)
        path = np.round(np.c_[np.interp(sp, d, P[:, 0]), np.interp(sp, d, P[:, 1])] * 100) / 100   # cm, as on the box
        path, sp = clean_path(path)
        # map: scans from BEFORE_M before the start to AFTER_M into the next leg
        ra = int(np.searchsorted(dist, dist[r0] - BEFORE_M))
        rb = min(len(A) - 1, int(np.searchsorted(dist, dist[r1] + AFTER_M)))
        pts = [to_frame(T(scan_pts(S[i]), Tw[i, 0], Tw[i, 1], Tw[i, 2]), o) for i in range(ra, rb + 1) if Tw[i, 3]]
        W = np.vstack(pts) if pts else np.zeros((0, 2))
        if len(W):   # flash budget: map only what lies within MAP_MARGIN of the leg's path box
            pl, ph = path.min(0) - MAP_MARGIN, path.max(0) + MAP_MARGIN
            W = W[(W[:, 0] > pl[0]) & (W[:, 0] < ph[0]) & (W[:, 1] > pl[1]) & (W[:, 1] < ph[1])]
        lo = np.floor((np.minimum(W.min(0) if len(W) else path.min(0), path.min(0)) - 1.0) * 100) / 100  # cm
        hi = np.maximum(W.max(0) if len(W) else path.max(0), path.max(0)) + 1.0
        n = np.minimum(((hi - lo) / RES).astype(int) + 1, 400)
        occ = np.ones(n, bool)
        ij = ((W - lo) / RES).astype(int)
        keep = (ij[:, 0] >= 0) & (ij[:, 0] < n[0]) & (ij[:, 1] >= 0) & (ij[:, 1] < n[1])
        occ[ij[keep, 0], ij[keep, 1]] = False
        grid = np.round(15 * np.exp(-0.5 * (distance_transform_edt(occ) * RES / SIG) ** 2)).astype(np.uint8)  # 4 bit
        dfield = distance_transform_edt(occ) * RES   # m to the nearest mapped wall point

        def clear(Q, dfield=dfield, lo=lo):
            ij = np.floor((np.asarray(Q) - lo) / RES).astype(int)
            ok = (ij[:, 0] >= 0) & (ij[:, 0] < dfield.shape[0]) & (ij[:, 1] >= 0) & (ij[:, 1] < dfield.shape[1])
            v = np.full(len(ij), 9.0)
            v[ok] = dfield[ij[ok, 0], ij[ok, 1]]
            return v
        path_raw = path
        if STRAIGHTEN:
            path, sp = straighten(path, clear)
        nxt_o = np.array([xs[r1 + 1], ys[r1 + 1], Tw[r1 + 1, 2]]) if r1 + 1 < len(A) else o
        out.append(dict(name=L["name"], kind=L["kind"], side=L["side"], end=L["end"], eff=L["eff"],
                        origin=o, path=path, path_raw=path_raw, s_end=float(sp[-1]), grid=grid, lo=lo,
                        next=pose_to_frame(nxt_o, o), r0=r0, r1=r1))
        r0 = r1 + 1
    return out, Tw


def save(model, fn):
    d = {"n": len(model)}
    for k, L in enumerate(model):
        for key in ("origin", "path", "s_end", "grid", "lo", "next", "eff", "r0", "r1"):
            d[f"{k}_{key}"] = L[key]
        d[f"{k}_path_raw"] = L.get("path_raw", L["path"])
        d[f"{k}_name"] = L["name"]
    np.savez_compressed(fn, **d)


def load(fn):
    z = np.load(fn, allow_pickle=True)
    return [dict(name=str(z[f"{k}_name"]), origin=z[f"{k}_origin"], path=z[f"{k}_path"], s_end=float(z[f"{k}_s_end"]),
                 grid=z[f"{k}_grid"], lo=z[f"{k}_lo"], next=z[f"{k}_next"], eff=float(z[f"{k}_eff"]),
                 r0=int(z[f"{k}_r0"]), r1=int(z[f"{k}_r1"]),
                 path_raw=z[f"{k}_path_raw"] if f"{k}_path_raw" in z else z[f"{k}_path"]) for k in range(int(z["n"]))]


# ---------------------------------------------------------------------------------------- localise
def score(grid, lo, W):
    i = ((W[..., 0] - lo[0]) / RES).astype(int)
    j = ((W[..., 1] - lo[1]) / RES).astype(int)
    ok = (i >= 0) & (i < grid.shape[0]) & (j >= 0) & (j < grid.shape[1])
    v = np.zeros(i.shape)
    v[ok] = grid[i[ok], j[ok]]
    return v.sum(-1) / 15.0


STAGES_WIDE = ((0.10, 15), np.arange(-20, 21, 4))     # (step m, half steps), headings deg
STAGE_COARSE = ((0.05, 8), np.arange(-4, 5, 2))
STAGE_FINE = ((0.02, 3), np.arange(-1, 2, 1))
STAGE_RELOC = ((0.05, 4), np.array([0]))


def _search(L, P, cx, cy, ct, stage, pred, lam):
    (step, nh), dths = stage
    offs = np.arange(-nh, nh + 1) * step
    DX, DY = [a.ravel() for a in np.meshgrid(offs, offs, indexing="ij")]
    best = None
    for dth in np.radians(dths):
        th = ct + dth
        R = T(P, 0, 0, th)
        Wp = R[None, :, :] + np.c_[cx + DX, cy + DY][:, None, :]
        sc = score(L["grid"], L["lo"], Wp) / len(P)
        sc = sc - lam * ((cx + DX - pred[0]) ** 2 + (cy + DY - pred[1]) ** 2) - 0.3 * (th - pred[2]) ** 2
        k = int(np.argmax(sc))
        if best is None or sc[k] > best[3]:
            best = (cx + DX[k], cy + DY[k], th, sc[k])
    return best


def match(L, P, pred, wide=False, lam=1.0):
    cx, cy, ct = pred
    stages = ((STAGES_WIDE,) if wide else ()) + (STAGE_COARSE, STAGE_FINE)
    best = None
    for st in stages:
        best = _search(L, P, cx, cy, ct, st, pred, lam)
        cx, cy, ct = best[:3]
    return np.array(best[:3]), best[3]


def relocate(L, P, pose, pi, span_m=3.0):
    """lost / off the path: try the taught path from span_m behind to span_m ahead of the current
    path index (every 0.5 m), heading = path direction -20..+20 deg in 10 deg steps, each with a
    small local search (+-0.2 m) and the fine stage"""
    path = L["path"]
    best = (None, -9.0)
    n = int(span_m / STEP)
    for q in range(max(0, pi - n), min(len(path), pi + n + 1), 2):
        a = path[min(len(path) - 1, q + 1)] - path[max(0, q - 1)]
        th0 = np.arctan2(a[1], a[0])
        for dth in np.radians(np.arange(-20, 21, 10)):
            c = np.array([path[q, 0], path[q, 1], th0 + dth])
            b1 = _search(L, P, c[0], c[1], c[2], STAGE_RELOC, c, 0.3)
            b2 = _search(L, P, b1[0], b1[1], b1[2], STAGE_FINE, c, 0.3)
            if b2[3] > best[1]:
                best = (np.array(b2[:3]), b2[3])
    return best


def project(path, xy, i0):
    lo, hi = max(0, i0 - 8), min(len(path), i0 + 40)
    d = np.hypot(*(path[lo:hi] - xy).T)
    k = lo + int(np.argmin(d))
    return k, float(d[k - lo])


def replay(model, csv, start_leg=0, accept=0.35):
    B = pd.read_csv(csv)
    S = B[[c for c in B.columns if c.startswith("lidar_")]].values.astype(float)
    yaw = np.unwrap(np.radians(B.yaw_iso_deg.values))
    l, r = B.out_l.values, B.out_r.values
    v = B.battery_v.values if "battery_v" in B else np.full(len(B), 11.6)
    eq = np.where((v > 9) & (v < 13.5), v / 11.6, 1.0)
    p0 = np.median(B.pitch_deg.values)
    i = int(np.argmax(np.abs(l) + np.abs(r) > 0.05))
    i_start = i
    k = start_leg
    pose = np.array([0.0, 0.0, 0.0])
    first = True
    pi = 0
    lost = 0
    rec = []
    while i < len(B) and k < len(model):
        L = model[k]
        standing = abs(l[i]) < 0.02 and abs(r[i]) < 0.02
        pivot = np.sign(l[i]) == -np.sign(r[i]) and abs(l[i]) > 0.05
        tilted = abs(B.pitch_deg.values[i] - p0) >= 5
        still = i - i_start >= 4 and LMAP.scan_still(S, i)   # like the box: history only since the start
        dth = yaw[i] - yaw[i - 1] if i else 0.0
        pose[2] = wrap(pose[2] + dth)
        if not (standing or pivot or tilted or still):
            d = K_SPEED * max(0.0, 0.5 * (l[i] + r[i])) * eq[i] * 0.25
            pose[:2] += d * np.array([np.cos(pose[2]), np.sin(pose[2])])
        P = scan_pts(S[i])
        sc, ok = 0.0, False
        if len(P) >= 12 and not tilted:
            m, sc = match(L, P, pose.copy(), wide=first)
            if sc > accept:
                pose, ok = m, True
                first = False
                lost = 0
            else:
                lost += 1
            _, off_now = project(L["path"], pose[:2], pi)
            if (lost >= 8 or off_now > 1.0) and len(P) >= 20:   # lost: search along the taught path
                m, sc2 = relocate(L, P, pose, pi)
                if m is not None and sc2 > accept + 0.1:
                    pose, ok, sc, lost = m, True, sc2, 0
        pi, off = project(L["path"], pose[:2], pi)
        s = pi * STEP
        rec.append((i, k, pose[0], pose[1], pose[2], s, off, sc, ok))
        if s >= L["s_end"] - 0.25 and k + 1 < len(model):   # leg end: hand the pose to the next frame
            pose = pose_to_frame(pose, L["next"])
            k += 1
            pi = 0
        i += 1
    return pd.DataFrame(rec, columns=["rec", "leg", "x", "y", "th", "s", "off", "score", "ok"]), B


def to_global(model, D):
    out = np.zeros((len(D), 2))
    for j, (k, x, y) in enumerate(zip(D.leg.values, D.x.values, D.y.values)):
        o = model[k]["origin"]
        c, s = np.cos(o[2]), np.sin(o[2])
        out[j] = o[0] + c * x - s * y, o[1] + s * x + c * y
    return out


def plot_model(model, Tw, fn, D=None, model_glob=None, title=""):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(figsize=(12, 12))
    cols = plt.cm.tab20(np.linspace(0, 1, len(model)))
    for k, L in enumerate(model):
        o = L["origin"]
        c, s = np.cos(o[2]), np.sin(o[2])
        g = L["grid"]
        ii, jj = np.where(g > 12)
        X = L["lo"][0] + ii * RES; Y = L["lo"][1] + jj * RES
        ax.scatter(o[0] + c * X - s * Y, o[1] + s * X + c * Y, s=1, color="#9aa4b2", alpha=0.15)
        P = L["path"]
        ax.plot(o[0] + c * P[:, 0] - s * P[:, 1], o[1] + s * P[:, 0] + c * P[:, 1], "-", color=cols[k], lw=2.5)
        ax.annotate("", xy=(o[0] + 1.2 * c, o[1] + 1.2 * s), xytext=(o[0], o[1]),
                    arrowprops=dict(arrowstyle="-|>", color="k", lw=1.2))
        ax.text(o[0], o[1], f" {k + 1}", fontsize=9, weight="bold")
    if D is not None:
        G = to_global(model, D)
        okm = D.ok.values.astype(bool)
        ax.plot(G[okm, 0], G[okm, 1], ".", color="#2ea043", ms=3, label="repeat drive: located (scan matched)")
        ax.plot(G[~okm, 0], G[~okm, 1], ".", color="#d62a1e", ms=3, label="repeat drive: prediction only")
        ax.legend(loc="best")
    ax.set_aspect("equal"); ax.grid(alpha=0.3)
    ax.set_title(title or "legs in their own frames (arrow = leg origin + x axis), coloured = taught path")
    plt.tight_layout(); plt.savefig(fn, dpi=100)


def export_h(model, fn, src):
    """cas_box/LocalPath.h: the leg maps for LocalLocalizer (two 4-bit grid cells per byte)"""
    with open(fn, "w") as f:
        f.write("#pragma once\n")
        f.write(f"// LocalPath - leg maps in local frames for LocalLocalizer. GENERATED by tools/localpath.py from {src}.\n")
        f.write("// Re-make together with make_legs.py / make_segments.py / make_webmap.py (same teaching log).\n")
        f.write("// Per leg: path (x, y in cm, every 25 cm, leg frame), grid (10 cm cells, 4-bit likelihood of a\n")
        f.write("// wall, two cells per byte: low nibble = even cell index, index = i * ny + j), lo = grid corner\n")
        f.write("// (cm), next = pose of the next leg's origin in this frame, origin = leg origin in the web map.\n")
        f.write("#include <stdint.h>\nnamespace LocalPathMap {\n")
        f.write(f"static const uint8_t kLegs = {len(model)};\n")
        f.write("struct Leg { uint16_t nPath; const int16_t* path; float sEnd; int16_t loX, loY; uint16_t nx, ny;\n"
                "  const uint8_t* grid; float nextX, nextY, nextTh, origX, origY, origTh; };\n")
        for k, L in enumerate(model):
            pc = np.round(L["path"] * 100).astype(int).ravel()
            f.write(f"static const int16_t kPath{k}[] = {{{','.join(map(str, pc))}}};\n")
            g = L["grid"].ravel()
            if len(g) % 2:
                g = np.r_[g, 0]
            packed = (g[0::2] & 15) | ((g[1::2] & 15) << 4)
            f.write(f"static const uint8_t kGrid{k}[] = {{{','.join(map(str, packed))}}};\n")
        f.write("static const Leg kLeg[kLegs] = {\n")
        for k, L in enumerate(model):
            lo = np.round(L["lo"] * 100).astype(int)
            nx, ny = L["grid"].shape
            n_, o = L["next"], L["origin"]
            f.write(f"  {{{len(L['path'])}, kPath{k}, {L['s_end']:.3f}f, {lo[0]}, {lo[1]}, {nx}, {ny}, kGrid{k}, "
                    f"{n_[0]:.4f}f, {n_[1]:.4f}f, {n_[2]:.5f}f, {o[0]:.4f}f, {o[1]:.4f}f, {o[2]:.5f}f}},  // {k + 1}: {L['name']}\n")
        f.write("};\n}  // namespace LocalPathMap\n")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    b = sub.add_parser("build"); b.add_argument("teach"); b.add_argument("--out", help="LocalPath.h path")
    r = sub.add_parser("replay"); r.add_argument("model"); r.add_argument("log"); r.add_argument("--leg", type=int, default=1)
    a = ap.parse_args()
    if a.cmd == "build":
        model, Tw = build(a.teach)
        fn = Path(a.teach).with_name(Path(a.teach).stem + "_local.npz")
        save(model, fn)
        hfn = Path(__file__).resolve().parent.parent / "cas_box" / "LocalPath.h"
        export_h(model, hfn if not a.out else Path(a.out), Path(a.teach).name)
        print(f"-> {hfn if not a.out else a.out}")
        plot_model(model, Tw, str(fn).replace(".npz", ".png"), title=f"{Path(a.teach).stem}: {len(model)} legs in local frames")
        for k, L in enumerate(model):
            print(f"leg {k + 1:>2} {L['name']:<36} path {L['s_end']:5.1f} m  grid {L['grid'].shape[0]}x{L['grid'].shape[1]}")
        print(f"-> {fn}")
    else:
        model = load(a.model)
        D, B = replay(model, a.log, a.leg - 1)
        print(f"{'leg':<40} {'records':>7} {'located':>8} {'score':>6} {'off path m':>10}")
        for k in sorted(D.leg.unique()):
            m = D.leg == k
            print(f"{k + 1:>2} {model[k]['name']:<37} {m.sum():>7} {100 * D.ok[m].mean():7.0f} % {D.score[m].mean():6.2f} {D.off[m].median():10.2f}")
        reached = D.leg.max() + 1
        print(f"reached leg {reached} of {len(model)}; located {100 * D.ok.mean():.0f} % of the records")
        fn = Path(a.log).with_name(Path(a.log).stem + "_localreplay.png")
        plot_model(model, None, str(fn), D, title=f"{Path(a.log).stem} localised leg by leg in the legs of {Path(a.model).stem}")
        D.to_csv(str(fn).replace(".png", ".csv"), index=False, float_format="%.3f")
        print(f"-> {fn}")


if __name__ == "__main__":
    main()
