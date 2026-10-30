#!/usr/bin/env python3
"""Map + trajectory of a drive log from IMU + LiDAR (+ ultrasonic layer) only - no GPS, no wheel odometry.

    python3 lidar_map.py log_012.csv        # -> log_012_map_slam.png, log_012_traj.csv

POSE PER RECORD (4 Hz)
  heading   gyro yaw (BNO055), refined by the scan match within +-3 deg
  position  correlative scan matching: the 90 LiDAR points (0.3-8 m, rear sector left out - that is
            the operator walking behind) are scored against a likelihood field (5 cm cells, blurred)
            of the recent keyframes, drawn as connected wall lines (neighbouring points < 0.45 m
            apart joined): single 4-deg samples of a side wall would pin the rover to "no motion". Brute-force search around the prediction (+-0.45 m, 3 cm steps,
            then 1 cm), headings -3..+3 deg. A weak prior towards the prediction decides only where the
            walls cannot (along a straight corridor).
  prediction  forward command * K_SPEED (1.0 m/s per command, measured with the LiDAR against walls
            ahead), along the gyro heading; 0 when the rover stands (no command) or pivots (wheels
            opposite) or the scan has not changed for 1 s (median < 2.5 cm: stuck, wheels slipping). The scan match corrects it wherever the walls show the motion; along a
            featureless straight corridor the prediction carries it.
  tilted records (|pitch - median| > 5 deg: LiDAR plane on the ground, rover stuck on the ramp edge
  or lifted) are skipped and the position is held (log_012: twice ~1 min at 10-13.5 deg with the
  wheels commanded but not moving).
MAP
  LiDAR points of every keyframe at its pose; ultrasonic hits (front / left / right / front angled,
  < 2.5 m) as a separate layer: they read 25-75 cm shorter than the LiDAR in the same direction
  (wide beam: kerbs, plants, ground) = low obstacles the LiDAR plane misses. Not used for matching.
"""
import argparse
from pathlib import Path

import numpy as np
import pandas as pd

DEG = np.arange(90) * 4 + 2.0
ANG = -np.radians(DEG)                        # rover frame, + = left
REAR = (DEG > 130) & (DEG < 230)
RES = 0.05                                    # likelihood field cell (m)
SIG = 0.07                                    # blur (m)
K_SPEED = 1.0     # m/s per forward command: measured with the LiDAR (range to walls ahead shrinking
                  # while driving straight, log_012: 0.35 m/s at 0.37 = 0.97); the match corrects it
US_DIR = {"us_front": 0.0, "us_left": 90.0, "us_right": -90.0, "us_front_angled": 0.0}


def body_points(cm):
    r = np.asarray(cm, float) / 100.0
    ok = (r > 0.3) & (r < 8.0) & ~REAR
    return np.c_[r[ok] * np.cos(ANG[ok]), r[ok] * np.sin(ANG[ok])]


def to_world(P, x, y, th):
    c, s = np.cos(th), np.sin(th)
    return np.c_[x + c * P[:, 0] - s * P[:, 1], y + s * P[:, 0] + c * P[:, 1]]


def densify(W, gap=0.45):
    """join neighbouring scan points (consecutive bins closer than gap) by lines, 2 cm steps: a wall
    sampled every 4 deg would otherwise look the same after the rover moves along it ("no motion")"""
    out = [W]
    if len(W) > 1:
        d = np.hypot(*np.diff(W, axis=0).T)
        for k in np.where(d < gap)[0]:
            m = max(2, int(d[k] / 0.02))
            t = np.linspace(0, 1, m)[1:-1, None]
            out.append(W[k] + t * (W[k + 1] - W[k]))
    return np.vstack(out)


class Field:
    """likelihood field of a point set: exp(-d^2 / 2 sig^2), d = distance to the nearest map point"""

    def __init__(self, Q):
        from scipy.ndimage import distance_transform_edt
        self.x0, self.y0 = Q.min(0) - 1.0
        nx, ny = ((Q.max(0) + 1.0 - (self.x0, self.y0)) / RES).astype(int) + 1
        occ = np.ones((nx, ny), bool)
        ij = ((Q - (self.x0, self.y0)) / RES).astype(int)
        occ[ij[:, 0], ij[:, 1]] = False
        d = distance_transform_edt(occ) * RES
        self.f = np.exp(-0.5 * (d / SIG) ** 2)
        self.nx, self.ny = nx, ny

    def score(self, W):
        """W: (..., n, 2) world points -> summed likelihood over n"""
        i = ((W[..., 0] - self.x0) / RES).astype(int)
        j = ((W[..., 1] - self.y0) / RES).astype(int)
        ok = (i >= 0) & (i < self.nx) & (j >= 0) & (j < self.ny)
        v = np.zeros(i.shape)
        v[ok] = self.f[i[ok], j[ok]]
        return v.sum(-1)


def match(P, field, pred, lam=1.5):
    """best (x, y, th) near pred = (x, y, th); returns pose, normalised score"""
    best = None
    for step, half in ((0.03, 0.45), (0.01, 0.04)):
        cx, cy = (pred[0], pred[1]) if best is None else (best[0], best[1])
        offs = np.arange(-half, half + 1e-9, step)
        DX, DY = np.meshgrid(offs, offs, indexing="ij")
        DX, DY = DX.ravel(), DY.ravel()
        ths = pred[2] + np.radians(np.arange(-3, 3.1, 1.0 if step > 0.02 else 0.5))
        if best is not None:
            ths = best[2] + np.radians(np.arange(-1, 1.1, 0.5))
        for th in ths:
            W0 = to_world(P, 0, 0, th)                                   # rotated body points
            W = W0[None, :, :] + np.c_[cx + DX, cy + DY][:, None, :]
            sc = field.score(W) / len(P)
            sc -= lam * ((cx + DX - pred[0]) ** 2 + (cy + DY - pred[1]) ** 2)   # weak prior
            sc -= 0.5 * (th - pred[2]) ** 2                                       # tiny heading prior
            k = int(np.argmax(sc))
            if best is None or sc[k] > best[3]:
                best = (cx + DX[k], cy + DY[k], th, sc[k])
    return best[:3], best[3]


def scan_still(S, i, span=4, tol_cm=2.5):
    """median change of the LiDAR points over the last second < tol: the rover does not move
    (same test as RouteFollow's stuck detection)"""
    if i < span:
        return False
    a, b = S[i], S[i - span]
    m = (a > 25) & (b > 25) & (a < 500) & (b < 500)
    return m.sum() >= 15 and float(np.median(np.abs(a[m] - b[m]))) < tol_cm


def run(A, S, verbose=True):
    yaw = np.unwrap(np.radians(A.yaw_iso_deg.values))
    yaw -= yaw[0]
    pitch = A.pitch_deg.values
    p0 = np.median(pitch)
    l, r = A.out_l.values, A.out_r.values
    n = len(A)
    T = np.zeros((n, 5))                         # x, y, th, ok, score
    x = y = 0.0
    th_off = 0.0                                 # scan-match correction of the gyro heading
    vx = vy = 0.0
    keys, kp, field = [], None, None
    for i in range(n):
        standing = abs(l[i]) < 0.02 and abs(r[i]) < 0.02 and (i == 0 or (abs(l[i - 1]) < 0.02 and abs(r[i - 1]) < 0.02))
        pivot = np.sign(l[i]) == -np.sign(r[i]) and abs(l[i]) > 0.05
        th_pred = yaw[i] + th_off
        tilted = abs(pitch[i] - p0) >= 5          # stuck on the ramp edge / lifted: wheels turn, no travel
        still = scan_still(S, i)                  # scene unchanged for 1 s: not moving, whatever the wheels do
        if standing or pivot or tilted or still:
            pred = (x, y, th_pred)
        else:                                     # speed guess from the forward command
            v = K_SPEED * max(0.0, 0.5 * (l[i] + r[i])) * 0.25
            pred = (x + v * np.cos(th_pred), y + v * np.sin(th_pred), th_pred)
        P = body_points(S[i])
        ok, sc = 0, 0.0
        if field is not None and abs(pitch[i] - p0) < 5 and len(P) >= 15:
            pose, sc = match(P, field, pred, lam=6.0 if standing else 1.5)
            if sc > 0.25:                         # enough points on known structure
                ok = 1
                x2, y2, th2 = pose
            else:
                x2, y2, th2 = pred
        else:
            x2, y2, th2 = pred
        th_off += 0.3 * ((th2 - yaw[i]) - th_off) if ok else 0.0
        x, y = x2, y2
        th = yaw[i] + th_off
        T[i] = x, y, th, ok, sc
        # keyframes also from unmatched scans (at the predicted pose): otherwise the map stops at the
        # last match, the rover drives out of it and nothing ever matches again
        if len(P) >= 15 and abs(pitch[i] - p0) < 5:
            if kp is None or np.hypot(x - kp[0], y - kp[1]) > 0.25 or abs(th - kp[2]) > np.radians(4):
                keys.append(densify(to_world(P, x, y, th)))
                kp = (x, y, th)
                keys = keys[-25:]
                field = Field(np.vstack(keys))
        if verbose and i % 1000 == 0:
            print(f"  {i}/{n}")
    return T


def plot(png, A, S, T, title):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    x, y, th, ok = T[:, 0], T[:, 1], T[:, 2], T[:, 3].astype(bool)
    fig, ax = plt.subplots(figsize=(12, 12))
    # LiDAR landmarks (matched records, every 2nd)
    px, py = [], []
    for i in range(0, len(A), 2):
        if not ok[i]:
            continue
        W = to_world(body_points(S[i]), x[i], y[i], th[i])
        px.append(W[:, 0]); py.append(W[:, 1])
    ax.scatter(np.concatenate(px), np.concatenate(py), s=1, color="#4a78b5", alpha=0.18,
               label="LiDAR landmarks (walls, railings, ramp edges)")
    # ultrasonic layer
    ux, uy = [], []
    for col, a in US_DIR.items():
        if col not in A:
            continue
        d = A[col].values / 100.0
        m = ok & (d > 0.05) & (d < 2.5)
        b = th[m] + np.radians(a)
        ux.append(x[m] + d[m] * np.cos(b)); uy.append(y[m] + d[m] * np.sin(b))
    ax.scatter(np.concatenate(ux), np.concatenate(uy), s=3, color="#e08a00", alpha=0.35, marker="x",
               label="ultrasonic hits < 2.5 m (low obstacles: kerbs, plants)")
    # trajectory: solid where the scan matched, dotted where it is only command speed x gyro
    sm = pd.Series(ok.astype(float)).rolling(20, center=True, min_periods=1).mean().values > 0.5
    for good, sty, lab in ((True, "-", "trajectory (LiDAR scan match)"),
                           (False, ":", "trajectory, few LiDAR points: command speed x gyro only")):
        xx = np.where(sm == good, x, np.nan); yy = np.where(sm == good, y, np.nan)
        ax.plot(xx, yy, sty, color="#d62a1e", lw=2.4 if good else 2.0, label=lab)
    mk = A.mark.values
    for i in range(1, len(A)):
        if mk[i] > mk[i - 1]:
            ax.plot(x[i], y[i], "o", color="k", ms=4)
            ax.text(x[i], y[i], f"  M{mk[i]}", fontsize=8)
    ax.plot(x[0], y[0], "s", color="green", ms=9, label="start of the log")
    ax.set_aspect("equal"); ax.grid(alpha=0.3)
    ax.set_xlabel("x (m) - direction of the rover at the start"); ax.set_ylabel("y (m) - left of it")
    ax.set_title(title); ax.legend(loc="best", fontsize=9)
    plt.tight_layout(); plt.savefig(png, dpi=100)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("log", help="CSV from analyze_log.py")
    a = ap.parse_args()
    A = pd.read_csv(a.log)
    S = A[[c for c in A.columns if c.startswith("lidar_")]].values.astype(float)
    T = run(A, S)
    base = Path(a.log).with_suffix("")
    pd.DataFrame(dict(t_s=A.t_s, x_m=T[:, 0], y_m=T[:, 1], theta_deg=np.degrees(T[:, 2]), matched=T[:, 3].astype(int),
                      score=T[:, 4], mark=A.mark)).to_csv(f"{base}_traj.csv", index=False, float_format="%.3f")
    plot(f"{base}_map_slam.png", A, S, T, f"{Path(a.log).stem}: map from IMU + LiDAR + ultrasonic only (no GPS)")
    L = np.sum(np.hypot(np.diff(T[:, 0]), np.diff(T[:, 1])))
    print(f"{len(A)} records, matched {100 * T[:, 3].mean():.0f} %, trajectory {L:.1f} m -> {base}_traj.csv, "
          f"{base}_map_slam.png")


if __name__ == "__main__":
    main()
