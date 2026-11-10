#!/usr/bin/env python3
"""landmark_scan.py - find LANDMARK LINES in a drive log and suggest LmkPhase tables (v3, 2026-11-10).

Same line extraction as the firmware (cas_box_v3/LineFeat.h: RANSAC over all point pairs + TLS refit
on the 90-point snapshot). For every scan of a leg it looks LEFT / RIGHT / AHEAD / BEHIND and prints,
per metre of odometry, which long straight lines are there and how far away. That is what the
batchmate measured for the Open Sea ("store front 6.5-6.8 m in 5 of 9 runs, seen on 39-56 % of scans").

  python3 tools/landmark_scan.py cas_box_log/log_027.csv --marks 15 34          # one stretch, by Mark numbers
  python3 tools/landmark_scan.py cas_box_log/log_027.csv --legs                  # summary for every kRoute leg
  python3 tools/landmark_scan.py cas_box_log/log_027.csv --marks 71 72 --table   # + a C++ LmkPhase table

Odometry: s = M_PER_EFF * sum(mean forward output * dt) - the firmware's legMetres() (log_027: 1.03).
"""
import csv, math, sys, argparse

M_PER_EFF = 1.03
SECT = {"L": 90.0, "R": -90.0, "A": 0.0, "B": 180.0}
# kRoute legs of cas_box_027 (Mark numbers at the leg ends)
LEG_MARKS = [(1, 15), (15, 34), (34, 38), (38, 42), (42, 45), (45, 55), (55, 57), (57, 60), (60, 61), (61, 63),
             (63, 64), (64, 66), (66, 69), (69, 71), (71, 72)]


def wrap(a):
    return (a + 180.0) % 360.0 - 180.0


def extract(scan_cm, center, foot_exp, r_min=0.3, r_max=10.0, half=45.0, foot_tol=30.0, tol0=0.05, tolk=0.025,
            min_sep=0.4, min_pts=5, min_len=1.0, max_rms=0.12):
    """python twin of LineFeat::extract -> dict or None"""
    pts = []
    for i, c in enumerate(scan_cm):
        a = wrap(-(i * 4 + 2.0))
        if abs(wrap(a - center)) > half or c == 0:
            continue
        r = c / 100.0
        if r < r_min or r > r_max:
            continue
        pts.append((r * math.cos(math.radians(a)), r * math.sin(math.radians(a)), r))
    n = len(pts)
    if n < min_pts:
        return None
    best = (0, 0, 0, 0, 0)
    for i in range(n):
        for j in range(i + 1, n):
            tx, ty = pts[j][0] - pts[i][0], pts[j][1] - pts[i][1]
            sep = math.hypot(tx, ty)
            if sep < min_sep:
                continue
            tx, ty = tx / sep, ty / sep
            nx, ny = -ty, tx
            d = nx * pts[i][0] + ny * pts[i][1]
            if d < 0:
                nx, ny, d = -nx, -ny, -d
            if abs(wrap(math.degrees(math.atan2(ny, nx)) - foot_exp)) > foot_tol:
                continue
            ss = [-ny * x + nx * y for x, y, r in pts if abs(nx * x + ny * y - d) <= tol0 + tolk * r]
            span = max(ss) - min(ss) if ss else 0
            if len(ss) > best[0] or (len(ss) == best[0] and span > best[1]):
                best = (len(ss), span, nx, ny, d)
    if best[0] < min_pts:
        return None
    foot, d = math.degrees(math.atan2(best[3], best[2])), best[4]
    for _ in range(2):
        nx, ny = math.cos(math.radians(foot)), math.sin(math.radians(foot))
        use = [(x, y) for x, y, r in pts if abs(nx * x + ny * y - d) <= tol0 + tolk * r]
        if len(use) < 2:
            return None
        cx = sum(p[0] for p in use) / len(use); cy = sum(p[1] for p in use) / len(use)
        sxx = sum((p[0] - cx) ** 2 for p in use); syy = sum((p[1] - cy) ** 2 for p in use)
        sxy = sum((p[0] - cx) * (p[1] - cy) for p in use)
        th = 0.5 * math.atan2(2 * sxy, sxx - syy)
        nx, ny = -math.sin(th), math.cos(th)
        d = nx * cx + ny * cy
        if d < 0:
            nx, ny, d = -nx, -ny, -d
        foot = math.degrees(math.atan2(ny, nx))
        tr, det = sxx + syy, sxx * syy - sxy * sxy
        rms = math.sqrt(max(0.5 * (tr - math.sqrt(max(tr * tr - 4 * det, 0))), 0) / len(use))
    ss = [-ny * x + nx * y for x, y in use]
    ln = max(ss) - min(ss)
    ok = len(use) >= min_pts and ln >= min_len and rms <= max_rms and abs(wrap(foot - foot_exp)) <= foot_tol
    return dict(d=d, foot=foot, len=ln, rms=rms, pts=len(use)) if ok else None


def load(path):
    rows = list(csv.DictReader(open(path)))
    out, s, tprev = [], 0.0, None
    for r in rows:
        t = float(r["t_s"])
        v = 0.5 * (float(r["out_l"]) + float(r["out_r"]))
        if tprev is not None and v > 0:
            s += M_PER_EFF * v * (t - tprev)
        tprev = t
        scan = [int(float(r["lidar_%03d" % (4 * i)] or 0)) for i in range(90)]
        out.append(dict(t=t, s=s, mark=int(float(r["mark"] or 0)), leg=int(float(r["route_leg"] or 0)),
                        yaw=float(r["yaw_iso_deg"]), scan=scan))
    return out


def analyse(rows, title, table=False):
    if not rows:
        print(title, ": no rows"); return
    s0 = rows[0]["s"]
    print("\n== %s: %.1f m, %d scans ==" % (title, rows[-1]["s"] - s0, len(rows)))
    stats = {}
    for k, c in SECT.items():
        hits = []
        for r in rows:
            L = extract(r["scan"], c, c, min_len=2.0 if k in "AB" else 1.0)
            if L:
                hits.append((r["s"] - s0, L))
        stats[k] = hits
        if not hits:
            print("  %s: no line" % k); continue
        ds = sorted(h[1]["d"] for h in hits)
        med = ds[len(ds) // 2]
        print("  %s: line on %3.0f %% of scans, d median %.2f m (p10 %.2f, p90 %.2f), seen s %.1f..%.1f m" %
              (k, 100.0 * len(hits) / len(rows), med, ds[len(ds) // 10], ds[9 * len(ds) // 10], hits[0][0], hits[-1][0]))
        # 2 m bins: how often, which distance
        line = "     s:"
        for b in range(0, int(rows[-1]["s"] - s0) + 2, 2):
            inb = [h[1]["d"] for h in hits if b <= h[0] < b + 2]
            nb = sum(1 for r in rows if b <= r["s"] - s0 < b + 2)
            line += " %d:%s" % (b, ("%.1f" % sorted(inb)[len(inb) // 2]) if inb and nb and len(inb) >= 0.3 * nb else "-")
        print(line)
    if table:
        suggest(stats, rows[-1]["s"] - s0)


def suggest(stats, total):
    """a first LmkPhase table: anchor behind, side lines where steady, stop wall ahead"""
    print("\n  // suggested LmkPhase table (check every number against a 2nd log before flashing)")
    print("  // {type, footExp, sFrom, sTo, dRef, dMin, dMax, kp, ki, kd, sat, para, minLen, anchor, a, a2, b, c}")
    B = [h for h in stats["B"] if h[0] < 0.35 * total]
    if len(B) >= 5:
        e = max(h[0] for h in B)
        print("  {LandmarkLeg::ANCHOR, 180, 0.0f, %.1ff, 0, 0.5f, 11.0f, 0, 0, 0, 0, 8, 2.0f, 0.10f, 0, 0, 0, 0}," % e)
    for k, foot in (("L", 90), ("R", -90)):
        H = stats[k]
        if len(H) < 8:
            continue
        # longest stretch where the distance stays within +-0.6 m of its median
        H = sorted(H, key=lambda h: h[0])
        best = None
        i = 0
        while i < len(H):
            j, ds = i, [H[i][1]["d"]]
            while j + 1 < len(H) and abs(H[j + 1][1]["d"] - sorted(ds)[len(ds) // 2]) < 0.6 and H[j + 1][0] - H[j][0] < 4.0:
                j += 1; ds.append(H[j][1]["d"])
            if j - i >= 6 and (best is None or H[j][0] - H[i][0] > best[1] - best[0]):
                best = (H[i][0], H[j][0], sorted(ds)[len(ds) // 2])
            i = j + 1
        if best:
            d = best[2]
            print("  {LandmarkLeg::SIDE, %d, %.1ff, %.1ff, %.2ff, %.1ff, %.1ff, 10, 1.5f, 20, 8, 8, %.1ff, 0.05f, 0, 0, 0, 0},  // %s line" %
                  (foot, best[0], best[1], d, max(0.4, d - 2.5), d + 3.0, 2.0 if d > 3 else 1.0, k))
    A = [h for h in stats["A"] if h[0] > 0.6 * total]
    if len(A) >= 3:
        print("  {LandmarkLeg::STOP, 0, %.1ff, %.1ff, 1.5f, 0.8f, 9.0f, 0, 0, 0, 0, 10, 2.0f, 0.10f, 0, 0, 0, 0},  // wall ahead, d at end %.2f m" %
              (0.6 * total, total + 3.0, A[-1][1]["d"]))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("log")
    ap.add_argument("--marks", nargs=2, type=int)
    ap.add_argument("--legs", action="store_true")
    ap.add_argument("--leg", type=int, help="mission log: route_leg number")
    ap.add_argument("--table", action="store_true")
    a = ap.parse_args()
    rows = load(a.log)
    if a.marks:
        m0, m1 = a.marks
        analyse([r for r in rows if m0 <= r["mark"] < m1], "M%d-M%d" % (m0, m1), a.table)
    elif a.leg:
        analyse([r for r in rows if r["leg"] == a.leg], "leg %d" % a.leg, a.table)
    elif a.legs:
        for i, (m0, m1) in enumerate(LEG_MARKS):
            analyse([r for r in rows if m0 <= r["mark"] < m1], "leg %d (M%d-M%d)" % (i + 1, m0, m1), a.table)
    else:
        analyse(rows, a.log, a.table)


if __name__ == "__main__":
    main()
