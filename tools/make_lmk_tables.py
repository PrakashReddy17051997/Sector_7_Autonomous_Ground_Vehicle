#!/usr/bin/env python3
"""make_lmk_tables.py - landmark phase tables (cas_box_v3/LandmarkRoute.h) from a teaching log (v3, 11.11.2026).

For every leg (Mark range, kRoute order) and every metre of the taught drive it looks for straight lines LEFT,
RIGHT and AHEAD with the firmware's own extractor (landmark_scan.extract = LineFeat.h), keeps only lines that run
parallel to the leg course (+-12 deg), and turns the leg into phases:
  CENTER  both side lines seen, corridor <= 4.5 m wide         -> stay in the middle (one side missing: keep its
                                                                 taught distance)
  SIDE    one side line seen (left preferred, <= 4.5 m away)    -> keep its taught distance (PID)
  HOLD    nothing usable                                         -> heading hold / taught path (if locked)
  STOP    a wall ahead in the last metres                        -> stop at its taught end distance
A SIDE run is split where the taught distance drifts by > 0.5 m (each piece its own dRef).
lineOff = taught course of the line - leg ref (deg): the crab / LiDAR mounting angle in the teaching run.

  python3 tools/make_lmk_tables.py cas_box_log/log_027.csv 6 15  > /tmp/tables.h   (legs 6..15, 1-based)
"""
import sys, math, statistics as st
sys.path.insert(0, __import__("os").path.dirname(__file__))
import landmark_scan as T

# kRoute (cas_box_027 / v3): leg refs (route yaw = log_027 gyro yaw) and Mark ranges
REF = [1.6, -84.1, -182.3, -94.1, -93.2, -82.4, -93.6, -99.2, -112.9, -183.0, -184.7, -183.9, -183.2, -183.5, -273.5]
MARKS = T.LEG_MARKS


def wrap(a):
    return (a + 180.0) % 360.0 - 180.0


def leg_bins(rows, ref):
    s0 = rows[0]["s"]
    bins = {}
    for r in rows:
        s = r["s"] - s0
        b = bins.setdefault(int(s), {"L": [], "R": [], "A": [], "n": 0})
        b["n"] += 1
        for k, foot in (("L", 90.0), ("R", -90.0), ("A", 0.0)):
            L = T.extract(r["scan"], foot, foot, r_min=0.3, r_max=7.0 if k != "A" else 10.0,
                          min_len=1.0 if k != "A" else 1.5)
            if not L:
                continue
            line_yaw = wrap(r["yaw"] - wrap(foot - L["foot"]))
            off = wrap(line_yaw - ref)
            if abs(off) > 22.0:              # not parallel to the leg course (crab ~ -10 is inside)
                continue
            b[k].append((L["d"], off))
    return bins, rows[-1]["s"] - s0


def label(b):
    n = max(1, b["n"])
    fL, fR = len(b["L"]) / n, len(b["R"]) / n
    dL = st.median([q[0] for q in b["L"]]) if b["L"] else 99
    dR = st.median([q[0] for q in b["R"]]) if b["R"] else 99
    if fL >= 0.5 and fR >= 0.5 and dL + dR <= 4.5:
        return "C"
    if fL >= 0.5 and dL <= 4.5:
        return "L"
    if fR >= 0.5 and dR <= 4.5:
        return "R"
    return "H"


def phases(bins, length):
    keys = sorted(bins)
    lab = [(k, label(bins[k])) for k in keys]
    runs = []
    for k, l in lab:
        if runs and runs[-1][0] == l and k == runs[-1][2] + 1:
            runs[-1][2] = k
        else:
            runs.append([l, k, k])
    # short runs (< 2 m) join the previous run
    out = []
    for r in runs:
        if out and r[2] - r[1] + 1 < 2:
            out[-1][2] = r[2]
        else:
            out.append(r)
    # split side runs where the distance drifts > 0.5 m
    res = []
    for l, a, b in out:
        if l in "LR":
            side = l
            cur = [a, a]
            ref = None
            for k in range(a, b + 1):
                ds = [q[0] for q in bins[k][side]]
                d = st.median(ds) if ds else None
                if ref is None and d is not None:
                    ref = d
                if d is not None and ref is not None and abs(d - ref) > 0.5 and k - cur[0] >= 2:
                    res.append([l, cur[0], k - 1]); cur = [k, k]; ref = d
                cur[1] = k
            res.append([l, cur[0], cur[1]])
        else:
            res.append([l, a, b])
    return res


def stats(bins, a, b, side):
    ds = [q[0] for k in range(a, b + 1) for q in bins.get(k, {"L": [], "R": [], "A": []})[side]]
    offs = [q[1] for k in range(a, b + 1) for q in bins.get(k, {"L": [], "R": [], "A": []})[side]]
    return (st.median(ds) if ds else None, st.median(offs) if offs else 0.0, len(ds))


def main():
    log = sys.argv[1]
    first, last = int(sys.argv[2]), int(sys.argv[3])
    rows_all = T.load(log)
    regs = []
    for leg in range(first, last + 1):
        m0, m1 = MARKS[leg - 1]
        rows = [r for r in rows_all if m0 <= r["mark"] < m1]
        ref = REF[leg - 1]
        bins, length = leg_bins(rows, ref)
        ph = phases(bins, length)
        lines = []
        for l, a, b in ph:
            s0, s1 = float(a), float(b + 1)
            if l == "L" or l == "R":
                d, off, n = stats(bins, a, b, l)
                foot = 90 if l == "L" else -90
                lo, hi = max(0.3, d - max(0.8, 0.35 * d)), d + max(0.8, 0.35 * d)
                lines.append("  {LandmarkLeg::SIDE, %d, %.1ff, %.1ff, %.2ff, %.2ff, %.2ff, 10, 1.5f, 20, 8, 10, 1.0f, 0.05f, 0, 0, 0, 0, %.1ff},  // %s line %.2f m (%d scans)"
                             % (foot, s0, s1, d, lo, hi, off, "left" if l == "L" else "right", d, n))
            elif l == "C":
                dL, offL, nL = stats(bins, a, b, "L")
                dR, offR, nR = stats(bins, a, b, "R")
                lines.append("  {LandmarkLeg::CENTER, 90, %.1ff, %.1ff, %.2ff, 0.3f, %.2ff, 10, 1.5f, 20, 8, 10, 1.0f, 0.05f, %.2ff, 0, 0, 0, %.1ff},  // corridor L %.2f / R %.2f m"
                             % (s0, s1, dL, dL + 1.5, dR, offL, dL, dR))
            else:
                lines.append("  {LandmarkLeg::HOLD, 0, %.1ff, %.1ff, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},  // no usable line: heading / taught path" % (s0, s1))
        # stop wall ahead: only a CLOSE wall at the end (<= 3.5 m) is a useful stop; far ones (5-8 m) are left to the
        # distance end + the localizer check. The side phases are cut where the STOP phase starts.
        tail = [k for k in sorted(bins) if k >= int(length) - 2]
        aD = [q for k in tail for q in bins[k]["A"]]
        nTail = sum(bins[k]["n"] for k in tail)
        if aD and len(aD) >= 0.4 * nTail:
            last = [q[0] for q in bins[max(tail)]["A"]] or [q[0] for q in aD]
            dEnd = st.median(last)
            offA = st.median([q[1] for q in aD])
            if 0.8 <= dEnd <= 3.5:
                sF = max(0.5 * length, length - 2.0)
                cut = []
                for x, (l, a, b) in zip(lines, ph):
                    if a >= sF:
                        continue
                    if b + 1 > sF:
                        x = x.replace("%.1ff, %.1ff," % (float(a), float(b + 1)), "%.1ff, %.1ff," % (float(a), sF), 1)
                    cut.append(x)
                lines = cut
                lines.append("  {LandmarkLeg::STOP, 0, %.1ff, %.1ff, %.2ff, 0.8f, 6.0f, 0, 0, 0, 0, 12, 1.5f, 0.05f, 0, 0, 0, 0, %.1ff},  // wall ahead, taught end %.2f m"
                             % (sF, length + 3.0, dEnd, offA, dEnd))
        # the last non-STOP phase ends exactly at the taught length (the firmware's position tops out there)
        if lines and "STOP" not in lines[-1]:
            a_last = ph[len(lines) - 1][1]
            lines[-1] = lines[-1].replace("%.1ff, %.1ff," % (float(a_last), float(ph[len(lines) - 1][2] + 1)),
                                          "%.1ff, %.1ff," % (float(a_last), length), 1)
        name = "kLeg%d" % leg
        print("// leg %d (M%d-M%d), taught %.1f m, ref %.1f deg" % (leg, m0, m1, length, ref))
        print("static const LmkPhase %s[] = {" % name)
        for x in lines:
            print(x)
        print("};")
        regs.append((leg, name, length))
    print("// registration: leg number (1-based), table")
    print("struct LmkLegReg { uint8_t leg; const LmkPhase* ph; uint8_t n; float lenM; };")
    print("static const LmkLegReg kLmkLegs[] = {")
    for leg, name, ln in regs:
        print("  {%d, %s, LMK_COUNT(%s), %.1ff}," % (leg, name, name, ln))
    print("};")


if __name__ == "__main__":
    main()
