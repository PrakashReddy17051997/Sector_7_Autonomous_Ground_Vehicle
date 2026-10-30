#!/usr/bin/env python3
"""Decode and plot a CAS Box drive log (log_NNN.bin from the box web page).

Usage:
    python3 analyze_log.py log_001.bin [--heading gps|imu] [--decl 4.0] [--show-magcal] [--no-show]

Writes next to the log file:
    log_001.csv           one row per record (4 per second), LiDAR as lidar_000..lidar_356 in cm
    log_001_overview.png  GPS track, tilt, side distances, heading, GPS quality / safety stops
    log_001_map.png       LiDAR points placed with GPS position + IMU heading (rough map)

Mark tags (log version 2, 2026-10-29): the tag chosen on the page with every Mark (how the segment
was driven, which feature ends it, which side) is decoded into the CSV column mark_tag and printed
per mark together with what the LiDAR saw left / right / ahead at the press. Version 1 logs
(log_009 ... log_011) still decode as before (mark_tag = 0). Tag codes: marktags.py / DriveLog.h.

Heading (changed 2026-10-26):
    Since log_004 the BNO055 runs in IMUPLUS mode (gyro + accelerometer, no magnetometer), so the
    logged yaw is RELATIVE: 0 = the direction the box faced at power-on. It is not a compass.
    --heading gps (default): the IMU heading is aligned to true north with the GPS course over
        ground. On straight, moving stretches (>= 1 m in 4 s, yaw change < 10 deg) the difference
        GPS course - IMU heading is measured; a running circular mean over +-60 s of those
        differences is added to the IMU heading. This also removes slow gyro drift.
    --heading imu: old behaviour, heading = -yaw + decl (only meaningful with the magnetometer).

Grey "mag cal" line (BNO055 magnetometer calibration status 0..3) is a status number, not a
sensor signal, and unused in IMUPLUS mode - only drawn with --show-magcal.

Needs: numpy, matplotlib   (python3 -m pip install numpy matplotlib)
"""
import argparse
import csv
import math
import struct
import sys
import warnings
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).parent))
import marktags as mt  # noqa: E402  mark tag codes (DriveLog.h)

HEADER = struct.Struct("<6sBBHHI")  # magic, version, lidarPoints, recordSize, periodMs, startMs
R_EARTH = 6371000.0
SAFETY = {0: "ok", 1: "tilt", 2: "front US", 3: "front-angled US", 4: "LiDAR front", 5: "back US"}


_VERSION = 2   # of the last loaded file; v3: the GPS fields carry route debug (DriveLog.h Sample)


def record_dtype(lidar_points, version=2):
    tail = [("markTag", "u1"), ("reserved", "u1")] if version >= 2 else []  # v2: tag of the latest mark
    return np.dtype([
        ("ms", "<u4"), ("latE7", "<i4"), ("lonE7", "<i4"), ("hdopX100", "<u2"),
        ("sats", "u1"), ("flags", "u1"),
        ("yawX100", "<i2"), ("pitchX100", "<i2"), ("rollX100", "<i2"),
        ("magCal", "u1"), ("mark", "u1"),
        ("cmdL", "i1"), ("cmdR", "i1"), ("outL", "i1"), ("outR", "i1"),
        ("safety", "u1"), ("rtt10", "u1"), ("batteryX100", "<u2"),
        ("us", "<u2", (5,)), ("lidar", "<u2", (lidar_points,)),
    ] + tail)


def load(path):
    p = Path(path).expanduser()
    if not p.exists():
        sys.exit(f"file not found: {p}\n(current folder: {Path.cwd()}) - check the path or use the full path")
    raw = p.read_bytes()
    if len(raw) < HEADER.size:
        sys.exit(f"{path}: file too short")
    magic, version, npts, rsize, period, start_ms = HEADER.unpack_from(raw)
    global _VERSION
    if magic != b"CASLOG" or version not in (1, 2, 3):
        sys.exit(f"{path}: not a CAS log (magic {magic!r}, version {version})")
    _VERSION = version
    dt = record_dtype(npts, version)
    if dt.itemsize != rsize:
        sys.exit(f"{path}: record size {rsize} in file, decoder expects {dt.itemsize}")
    body = raw[HEADER.size:]
    n = len(body) // rsize  # a record cut off by a power loss is dropped
    rec = np.frombuffer(body[: n * rsize], dtype=dt)
    meta = dict(lidar_points=npts, period_ms=period, start_ms=start_ms, records=n, version=version)
    return rec, meta


def _wrap180(a):
    return (np.asarray(a) + 180.0) % 360.0 - 180.0


def _circ_mean(deg):
    r = np.radians(deg)
    return math.degrees(math.atan2(np.sin(r).sum(), np.cos(r).sum()))


def gps_course(d, span_s=4.0, min_move_m=1.0, max_turn_deg=10.0):
    """GPS course over ground on straight, moving stretches.

    Returns (times, course_deg, imu_heading_deg) at real GPS fixes where, over the next span_s,
    the rover moved >= min_move_m and the IMU heading changed < max_turn_deg."""
    fix = d["fix"]
    changed = np.r_[True, (np.diff(d["lat"]) != 0) | (np.diff(d["lon"]) != 0)]
    g = np.flatnonzero(fix & changed)  # real 1 Hz GPS updates
    t, e, n, h = d["t"], d["east"], d["north"], d["heading_rel"]
    out_t, out_c, out_h = [], [], []
    j = 0
    for a, i in enumerate(g):
        while j < len(g) and t[g[j]] < t[i] + span_s:
            j += 1
        if j >= len(g):
            break
        k = g[j]
        de, dn = e[k] - e[i], n[k] - n[i]
        if math.hypot(de, dn) < min_move_m:
            continue
        seg = d["yaw"][i:k + 1]
        if np.ptp(np.degrees(np.unwrap(np.radians(seg)))) > max_turn_deg:
            continue
        hm = np.mod(_circ_mean(h[i:k + 1]), 360.0)
        out_t.append(0.5 * (t[i] + t[k]))
        out_c.append(math.degrees(math.atan2(de, dn)) % 360.0)
        out_h.append(hm)
    return np.array(out_t), np.array(out_c), np.array(out_h)


def align_heading(d, window_s=60.0, min_samples=5):
    """Offset (deg) per record so that heading_rel + offset = true heading, from the GPS course.

    Running circular mean of (course - imu heading) within +-window_s; samples more than 30 deg
    from the global mean are dropped as outliers (GPS jumps, sliding). Returns (offset array,
    info dict) or (None, info) if there are too few straight moving stretches."""
    tc, course, himu = gps_course(d)
    diff = _wrap180(course - himu)
    info = dict(samples=len(diff))
    if len(diff) < min_samples:
        return None, info
    g = _circ_mean(diff)
    keep = np.abs(_wrap180(diff - g)) <= 30.0
    tc, diff = tc[keep], diff[keep]
    info.update(samples=len(diff), global_deg=_circ_mean(diff),
                spread_deg=float(np.std(_wrap180(diff - _circ_mean(diff)))))
    if len(diff) < min_samples:
        return None, info
    off = np.empty(len(d["t"]))
    gl = info["global_deg"]
    for i, ti in enumerate(d["t"]):
        m = np.abs(tc - ti) <= window_s
        off[i] = _circ_mean(diff[m]) if m.sum() >= 3 else gl
    return off, info


def derive(rec, decl_deg, heading_mode="gps"):
    """Physical units + local east/north metres around the first GPS fix."""
    t = (rec["ms"].astype(np.int64) - int(rec["ms"][0])) / 1000.0
    fix = (rec["flags"] & 1) > 0
    if _VERSION >= 3:
        fix = np.zeros(len(rec), bool)   # v3: no GPS in the record
    lat = rec["latE7"] / 1e7
    lon = rec["lonE7"] / 1e7
    if fix.any():
        i0 = int(np.argmax(fix))
        lat0, lon0 = lat[i0], lon[i0]
        k = math.pi / 180 * R_EARTH
        north = (lat - lat0) * k
        east = (lon - lon0) * k * math.cos(math.radians(lat0))
        north[~fix] = np.nan
        east[~fix] = np.nan
    else:
        lat0 = lon0 = float("nan")
        north = east = np.full(len(rec), np.nan)
    yaw = rec["yawX100"] / 100.0                  # ISO: + = left; IMUPLUS: 0 = power-on direction
    heading_rel = np.mod(-yaw, 360.0)             # clockwise, relative to the power-on direction
    d = dict(
        t=t, fix=fix, lat=lat, lon=lon, lat0=lat0, lon0=lon0, east=east, north=north,
        hdop=rec["hdopX100"] / 100.0, sats=rec["sats"].astype(int),
        imu=(rec["flags"] & 2) > 0, lidar_ok=(rec["flags"] & 4) > 0,
        yaw=yaw, heading_rel=heading_rel,
        pitch=rec["pitchX100"] / 100.0, roll=rec["rollX100"] / 100.0,
        # magCal byte: bit 7 set = it carries the route leg / state (Mission), else BNO055 mag cal
        mag=np.where(rec["magCal"] & 0x80, 0, rec["magCal"]).astype(int), mark=rec["mark"].astype(int),
        route_leg=(np.where(rec["reserved"] > 0, rec["reserved"], np.where(rec["magCal"] & 0x80, rec["magCal"] & 15, 0))
                   if "reserved" in rec.dtype.names else np.where(rec["magCal"] & 0x80, rec["magCal"] & 15, 0)).astype(int),
        route_state=np.where(rec["magCal"] & 0x80, (rec["magCal"] >> 4) & 7, 0).astype(int),
        cmdL=rec["cmdL"] / 200.0, cmdR=rec["cmdR"] / 200.0,
        outL=rec["outL"] / 200.0, outR=rec["outR"] / 200.0,
        safety=rec["safety"].astype(int), battery=rec["batteryX100"] / 100.0,
        rtt=rec["rtt10"].astype(int) * 10,
        mark_tag=(rec["markTag"].astype(int) if "markTag" in rec.dtype.names else np.zeros(len(rec), int)),
    )
    if _VERSION >= 3:   # route debug (DriveLog.h Sample::dbg and the fields in the GPS slots)
        la = rec["latE7"].astype(np.int64) & 0xFFFFFFFF
        lo = rec["lonE7"].astype(np.int64) & 0xFFFFFFFF
        s16 = lambda v: ((v.astype(np.int64) & 0xFFFF) ^ 0x8000) - 0x8000
        fl = rec["flags"].astype(int)
        d.update(lp_x=s16(la >> 16) / 100.0, lp_y=s16(la) / 100.0, lp_th=s16(lo >> 16) / 10.0,
                 carrot_err=s16(lo) / 10.0, ctrl_err=s16(rec["hdopX100"]) / 10.0,
                 lp_score=rec["sats"] / 100.0, dbg_tap=(fl >> 3) & 1, dbg_settle=(fl >> 4) & 1,
                 dbg_kick=(fl >> 5) & 1, dbg_step=(fl >> 6) & 1, located=(fl >> 7) & 1)
        d["hdop"] = np.zeros(len(rec)); d["sats"] = np.zeros(len(rec), int)
        d["lat"] = d["lon"] = np.zeros(len(rec))
    d["heading_imu"] = np.mod(-yaw + decl_deg, 360.0)   # old formula (magnetometer mode)
    d["heading_src"] = "IMU (relative)"
    d["align"] = dict(samples=0)
    if heading_mode == "gps" and fix.any():
        off, info = align_heading(d)
        d["align"] = info
        if off is not None:
            d["heading"] = np.mod(heading_rel + off, 360.0)
            d["heading_src"] = "IMU aligned to GPS course"
        else:
            d["heading"] = heading_rel
    elif heading_mode == "imu":
        d["heading"] = d["heading_imu"]
        d["heading_src"] = "IMU yaw + declination"
    else:
        d["heading"] = heading_rel
    return d


def sector_min(lidar, center_deg, half_deg=8):
    """Closest LiDAR return (cm) within center +- half_deg, per record; NaN if none."""
    npts = lidar.shape[1]
    step = 360 / npts
    idx = [int(round(((center_deg + a) % 360) / step)) % npts
           for a in np.arange(-half_deg, half_deg + 0.1, step)]
    sub = lidar[:, sorted(set(idx))].astype(float)
    sub[sub == 0] = np.nan
    with warnings.catch_warnings():  # rows without any return -> NaN, no warning
        warnings.simplefilter("ignore", RuntimeWarning)
        return np.nanmin(sub, axis=1)


def mark_events(d):
    """(record index, mark number) at each Mark press."""
    m = d["mark"]
    return [(i, int(m[i])) for i in range(1, len(m)) if m[i] > m[i - 1]]


DEBUG_COLS = ["lp_x", "lp_y", "lp_th", "carrot_err", "ctrl_err", "lp_score",
              "dbg_tap", "dbg_settle", "dbg_kick", "dbg_step", "located"]


def write_csv(path, rec, d):
    """Column names unchanged for the other tools; heading_true_deg now uses the selected
    heading source (default: IMU aligned to GPS); heading_rel_deg added after route_state."""
    npts = rec["lidar"].shape[1]
    cols = ["t_s", "lat", "lon", "east_m", "north_m", "gps_fix", "sats", "hdop",
            "yaw_iso_deg", "heading_true_deg", "pitch_deg", "roll_deg", "mag_cal", "mark",
            "cmd_l", "cmd_r", "out_l", "out_r", "safety", "battery_v", "rover_rtt_ms",
            "route_leg", "route_state", "heading_rel_deg", "mark_tag",
            "us_front", "us_left", "us_right", "us_back", "us_front_angled"]
    cols += [f"lidar_{i * 360 // npts:03d}" for i in range(npts)]
    dbg = [k for k in DEBUG_COLS if k in d]   # v3 logs: route debug at the end (other tools unaffected)
    cols += dbg
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(cols)
        for i in range(len(rec)):
            row = [f"{d['t'][i]:.2f}", f"{d['lat'][i]:.7f}", f"{d['lon'][i]:.7f}",
                   f"{d['east'][i]:.2f}", f"{d['north'][i]:.2f}", int(d["fix"][i]),
                   d["sats"][i], f"{d['hdop'][i]:.2f}", f"{d['yaw'][i]:.2f}",
                   f"{d['heading'][i]:.2f}", f"{d['pitch'][i]:.2f}", f"{d['roll'][i]:.2f}",
                   d["mag"][i], d["mark"][i], d["cmdL"][i], d["cmdR"][i], d["outL"][i],
                   d["outR"][i], d["safety"][i], f"{d['battery'][i]:.2f}", d["rtt"][i],
                   d["route_leg"][i], d["route_state"][i], f"{d['heading_rel'][i]:.2f}", d["mark_tag"][i]]
            row += [int(v) for v in rec["us"][i]] + [int(v) for v in rec["lidar"][i]]
            row += [d[k][i] if k.startswith(("dbg", "loc")) else f"{d[k][i]:.2f}" for k in dbg]
            w.writerow(row)


def summary(rec, meta, d):
    n = len(rec)
    dur = d["t"][-1] if n else 0
    print(f"records       {n}  ({dur:.0f} s at {meta['period_ms']} ms)")
    print(f"GPS fix       {100 * d['fix'].mean():.0f} % of records", end="")
    if d["fix"].any():
        print(f", HDOP median {np.median(d['hdop'][d['fix']]):.1f}, "
              f"max {d['hdop'][d['fix']].max():.1f}, sats median {int(np.median(d['sats'][d['fix']]))}")
        f = np.flatnonzero(d["fix"])
        k = min(8, len(f))  # average 2 s at both ends: one GPS sample can be metres off
        e0, n0 = np.mean(d["east"][f[:k]]), np.mean(d["north"][f[:k]])
        e1, n1 = np.mean(d["east"][f[-k:]]), np.mean(d["north"][f[-k:]])
        print(f"start -> end  {math.hypot(e1 - e0, n1 - n0):.0f} m straight line, "
              f"origin {d['lat0']:.7f}, {d['lon0']:.7f}")
    else:
        print()
    a = d["align"]
    if "global_deg" in a and d["heading_src"].startswith("IMU aligned"):
        print(f"heading       {d['heading_src']}: offset {a['global_deg']:+.0f} deg (mean), "
              f"spread {a['spread_deg']:.0f} deg, from {a['samples']} straight stretches")
    else:
        print(f"heading       {d['heading_src']}"
              + ("  (too few straight GPS stretches to align)" if d["heading_src"] == "IMU (relative)" else ""))
    if d["imu"].any():
        print(f"tilt          max |pitch| {np.abs(d['pitch']).max():.1f} deg, "
              f"max |roll| {np.abs(d['roll']).max():.1f} deg")
    stops = {SAFETY.get(k, k): int((d["safety"] == k).sum()) for k in np.unique(d["safety"]) if k}
    print(f"safety stops  {stops if stops else 'none'}  (records)")
    bat = d["battery"][d["battery"] > 0]
    if len(bat):
        print(f"battery       {bat.max():.2f} -> {bat.min():.2f} V")
    rt = d["rtt"][d["rtt"] > 0]
    if len(rt):
        print(f"rover delay   median {np.median(rt):.0f} ms, 90% {np.percentile(rt, 90):.0f} ms, max {rt.max()} ms (box -> rover command)")
    tagged = d["mark_tag"].any()
    if tagged:  # v2 log with mark tags: what was tagged and what the LiDAR saw at the press
        L, R, F = (sector_min(rec["lidar"], a, 12) / 100 for a in (270, 90, 0))
        fm = lambda v: f"{v:4.2f}" if v == v else "  - "
        kept = {m for _, m, _ in mt.presses(d["mark"], d["mark_tag"])}
    for i, m in mark_events(d):
        pos = (f"E {d['east'][i]:+.1f} m, N {d['north'][i]:+.1f} m"
               if not math.isnan(d["east"][i]) else "no GPS fix")
        line = (f"mark {m:<3}     t = {d['t'][i]:6.1f} s   {pos}, heading {d['heading'][i]:.0f} deg, "
                f"pitch {d['pitch'][i]:+.1f}")
        if tagged:
            t = d["mark_tag"][i]
            state = "" if (m in kept or t & 15 == mt.UNDO) else "  (cancelled by undo)"
            line += (f"\n              {mt.text(t):<28} LiDAR left {fm(L[i])}  right {fm(R[i])}  "
                     f"ahead {fm(F[i])} m{state}")
        print(line)


def _style(plt):
    plt.rcParams.update({
        "font.size": 10, "axes.titlesize": 11, "axes.titleweight": "bold", "axes.labelsize": 10,
        "legend.fontsize": 8.5, "legend.frameon": True, "legend.framealpha": 0.9,
        "axes.spines.top": False, "axes.spines.right": False, "axes.grid": True,
        "grid.alpha": 0.25, "figure.dpi": 100,
    })


def _group_labels(items):
    """items: [(i, m, lab)] -> [(i, m, text or None)]; a labelled mark followed by unlabelled
    ones gets the text 'Mfirst-Mlast' (e.g. M4-M9), the others get None."""
    out = []
    for k, (i, m, lab) in enumerate(items):
        if not lab:
            out.append((i, m, None))
            continue
        j = k
        while j + 1 < len(items) and not items[j + 1][2]:
            j += 1
        out.append((i, m, f"M{m}" if j == k else f"M{m}–M{items[j][1]}"))
    return out


def _thin_marks(d, min_gap_s):
    """Mark events, labelled only if at least min_gap_s after the last labelled one (groups merged)."""
    out, last = [], -1e9
    for i, m in mark_events(d):
        lab = d["t"][i] - last >= min_gap_s
        out.append((i, m, lab))
        if lab:
            last = d["t"][i]
    return _group_labels(out)


def _thin_marks_xy(d, min_dist_m):
    """Mark events with a label only if at least min_dist_m away from the last labelled position."""
    out, last = [], None
    for i, m in mark_events(d):
        e, n = d["east"][i], d["north"][i]
        if math.isnan(e):
            continue
        lab = last is None or math.hypot(e - last[0], n - last[1]) >= min_dist_m
        out.append((i, m, lab))
        if lab:
            last = (e, n)
    return _group_labels(out)


def _log_name(path):
    return Path(path).name.replace("_overview.png", "").replace("_map.png", "")


def plot_overview(path, rec, d, show, show_magcal=False):
    import matplotlib.pyplot as plt
    _style(plt)
    t = d["t"]
    span = max(t[-1] - t[0], 1.0)
    marks = _thin_marks(d, 0.025 * span)
    name = _log_name(path)

    fig = plt.figure(figsize=(14, 10), constrained_layout=True)
    gs = fig.add_gridspec(4, 2, width_ratios=[1, 1.45])

    # (a) GPS track
    ax = fig.add_subplot(gs[:, 0])
    if d["fix"].any():
        sc = ax.scatter(d["east"], d["north"], c=t, s=5, cmap="viridis")
        fig.colorbar(sc, ax=ax, label="Time (s)", shrink=0.8)
        f = np.flatnonzero(d["fix"])
        ax.plot(d["east"][f[0]], d["north"][f[0]], "o", color="tab:green", ms=9, label="Start")
        ax.plot(d["east"][f[-1]], d["north"][f[-1]], "s", color="tab:red", ms=9, label="End")
        for i, m, lab in _thin_marks_xy(d, 3.0):
            if lab:
                ax.annotate(lab, (d["east"][i], d["north"][i]), color="crimson", fontsize=8,
                            xytext=(5, 3), textcoords="offset points")
        ax.legend(loc="best")
    else:
        ax.text(0.5, 0.5, "No GPS fix in this log", ha="center", transform=ax.transAxes)
    ax.set_aspect("equal", adjustable="datalim")
    ax.set_xlabel("East (m)")
    ax.set_ylabel("North (m)")
    ax.set_title("(a) GPS track", loc="left")

    def marklines(a, labels):
        for i, m, lab in marks:
            a.axvline(t[i], color="crimson", lw=0.6, alpha=0.45)
            if labels and lab:
                a.text(t[i], 1.01, lab, color="crimson", transform=a.get_xaxis_transform(),
                       ha="center", va="bottom", fontsize=7)

    # (b) tilt
    a1 = fig.add_subplot(gs[0, 1])
    a1.axhspan(-12, 12, color="tab:green", alpha=0.07, label="±12° limit")
    a1.plot(t, d["pitch"], lw=0.9, label="Pitch (front down +)")
    a1.plot(t, d["roll"], lw=0.9, label="Roll (right down +)")
    a1.set_ylabel("Angle (°)")
    a1.set_title("(b) Vehicle tilt", loc="left", pad=14 if marks else 6)
    a1.legend(loc="upper right", ncol=3)

    # (c) LiDAR clearance
    a2 = fig.add_subplot(gs[1, 1], sharex=a1)
    lid = rec["lidar"]
    a2.plot(t, sector_min(lid, 270), lw=0.9, label="Left (270° ± 8°)")
    a2.plot(t, sector_min(lid, 90), lw=0.9, label="Right (90° ± 8°)")
    a2.plot(t, sector_min(lid, 0, 20), lw=0.9, alpha=0.6, label="Front (0° ± 20°)")
    a2.set_ylim(0, 500)
    a2.set_ylabel("Distance (cm)")
    a2.set_title("(c) LiDAR clearance", loc="left")
    a2.legend(loc="upper right", ncol=3)

    # (d) heading
    a3 = fig.add_subplot(gs[2, 1], sharex=a1)
    src = "IMU heading (GPS-aligned)" if d["heading_src"].startswith("IMU aligned") else \
          ("IMU heading (relative)" if d["heading_src"] == "IMU (relative)" else "IMU heading")
    a3.plot(t, d["heading"], ".", ms=1.5, label=src)
    tc, course, _ = gps_course(d) if d["fix"].any() else (np.array([]), np.array([]), None)
    if len(tc):
        a3.plot(tc, course, ".", ms=4, color="tab:orange", alpha=0.8, label="GPS course over ground")
    a3.set_ylim(0, 360)
    a3.set_yticks([0, 90, 180, 270, 360])
    a3.set_ylabel("Heading (°)")
    a3.legend(loc="upper left", ncol=2, markerscale=3)
    if show_magcal:
        a3b = a3.twinx()
        a3b.step(t, d["mag"], color="gray", alpha=0.6, where="post")
        a3b.set_ylim(-0.2, 3.5)
        a3b.set_ylabel("Mag. calibration (0–3)", color="gray")
        a3b.grid(False)
    a3.set_title("(d) Heading", loc="left")

    # (e) GPS quality + safety stops
    a4 = fig.add_subplot(gs[3, 1], sharex=a1)
    a4.plot(t, np.where(d["fix"], d["hdop"], np.nan), lw=1.2, label="HDOP")
    a4.plot(t, d["sats"], lw=1.2, alpha=0.8, label="Satellites")
    stop = d["safety"] > 0
    if stop.any():
        a4.plot(t[stop], np.zeros(stop.sum()), "|", color="tab:red", ms=12, label="Safety stop")
    a4.set_xlabel("Time (s)")
    a4.set_ylabel("HDOP / count")
    a4.set_title("(e) GPS quality and safety stops", loc="left")
    a4.legend(loc="center right", ncol=3)

    marklines(a1, True)
    for a in (a2, a3, a4):
        marklines(a, False)
    for a in (a1, a2, a3):
        plt.setp(a.get_xticklabels(), visible=False)
    fig.suptitle(f"Field test {name} – sensor overview", fontsize=13, fontweight="bold")
    fig.savefig(path, dpi=150)
    if show:
        plt.show()
    plt.close(fig)


def plot_map(path, rec, d, show, max_cm=800, every=2):
    """Each LiDAR return placed at GPS position + heading. GPS jitter smears it - good enough to
    see walls and gaps along the route, not a precise map."""
    import matplotlib.pyplot as plt
    _style(plt)
    name = _log_name(path)
    ok = d["fix"] & d["imu"]
    fig, ax = plt.subplots(figsize=(10, 10), constrained_layout=True)
    if not ok.any():
        ax.text(0.5, 0.5, "Needs GPS fix and IMU data", ha="center", transform=ax.transAxes)
    else:
        npts = rec["lidar"].shape[1]
        ang = (np.arange(npts) + 0.5) * 360 / npts  # clockwise from the box front
        xs, ys = [], []
        for i in np.flatnonzero(ok)[::every]:
            cm = rec["lidar"][i].astype(float)
            sel = (cm > 0) & (cm < max_cm)
            b = np.radians(d["heading"][i] + ang[sel])
            r = cm[sel] / 100.0
            xs.append(d["east"][i] + r * np.sin(b))
            ys.append(d["north"][i] + r * np.cos(b))
        if xs:
            ax.scatter(np.concatenate(xs), np.concatenate(ys), s=1, c="0.35", alpha=0.25,
                       label=f"LiDAR returns (< {max_cm / 100:.0f} m)", rasterized=True)
        f = np.flatnonzero(d["fix"])
        k = np.ones(min(8, len(f))) / min(8, len(f))  # 2 s average: the raw track zig-zags
        ax.plot(np.convolve(d["east"][f], k, "valid"), np.convolve(d["north"][f], k, "valid"),
                "-", color="tab:blue", lw=2, label="GPS track (2 s mean)")
        for i, m, lab in _thin_marks_xy(d, 3.0):
            if lab:
                ax.annotate(lab, (d["east"][i], d["north"][i]), color="crimson", fontsize=8,
                            xytext=(5, 3), textcoords="offset points")
        ax.legend(loc="upper right", markerscale=6)
    ax.set_aspect("equal", adjustable="datalim")
    ax.set_xlabel("East (m)")
    ax.set_ylabel("North (m)")
    if d["heading_src"].startswith("IMU aligned"):
        sub = "LiDAR placed with GPS position and GPS-aligned IMU heading"
    else:
        sub = f"LiDAR placed with GPS position and {d['heading_src']} heading (not north-aligned)"
    ax.set_title(f"Field test {name} – LiDAR map\n", fontsize=13, fontweight="bold")
    ax.text(0.5, 1.01, sub, transform=ax.transAxes, ha="center", va="bottom", fontsize=9, color="0.35")
    fig.savefig(path, dpi=150)
    if show:
        plt.show()
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("log", help="log_NNN.bin downloaded from the box")
    ap.add_argument("--heading", choices=["gps", "imu"], default="gps",
                    help="gps (default): IMU heading aligned to the GPS course; imu: old -yaw + decl")
    ap.add_argument("--decl", type=float, default=4.0,
                    help="magnetic declination in deg for --heading imu (Deggendorf ~ +4)")
    ap.add_argument("--show-magcal", action="store_true",
                    help="also draw the BNO055 magnetometer calibration status (grey)")
    ap.add_argument("--no-show", action="store_true", help="only write the PNG files")
    args = ap.parse_args()

    rec, meta = load(args.log)
    if meta["records"] == 0:
        sys.exit("no records in this log")
    d = derive(rec, args.decl, args.heading)
    base = Path(args.log).expanduser().with_suffix("")
    summary(rec, meta, d)
    write_csv(f"{base}.csv", rec, d)
    plot_overview(f"{base}_overview.png", rec, d, not args.no_show, args.show_magcal)
    plot_map(f"{base}_map.png", rec, d, not args.no_show)
    print(f"written       {base}.csv, {base}_overview.png, {base}_map.png")


if __name__ == "__main__":
    main()