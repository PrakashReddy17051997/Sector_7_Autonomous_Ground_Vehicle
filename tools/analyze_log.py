#!/usr/bin/env python3
"""Decode and plot a CAS Box drive log (log_NNN.bin from the box web page).

Usage:
    python3 analyze_log.py log_001.bin [--decl 4.0] [--no-show]

Writes next to the log file:
    log_001.csv           one row per record (4 per second), LiDAR as lidar_000..lidar_089 in cm
    log_001_overview.png  GPS track, tilt, side distances, heading / GPS quality - marks as lines
    log_001_map.png       LiDAR points placed with GPS position + compass heading (rough map)

Needs: numpy, matplotlib   (pip3 install numpy matplotlib)
"""
import argparse
import csv
import math
import struct
import sys
import warnings
from pathlib import Path

import numpy as np

HEADER = struct.Struct("<6sBBHHI")  # magic, version, lidarPoints, recordSize, periodMs, startMs
R_EARTH = 6371000.0
SAFETY = {0: "ok", 1: "tilt", 2: "front US", 3: "front-angled US", 4: "LiDAR front", 5: "back US"}


def record_dtype(lidar_points):
    return np.dtype([
        ("ms", "<u4"), ("latE7", "<i4"), ("lonE7", "<i4"), ("hdopX100", "<u2"),
        ("sats", "u1"), ("flags", "u1"),
        ("yawX100", "<i2"), ("pitchX100", "<i2"), ("rollX100", "<i2"),
        ("magCal", "u1"), ("mark", "u1"),
        ("cmdL", "i1"), ("cmdR", "i1"), ("outL", "i1"), ("outR", "i1"),
        ("safety", "u1"), ("rtt10", "u1"), ("batteryX100", "<u2"),
        ("us", "<u2", (5,)), ("lidar", "<u2", (lidar_points,)),
    ])


def load(path):
    raw = Path(path).read_bytes()
    if len(raw) < HEADER.size:
        sys.exit(f"{path}: file too short")
    magic, version, npts, rsize, period, start_ms = HEADER.unpack_from(raw)
    if magic != b"CASLOG" or version != 1:
        sys.exit(f"{path}: not a CAS log (magic {magic!r}, version {version})")
    dt = record_dtype(npts)
    if dt.itemsize != rsize:
        sys.exit(f"{path}: record size {rsize} in file, decoder expects {dt.itemsize}")
    body = raw[HEADER.size:]
    n = len(body) // rsize  # a record cut off by a power loss is dropped
    rec = np.frombuffer(body[: n * rsize], dtype=dt)
    meta = dict(lidar_points=npts, period_ms=period, start_ms=start_ms, records=n)
    return rec, meta


def derive(rec, decl_deg):
    """Physical units + local east/north metres around the first GPS fix."""
    t = (rec["ms"].astype(np.int64) - int(rec["ms"][0])) / 1000.0
    fix = (rec["flags"] & 1) > 0
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
    yaw = rec["yawX100"] / 100.0  # ISO: + = left, 0 = magnetic north
    heading = np.mod(-yaw + decl_deg, 360.0)  # compass: clockwise from true north
    return dict(
        t=t, fix=fix, lat=lat, lon=lon, lat0=lat0, lon0=lon0, east=east, north=north,
        hdop=rec["hdopX100"] / 100.0, sats=rec["sats"].astype(int),
        imu=(rec["flags"] & 2) > 0, lidar_ok=(rec["flags"] & 4) > 0,
        yaw=yaw, heading=heading,
        pitch=rec["pitchX100"] / 100.0, roll=rec["rollX100"] / 100.0,
        mag=np.where(rec["magCal"] & 0x80, 0, rec["magCal"]).astype(int), mark=rec["mark"].astype(int),
        route_leg=np.where(rec["magCal"] & 0x80, rec["magCal"] & 15, 0).astype(int),
        route_state=np.where(rec["magCal"] & 0x80, (rec["magCal"] >> 4) & 7, 0).astype(int),
        cmdL=rec["cmdL"] / 200.0, cmdR=rec["cmdR"] / 200.0,
        outL=rec["outL"] / 200.0, outR=rec["outR"] / 200.0,
        safety=rec["safety"].astype(int), battery=rec["batteryX100"] / 100.0,
        rtt=rec["rtt10"].astype(int) * 10,
    )


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


def write_csv(path, rec, d):
    npts = rec["lidar"].shape[1]
    cols = ["t_s", "lat", "lon", "east_m", "north_m", "gps_fix", "sats", "hdop",
            "yaw_iso_deg", "heading_true_deg", "pitch_deg", "roll_deg", "mag_cal", "mark",
            "cmd_l", "cmd_r", "out_l", "out_r", "safety", "battery_v", "rover_rtt_ms",
            "route_leg", "route_state",
            "us_front", "us_left", "us_right", "us_back", "us_front_angled"]
    cols += [f"lidar_{i * 360 // npts:03d}" for i in range(npts)]
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
                   d["route_leg"][i], d["route_state"][i]]
            row += [int(v) for v in rec["us"][i]] + [int(v) for v in rec["lidar"][i]]
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
    if d["imu"].any():
        print(f"tilt          max |pitch| {np.abs(d['pitch']).max():.1f} deg, "
              f"max |roll| {np.abs(d['roll']).max():.1f} deg, magnetometer cal min {d['mag'][d['imu']].min()}")
    stops = {SAFETY.get(k, k): int((d["safety"] == k).sum()) for k in np.unique(d["safety"]) if k}
    print(f"safety stops  {stops if stops else 'none'}  (records)")
    bat = d["battery"][d["battery"] > 0]
    if len(bat):
        print(f"battery       {bat.max():.2f} -> {bat.min():.2f} V")
    rt = d["rtt"][d["rtt"] > 0]
    if len(rt):
        print(f"rover delay   median {np.median(rt):.0f} ms, 90% {np.percentile(rt, 90):.0f} ms, max {rt.max()} ms (box -> rover command)")
    for i, m in mark_events(d):
        pos = (f"E {d['east'][i]:+.1f} m, N {d['north'][i]:+.1f} m"
               if not math.isnan(d["east"][i]) else "no GPS fix")
        print(f"mark {m:<3}     t = {d['t'][i]:6.1f} s   {pos}, heading {d['heading'][i]:.0f} deg, "
              f"pitch {d['pitch'][i]:+.1f}")


def plot_overview(path, rec, d, show):
    import matplotlib.pyplot as plt
    t = d["t"]
    marks = mark_events(d)
    fig = plt.figure(figsize=(14, 10))
    gs = fig.add_gridspec(4, 2, width_ratios=[1, 1.4])

    ax = fig.add_subplot(gs[:, 0])
    if d["fix"].any():
        sc = ax.scatter(d["east"], d["north"], c=t, s=6, cmap="viridis")
        fig.colorbar(sc, ax=ax, label="time (s)")
        f = np.flatnonzero(d["fix"])
        ax.plot(d["east"][f[0]], d["north"][f[0]], "go", ms=10, label="start")
        ax.plot(d["east"][f[-1]], d["north"][f[-1]], "rs", ms=10, label="end")
        for i, m in marks:
            if not math.isnan(d["east"][i]):
                ax.annotate(f"M{m}", (d["east"][i], d["north"][i]), color="crimson",
                            fontweight="bold", xytext=(4, 4), textcoords="offset points")
        ax.legend(loc="best")
    else:
        ax.text(0.5, 0.5, "no GPS fix in this log", ha="center", transform=ax.transAxes)
    ax.set_aspect("equal", adjustable="datalim")
    ax.set_xlabel("east (m)")
    ax.set_ylabel("north (m)")
    ax.set_title("GPS track (colour = time)")
    ax.grid(alpha=0.3)

    def marklines(a):
        for i, m in marks:
            a.axvline(t[i], color="crimson", lw=0.8, alpha=0.6)
            a.text(t[i], 1.0, f"M{m}", color="crimson", transform=a.get_xaxis_transform(),
                   ha="center", va="bottom", fontsize=8)

    a1 = fig.add_subplot(gs[0, 1])
    a1.plot(t, d["pitch"], label="pitch (+ = front down)")
    a1.plot(t, d["roll"], label="roll (+ = right down)")
    a1.axhspan(-12, 12, color="green", alpha=0.06)
    a1.set_ylabel("deg")
    a1.set_title("Tilt - ramps show up as pitch steps", loc="left")
    a1.legend(fontsize=8, loc="upper right")

    a2 = fig.add_subplot(gs[1, 1], sharex=a1)
    lid = rec["lidar"]
    a2.plot(t, sector_min(lid, 270), label="LiDAR left (270 +-8)")
    a2.plot(t, sector_min(lid, 90), label="LiDAR right (90 +-8)")
    a2.plot(t, sector_min(lid, 0, 20), label="LiDAR front (+-20)", alpha=0.6)
    a2.set_ylim(0, 500)
    a2.set_ylabel("cm")
    a2.set_title("Side distances - wall following / facade end", loc="left")
    a2.legend(fontsize=8, loc="upper right")

    a3 = fig.add_subplot(gs[2, 1], sharex=a1)
    a3.plot(t, d["heading"], ".", ms=2, label="compass heading (true)")
    a3.set_ylim(0, 360)
    a3.set_yticks([0, 90, 180, 270, 360])
    a3.set_ylabel("deg")
    a3b = a3.twinx()
    a3b.step(t, d["mag"], color="gray", alpha=0.6, where="post")
    a3b.set_ylim(-0.2, 3.5)
    a3b.set_ylabel("mag cal", color="gray")
    a3.set_title("Heading (dots) and magnetometer calibration (grey)", loc="left")

    a4 = fig.add_subplot(gs[3, 1], sharex=a1)
    a4.plot(t, np.where(d["fix"], d["hdop"], np.nan), label="HDOP")
    a4.plot(t, d["sats"], label="satellites", alpha=0.7)
    stop = d["safety"] > 0
    if stop.any():
        a4.plot(t[stop], np.zeros(stop.sum()), "r|", ms=12, label="safety stop")
    a4.set_xlabel("time (s)")
    a4.set_title("GPS quality", loc="left")
    a4.legend(fontsize=8, loc="upper right")

    for a in (a1, a2, a3, a4):
        a.grid(alpha=0.3)
        marklines(a)
    fig.suptitle(Path(path).name.replace("_overview.png", ".bin"))
    fig.tight_layout()
    fig.savefig(path, dpi=130)
    if show:
        plt.show()
    plt.close(fig)


def plot_map(path, rec, d, show, max_cm=800, every=2):
    """Each LiDAR return placed at GPS position + heading. GPS jitter smears it - good enough to
    see walls and gaps along the route, not a precise map."""
    import matplotlib.pyplot as plt
    ok = d["fix"] & d["imu"]
    fig, ax = plt.subplots(figsize=(10, 10))
    if not ok.any():
        ax.text(0.5, 0.5, "needs GPS fix + IMU", ha="center", transform=ax.transAxes)
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
                       label="LiDAR")
        f = np.flatnonzero(d["fix"])
        k = np.ones(min(8, len(f))) / min(8, len(f))  # 2 s average: the raw track zig-zags
        ax.plot(np.convolve(d["east"][f], k, "valid"), np.convolve(d["north"][f], k, "valid"),
                "-", color="tab:blue", lw=2, label="GPS track (2 s average)")
        for i, m in mark_events(d):
            if not math.isnan(d["east"][i]):
                ax.annotate(f"M{m}", (d["east"][i], d["north"][i]), color="crimson",
                            fontweight="bold")
        ax.legend(loc="best")
    ax.set_aspect("equal", adjustable="datalim")
    ax.set_xlabel("east (m)")
    ax.set_ylabel("north (m)")
    ax.set_title("LiDAR map from GPS + compass (rough)")
    ax.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(path, dpi=130)
    if show:
        plt.show()
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("log", help="log_NNN.bin downloaded from the box")
    ap.add_argument("--decl", type=float, default=4.0,
                    help="magnetic declination in deg, + = east (Deggendorf ~ +4)")
    ap.add_argument("--no-show", action="store_true", help="only write the PNG files")
    args = ap.parse_args()

    rec, meta = load(args.log)
    if meta["records"] == 0:
        sys.exit("no records in this log")
    d = derive(rec, args.decl)
    base = Path(args.log).with_suffix("")
    summary(rec, meta, d)
    write_csv(f"{base}.csv", rec, d)
    plot_overview(f"{base}_overview.png", rec, d, not args.no_show)
    plot_map(f"{base}_map.png", rec, d, not args.no_show)
    print(f"written       {base}.csv, {base}_overview.png, {base}_map.png")


if __name__ == "__main__":
    main()
