"""Plot a firmware log replay: what the route code believed at every moment of a real drive.

    tools/replay_fw/build.sh                                         # once
    tools/replay_fw/replay_fw cas_box_log/log_017.csv > log_017_replay.csv
    python3 tools/replay_fw/plot_replay.py cas_box_log/log_017.csv log_017_replay.csv

Panels (time axis = the log):
  1  leg and route state (log vs replay)
  2  LiDAR: match score (green dots = matched > 0.35) and number of LiDAR points 0.3-10 m
  3  heading: error to the look-ahead point (carrot), the error the controller acts on, and the real
     yaw rate of the rover; orange bars = the box sent a turn tap (log), purple = the replay would
  4  distance from the taught path (m), and pitch (deg, right axis) - ramps
Prints one line per leg: located share, median score, off path, and how often the replay command
agrees with what the box really sent.
For v3 logs (DriveLog version 3) the same CSV also has the box's OWN beliefs (lp_x, carrot_err,
dbg_tap ...): this script then overlays the box's carrot error (black) in panel 3.
"""
import sys
from pathlib import Path

import numpy as np
import pandas as pd
import matplotlib

if "--show" not in sys.argv:
    matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402


def main():
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    if len(args) == 1:   # only the log given: use log_NNN_replay.csv next to it
        args.append(str(Path(args[0]).with_name(Path(args[0]).stem + "_replay.csv")))
    if len(args) < 2:
        sys.exit(__doc__)
    for a in args[:2]:
        if not Path(a).exists():
            sys.exit(f"file not found: {a}  (current folder: {Path.cwd()})\n"
                     "make the log CSV with: python3 tools/analyze_log.py cas_box_log/log_NNN.bin")
    log, rep = pd.read_csv(args[0]), pd.read_csv(args[1])
    if rep.empty:
        sys.exit(f"{args[1]} is empty - did replay_fw run? (sh tools/replay_fw/build.sh first)")
    t = log.t_s.values - log.t_s.values[0]
    L = log[[c for c in log.columns if c.startswith("lidar_")]].values
    npts = ((L > 30) & (L < 1000)).sum(1)
    yaw = np.unwrap(np.radians(log.yaw_iso_deg.values)) * 57.29578
    rate = np.r_[0, np.diff(yaw) / np.maximum(np.diff(t), 0.05)]
    piv_log = (log.out_l * log.out_r < 0).values
    piv_rep = (rep.cmd_l * rep.cmd_r < 0).values

    fig, ax = plt.subplots(4, 1, figsize=(15, 11), sharex=True)
    ax[0].step(t, log.route_leg, where="post", label="log leg", color="0.6")
    ax[0].step(rep.t, rep.leg, where="post", label="replay leg", color="tab:blue")
    ax[0].step(rep.t, rep.st * 0.2, where="post", label="replay state / 5 (1 turn 2 drive 3 avoid 5 pause)", color="tab:red", lw=0.8)
    ax[0].set_ylabel("leg"); ax[0].legend(loc="upper left", fontsize=8)

    ax[1].plot(rep.t, rep.score, ".", ms=3, color="0.5", label="match score")
    m = rep.score > 0.35
    ax[1].plot(rep.t[m], rep.score[m], ".", ms=3, color="tab:green", label="matched")
    a2 = ax[1].twinx(); a2.plot(t, npts, color="tab:orange", lw=0.7, label="LiDAR points 0.3-10 m"); a2.set_ylabel("points")
    ax[1].set_ylabel("score"); ax[1].set_ylim(-0.05, 1.05); ax[1].legend(loc="upper left", fontsize=8)

    ax[2].plot(rep.t, rep.carrot_err, color="tab:green", lw=1, label="replay: error to look-ahead point")
    ax[2].plot(rep.t, rep.ctrl_err, color="tab:blue", lw=0.8, label="replay: controller error")
    if "carrot_err" in log.columns:
        ax[2].plot(t, log.carrot_err, color="k", lw=0.8, label="BOX (v3 log): error to look-ahead point")
    ax[2].plot(t, rate / 5, color="0.6", lw=0.7, label="real yaw rate / 5 (deg/s)")
    for tt in t[piv_log]:
        ax[2].axvspan(tt - 0.12, tt + 0.12, color="orange", alpha=0.25, lw=0)
    for tt in rep.t[piv_rep]:
        ax[2].axvline(tt, color="purple", alpha=0.3, lw=0.6)
    ax[2].axhline(8, color="k", lw=0.4, ls=":"); ax[2].axhline(-8, color="k", lw=0.4, ls=":")
    ax[2].set_ylim(-90, 90); ax[2].set_ylabel("deg (+ = left)"); ax[2].legend(loc="upper left", fontsize=8)

    ax[3].plot(rep.t, rep.off, color="tab:red", label="replay: off the taught path (m)")
    ax[3].set_ylabel("m"); ax[3].set_ylim(0, 2)
    a3 = ax[3].twinx(); a3.plot(t, log.pitch_deg, color="tab:purple", lw=0.7, label="pitch (deg, + front down)"); a3.set_ylabel("pitch deg")
    ax[3].legend(loc="upper left", fontsize=8); a3.legend(loc="upper right", fontsize=8)
    ax[3].set_xlabel("t (s)")
    fig.suptitle(f"firmware replay of {Path(args[0]).name}")
    fig.tight_layout()
    out = Path(args[1]).with_suffix(".png")
    fig.savefig(out, dpi=90)
    print(f"-> {out}")

    rep["agree"] = (np.sign(rep.cmd_l) == np.sign(rep.log_cmd_l)) & (np.sign(rep.cmd_r) == np.sign(rep.log_cmd_r))
    print(f"{'leg':>4} {'records':>7} {'matched':>8} {'score':>6} {'off m':>6} {'same cmd':>9}")
    for leg, g in rep.groupby("log_leg"):
        print(f"{leg:>4} {len(g):>7} {100 * (g.score > 0.35).mean():>7.0f}% {g.score.median():>6.2f} "
              f"{g.off.median():>6.2f} {100 * g.agree.mean():>8.0f}%")
    if "--show" in sys.argv:
        plt.show()


if __name__ == "__main__":
    main()
