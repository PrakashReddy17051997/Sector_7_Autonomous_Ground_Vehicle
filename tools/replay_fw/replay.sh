#!/bin/sh
# One command for a log replay (build if needed, replay, plot):
#   sh tools/replay_fw/replay.sh cas_box_log/log_020.csv
# -> cas_box_log/log_020_replay.csv and cas_box_log/log_020_replay.png
LOG="$1"
[ -f "$LOG" ] || { echo "usage: sh tools/replay_fw/replay.sh cas_box_log/log_NNN.csv  (make the CSV with tools/analyze_log.py first)"; exit 1; }
D="$(dirname "$0")"
sh "$D/build.sh" || exit 1
OUT="${LOG%.csv}_replay.csv"
"$D/replay_fw" "$LOG" > "$OUT" || exit 1
python3 "$D/plot_replay.py" "$LOG" "$OUT"
