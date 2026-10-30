#!/bin/sh
# builds the log replay (real firmware route code on the laptop):  sh tools/replay_fw/build.sh
# then:  tools/replay_fw/replay_fw cas_box_log/log_018.csv > log_018_replay.csv
#        python3 tools/replay_fw/plot_replay.py cas_box_log/log_018.csv log_018_replay.csv
# The route sources are COPIED next to the laptop stubs (stub/Sensors.h, WallFollow.h, Arduino.h) so
# that the box's own Sensors.h (FreeRTOS, serial ports, TinyGPS) is never picked up.
cd "$(dirname "$0")" || exit 1
B=../../cas_box
rm -rf build && mkdir build || exit 1
for f in RouteFollow.h RouteFollow.cpp RouteLocalizer.h RouteLocalizer.cpp LocalLocalizer.h LocalLocalizer.cpp \
         LocalPath.h RouteMap.h; do
  cp "$B/$f" build/ || exit 1
done
cp stub/*.h build/ || exit 1
CXX=${CXX:-c++}
$CXX -std=gnu++17 -O2 -Ibuild replay_fw.cpp build/RouteFollow.cpp build/RouteLocalizer.cpp build/LocalLocalizer.cpp \
  -o replay_fw && echo "built tools/replay_fw/replay_fw"
