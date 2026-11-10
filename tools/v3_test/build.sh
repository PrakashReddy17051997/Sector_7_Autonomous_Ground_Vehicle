#!/bin/sh
# laptop tests of the v3 features (no ESP32 needed):  sh tools/v3_test/build.sh
#   ./align_test ../../cas_box_log/log_028.csv 8        start align: scan rotated to all 90 headings
#   ./lmk_replay ../../cas_box_log/log_027.csv 15 34    Open Sea table on a teaching log (Mark range)
#   ./lmk_replay ../../cas_box_log/log_029.csv leg 2    ... on a mission log (route leg)
#   ./tap_test                                          double-tap detector on synthetic signals
cd "$(dirname "$0")" || exit 1
S=../../cas_box_v3
CXX=${CXX:-c++}
$CXX -O2 -std=gnu++17 -I$S align_test.cpp $S/LocalLocalizer.cpp -o align_test &&
$CXX -O2 -std=gnu++17 -I$S lmk_replay.cpp $S/LandmarkLeg.cpp -o lmk_replay &&
$CXX -O2 -std=gnu++17 -Istub -I$S tap_test.cpp $S/ImuManager.cpp -o tap_test &&
echo "built align_test, lmk_replay, tap_test"
