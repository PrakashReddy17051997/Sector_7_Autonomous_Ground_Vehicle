#!/bin/sh
# Builds tools/sim_v3/rover_sim from the CURRENT firmware sources in cas_box_v3 (no Arduino needed).
#   sh tools/sim_v3/build.sh        (Mac: needs the Xcode command line tools -> c++)
# Re-run after every change in cas_box_v3: the sim always uses the code you are about to flash.
set -e
cd "$(dirname "$0")"
S=../../cas_box_v3
B=build
rm -rf $B && mkdir -p $B
for f in RouteFollow.cpp RouteFollow.h LandmarkLeg.cpp LandmarkLeg.h LandmarkRoute.h LineFeat.h LocalLocalizer.cpp \
         LocalLocalizer.h LocalPath.h RouteLocalizer.cpp RouteLocalizer.h RouteMap.h WallFollow.h MotorShaper.h; do
  cp $S/$f $B/
done
cp stub/Arduino.h stub/Sensors.h stub/ESPAsyncWebServer.h $B/
# WallFollow.h: drop the web page parts (String), keep the class
sed -i.bak '/^[[:space:]]*String /d' $B/WallFollow.h && rm -f $B/WallFollow.h.bak
# WallFollow::measure + fitLine out of WallFollow.cpp (RouteFollow uses only these; fitLine .. "Starts following")
{ echo '#include "WallFollow.h"'
  awk '/^static bool fitLine/{on=1} /^\/\/ Starts following/{on=0} on' $S/WallFollow.cpp
} > $B/wf_measure.cpp
CXX=${CXX:-c++}
$CXX -O2 -std=gnu++17 -I$B -o rover_sim rover_sim.cpp $B/RouteFollow.cpp $B/LandmarkLeg.cpp $B/LocalLocalizer.cpp \
     $B/RouteLocalizer.cpp $B/wf_measure.cpp
echo "built tools/sim_v3/rover_sim   (./rover_sim --help)"
