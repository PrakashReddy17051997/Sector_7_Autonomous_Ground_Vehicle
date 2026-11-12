# rover_sim - test the box code on the laptop (v3, 12.11.2026)

The **same code as on the box** (RouteFollow, LocalLocalizer start-align + scan matching, LandmarkLeg, WallFollow
wall fit, MotorShaper 2 s ramps and the +-0.5 limit) drives a simulated rover. Only the hardware is simulated.

## Build and run (Mac / Linux)

    sh tools/sim_v3/build.sh                 # rebuild after EVERY change in cas_box_v3 (it copies the current code)
    cd tools/sim_v3
    ./rover_sim --help
    ./rover_sim --model rover7 --start-off 180 --legs 1 --time 70 --csv out/start180
    python3 plot_sim.py out/start180         # -> out/start180.png  (needs matplotlib)
    open out/start180.png

## Typical tests

| what | command |
|---|---|
| start turned 90 / 180 deg | `./rover_sim --model rover7 --start-off 180 --legs 1 --csv out/s180` |
| same with a rover that can pivot | `--model free` |
| box on the path, leg 1, 8 m along | `./rover_sim --model rover7 --obst 1:8:0:0.25 --legs 1 --csv out/box` |
| foot for 12 s (t 20-32 s) | `--obst 1:8:0:0.15:20:32` |
| object 0.4 m right of the path | `--obst 1:8:-0.4:0.2` |
| several objects | repeat `--obst` |
| ramp section M45-M61 | `./rover_sim --model rover7 --start-leg 6 --legs 4 --csv out/ramp` |
| box 7 LiDAR error vs the maps | `--lidar-err 8` |
| try a tuning value | `--set pivotDeg=40 --set rollStart=0 --shaper accelS=1` |

Plot (plot_sim.py, one PNG): left = map with the rover footprint every 2 s (light = early, dark = late) and the
track coloured by state; right = heading error, wheel commands asked by RouteFollow (dashed) and sent to the
rover after the MotorShaper (solid), state band. The firmware messages are in out/NAME_events.txt.
(`--out FILE.html` still writes the old interactive viewer.)

Old viewer: Play / speed / timeline, "follow rover" (zooms in), LiDAR points (green), track coloured by state
(orange TURN, blue DRIVE, red AVOID, purple BACKUP, grey PAUSED), wheel commands **asked** by RouteFollow vs
**after the MotorShaper** (what the rover gets), and every firmware message - click one to jump there.
`[sim] BUMP` = the rover body touches a wall / obstacle.

## Rover models (`--model`)

* `rover7` (default) - measured on rover 7, logs 030-034 (7900 samples):
  wheels in opposite directions (turn on the spot): **no rotation** (median 0, 10 % of 3540 samples turned);
  both wheels forward (curve): 16 deg/s per unit of w = (r - l)/2 (e.g. l 0 / r 0.5 -> radius ~2.6 m);
  both wheels backward: 30 deg/s per unit w (only 16 samples - check on the rover).
* `free` - turns on the spot (40 deg/s at 0.5), like Kethan's rover / a smooth floor.
* `weak` - in between.
* forward rolling moves 9 deg right of the nose (`--crab`, like the teaching run).

## World and limits

* Walls = the occupied 10 cm cells of the LocalPath.h maps (teaching run log_027). Cells within 0.5 m of the taught
  path are removed (people next to the rover in the teaching run, the ramp surface the tilted LiDAR hit) -
  `--clear` changes that. Glass that the LiDAR saw through is missing. Steps / kerbs are not in the map.
* Pitch comes from the teaching run (sim_ramps.h, `make_sim_ramps.py`), roll 0; the rover does not slow on a slope.
* SafetyGuard simplified: no forward drive with LiDAR < 30 cm in +-20 deg or front ultrasonic < 25 cm
  (on a ramp 0 / 10 cm), like cas_box_v3.ino.
* No GPS, no Wi-Fi, no page heartbeat; Pause ends the run (`--resume S` presses Resume after S s).
