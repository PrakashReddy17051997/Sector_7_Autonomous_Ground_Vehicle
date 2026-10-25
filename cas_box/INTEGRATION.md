# Drive logger + safety guard: how to add them to cas_box

New files: `DriveLog.h`, `DriveLog.cpp`, `SafetyGuard.h`, `tools/analyze_log.py`.
Changed files: `Sensors.h` and `WebHMI.h` (only additions, nothing removed), plus one line in
`WebHMI.cpp` and a few lines in `cas_box.ino`.

Compile-checked against the ESP32 core 3.3.0, ESP Async WebServer 3.9.2 and Async TCP 3.4.9,
with no warnings. Library versions are unchanged, and LittleFS is part of the board package.

---

## 1. Copy the files

Into the `cas_box` sketch folder:

| File | Action |
|---|---|
| `DriveLog.h`, `DriveLog.cpp`, `SafetyGuard.h` | add (new) |
| `Sensors.h` | replace. Adds read-only getters: `lidarCm(i)`, `yawDeg()`, `pitchDeg()`, `rollDeg()`, `lat()`, `lon()`, `hdop()`, `sats()`, `gpsFix()`, `imuOk()`, `lidarOk()`, `magCal()` |
| `WebHMI.h` | replace. Adds `AsyncWebServer* server() const` |

`tools/analyze_log.py` stays on the Mac and does not go into the sketch folder.

## 2. One line in `WebHMI.cpp`

Near the end of the `kPage` string, find:

```
setInterval(poll,250);poll();
</script></body></html>)rawpage";
```

and change the second line to:

```
</script><script src='/log.js'></script></body></html>)rawpage";
```

This loads the "Drive log" card (Start / Stop / Mark / Delete all / download) below the drive pad.

## 3. `cas_box.ino`

**Easiest:** use the `cas_box.ino` from this zip. It's the starter file with everything below
already added. Check `BOX_ID` (it's set to 10, as in your copy) before flashing.

If you want to merge the changes by hand instead, these are the additions:

**Top of the file:**

```cpp
#include "DriveLog.h"
#include "SafetyGuard.h"

static DriveLog logger;
static SafetyGuard guard;
static const bool kUseGuard = true;   // false = guard off (e.g. if "front angled" looks at the floor)
```

**`setup()`**, after `hmi.begin(...)`:

```cpp
  logger.begin(hmi.server());   // first boot: formats the flash file system (a few seconds, once)
```

**`loop()`**, section "driving". Keep your existing code that reads the drive pad into `l` / `r`,
then replace the plain `rover.setDrive(l, r);` with:

```cpp
  const float cmdL = l, cmdR = r;                         // what the pad asked for
  const uint8_t why = kUseGuard ? guard.apply(sensors, l, r) : 0;
  rover.setDrive(l, r);                                   // still every loop pass

  DriveLog::Sample smp;
  smp.cmdL = cmdL;  smp.cmdR = cmdR;
  smp.outL = l;     smp.outR = r;
  smp.safety = why;
  smp.batteryV = rover.voltage();
  logger.update(sensors, smp);                            // after sensors.update()

  hmi.showValue("safety", SafetyGuard::reasonText(why));  // "Dev" view
  hmi.showValue("log", logger.statusText());
```

Optional: put the guard reason into the state line too, so you can see it without opening the Dev view.

## 4. If the upload says "Sketch too big" (since 2026-10-24: use No OTA, see section 11)

Logging needs the **Default** partition scheme: 1.25 MB app plus about 1.3 MB for logs, which is about 25 minutes of driving.
The test build used 82 % of the app space with only the web server, so your full sketch should fit.
If it doesn't, choose **"Minimal SPIFFS (1.9MB APP with OTA / 190KB SPIFFS)"**. That leaves room for only about 3 minutes of
log, so delete files between runs. Don't use "Huge APP", because it has no file-system partition.

## 5. Tomorrow: test order

1. **Bench test (rover on a box, wheels free):**
   - Upload the sketch. The serial monitor should show `[log] LittleFS ... kB used`.
   - Open the page. The "Drive log" card should appear.
   - Press Start, wait 10 s, Mark, wait, then Stop. A file `log_001.bin` (about 9 kB) should be listed.
   - Download it and run the analyser on it (step 7).
2. **Safety guard (still on the bench):**
   - Hold forward and put a hand in front of the front ultrasonic. The wheels should stop, and the Dev view
     should show `STOP: front ultrasonic`. Turning left or right must still work.
   - Hand behind the rover while reversing should give `STOP: back ultrasonic`.
   - Tilt the box by hand more than 15 deg. The result should be `STOP: tilt`, with only slow reversing allowed.
   - **Check the front-angled sensor.** On flat ground with nothing ahead, look at its value. If it
     reads under 25 cm because it points at the floor, set `cfg.useFrontAngled = false;`. Then
     tell me its flat-ground value, because it may be useful as a cliff sensor.
3. **Outdoor manual drive B → H with logging:**
   - Wait for a GPS fix first. HDOP should be below 3, and the magnetometer calibration (last of the 4 cal numbers) should reach 3.
     If it doesn't, move the box in a slow figure eight.
   - Start the log, then drive the route at a steady, slow speed.
   - Press **Mark** at each landmark, and write in your notebook what each mark was.
     For example: M1 = leaving start, M2 = end of B facade, M3 = top of ramp 1, M4 = bottom of
     ramp 1, M5 = entering the D/E gap, M6 = leaving it, M7 = at H.
   - Drive the route **2–3 times**, one file per run, and stay on the same line each time.

## 6. Getting the files to the Mac

The easiest way is to join the `CAS-Box-NN` Wi-Fi from the Mac, open the box page, and click
"download" next to each file. A phone works too, but then you have to move the files on (AirDrop, mail).
You can't download the file that is currently recording; press Stop first.

## 7. Analyse

```bash
pip3 install numpy matplotlib
python3 analyze_log.py ~/Downloads/log_001.bin            # add --decl 4.0 (default) if needed
```

It prints a summary (duration, GPS fix %, HDOP, max tilt, safety stops, and every mark with its
position and heading) and writes three files next to the log:

- `log_001.csv`: all values. LiDAR columns are `lidar_000` … `lidar_356` (degrees clockwise from front, in cm).
- `log_001_overview.png`: GPS track with marks, pitch/roll (ramps), left/right/front LiDAR distance
  (wall following, facade end), heading plus magnetometer calibration, and GPS quality.
- `log_001_map.png`: LiDAR points placed using GPS and compass. It's a rough map, but you can see walls and gaps.

What to look for (input for the leg table):

| Question | Where to look |
|---|---|
| Distance to the B facade on the raised path | Overview: "LiDAR left" (or right) before the mark at the facade end |
| How the facade end looks | The same curve jumps from about 80 cm to far or none |
| Ramp angle and length | Pitch step between the two ramp marks |
| How far GPS wanders near the buildings | Compare the tracks of the 2–3 runs; HDOP over time |
| Is the compass trustworthy near steel and concrete? | Heading on a straight leg should stay flat |

## Log file format (for your own scripts)

16-byte header (`CASLOG`, version 1, 90 LiDAR points, record size 222, period 250 ms, start ms),
then 222-byte records, little endian. See `LogRecord` in `DriveLog.h`. The analyser decodes it.

---

## 8. Autonomous modes (GpsNav, WallFollow), added 2026-10-22

Files: `GpsNav.h/.cpp`, `WallFollow.h/.cpp` (already wired into `cas_box.ino`). The page gets two
more cards below the drive pad: "Wall following (LiDAR)" and "Autonomous (GPS)".

Common rule for both: **the rover only drives while you HOLD ▲** (dead-man switch). Releasing it
stops the rover. ◀ ▶ ▼ are manual override, and the slider sets the speed (use 0.15–0.25).
SafetyGuard and the acceleration ramp still act afterwards. Only one mode can be active at a time.

**Wall following:** place the rover parallel to a wall, at the distance you want to keep
(0.35–2.0 m). Press "Follow LEFT wall" or "Follow RIGHT wall", then hold ▲.
- `follow`: keeps the distance (±10 cm buttons adjust it)
- `bridge`: short gap in the wall, keeps going straight for up to 2 s
- `avoid`: obstacle ahead, drives around it, only on the wall side by default
- `lost (wall ended)`: stops at the end of the wall

The LiDAR can't see the edge of the raised path. Test on flat ground first, next to a wall with open space on the other side.

**GPS waypoints:** "Clear WPs", then "Add WP here" at each point, or use "Load route B→H".
Press "Auto ON", give it about 3 m of free space ahead, and hold ▲.
- `align`: drives straight about 2 m to learn its heading from GPS
- `follow`: drives to each waypoint in turn
- `done`: last waypoint reached

Simulations (before changing parameters): `python3 tools/sim_wallfollow.py 0 out.png` and
`python3 tools/sim_gpsnav.py 1 out.png`.

---

## 9. Full-auto mission + fixes from log_004, added 2026-10-22

New: `Mission.h/.cpp`. Changed: `ImuManager.cpp` (starter file!), `GpsNav`, `WallFollow`,
`DriveLog` (logs the rover delay), `cas_box.ino`, `tools/analyze_log.py`.

**Mission card (top of the page):**
- "Run: wall B → GPS to H":
  1. Follows the LEFT wall at its current distance.
  2. Learns the GPS heading on the way.
  3. At the facade end, switches to the GPS route from the nearest waypoint to H.
- "Run: GPS route only": GPS waypoints only (aligns first by driving about 2 m straight).
- No key needs to be held. The page sends a heartbeat. If the page is closed, hidden or out of
  Wi-Fi range, the rover pauses, and it continues when the heartbeat is back.
- The big STOP button, or any drive key, ends the mission.
- Speed comes from the drive slider (limited to 0.10–0.30).

**ImuManager.cpp:** `kUseMagnetometer = false` switches the BNO055 to IMUPLUS mode (gyro and
accelerometer only). In log_004 the NDOF heading stood still while the rover turned 55°. Yaw now
starts at 0 at power-on, so it is relative, not north. Set the constant to true for the old behaviour.

**Delay:**
- The box now wakes the rover's Wi-Fi at every beacon (DTIM 1 instead of 2).
- The log records the rover command round trip. The analyser prints "rover delay median / 90% / max".

---

## 10. After the field tests of 2026-10-23 (logs 5–8)

Copy ALL files again: `cas_box.ino`, `*.h`, `*.cpp` (including `ImuManager.cpp`), new
`MotorShaper.h` and `PageUi.h`. Use the new `tools/analyze_log.py` (it shows the rover delay).

- **Web page:** manual drive, Mission (full auto) and Drive log only. The hold-to-drive wall and
  GPS cards are gone. One "Sensors" table: ultrasonic L | R | F | B | FA, then IMU, then GPS.
- **MotorShaper** (replaces the ramp): dead band 0.18, pivots at least 0.5, breakaway kick 0.5 for
  0.4 s from standstill. In the logs, the rover did not move or turn at small commands (checked with the LiDAR).
- **GPS steering in arcs** (`arcOnly`): the inner wheel slows to 0 but never reverses, so there are no full-power pivots.
- **Heading seed:** when the mission starts within 10 m of the route start, the heading is set from the
  B facade direction (`facadeBearingDeg = 118`, from log_003; please verify) plus the measured
  wall angle. The switch wall → GPS is then automatic.
- **Mission heartbeat timeout: 0.8 s** (was 0.4 s). Page traffic is reduced (heartbeat every 250 ms,
  status every 1 s, two cards fewer).
- **Box restarts:** `/boot.txt` (open `http://192.168.17.1/boot.txt`) lists why the box started.
  BROWNOUT means the supply voltage dropped. For 2 minutes after an unexpected restart, the state line shows it.
- Full Wi-Fi transmit power on the box.

---

## 11. Route B → H by teach & repeat (RouteFollow), added 2026-10-24

**What drives now:**
`cas_box.ino` → `Mission` (plan "route") → **`RouteFollow`** → SafetyGuard → MotorShaper → RoverLink.
The old wall → GPS plan is only reachable as the "GPS route only" button. It is GPS-primary and failed in log_010.

**The idea:** log_009 is the teaching run. Its 32 Mark presses cut it into **10 legs**, built by `tools/make_legs.py`:

| # | leg | tracking | ends when |
|---|---|---|---|
| 1 | B facade | left wall 1.45 m | left wall ends (facade end) |
| 2 | after facade | heading | taught effort |
| — | turn right ~87° | | |
| 3 | to ramp D | heading | wall ahead < 2.5 m |
| — | turn right ~93° | | |
| 4 | wall at ramp D | left wall 1.33 m | left wall ends (corner of D) |
| — | turn left ~91° | | |
| 5 | around corner D | left wall 1.39 m | left wall ends |
| 6 | open stretch | heading | object on the right < 0.8 m |
| 7 | narrow passage | right wall 0.49 m | right wall ends |
| 8 | diagonal | heading (GPS may pull ±15°) | taught effort / GPS end zone 2 m |
| 9 | wall to H | left wall 1.9 m | taught effort |
| 10 | last stretch H | left wall 2.65 m | taught effort → DONE |

- **Heading:** gyro, anchored at the start on the B facade and re-anchored on every wall leg by the
  measured wall direction. In log_009 the wall directions stay within 1–3° over 10 min.
- **Progress:** effort = Σ forward wheel command × time, the same quantity as in log_009. A landmark only
  counts after `gate` × the taught effort. Much more effort than taught with no landmark → PAUSE.
- **GPS:** zone check only. The status shows "(GPS NN m off)" at a leg end if more than 15 m from
  the taught end point. GPS ends an effort leg early inside 2 m of the taught end point.
- **Obstacles:**
  1. Wait 3 s.
  2. Sidestep to the closest free side; never to the right on legs 1–2 (raised path).
  3. Turn back parallel and carry on.
  4. If no way past after 3 tries → PAUSE.
- **Stuck:** the LiDAR scan doesn't change while the wheels are driven → back up 1.5 s. After 3 times → PAUSE.

Checks before flashing:
- `make_legs.py --check`: replaying the end detectors on log_009, every leg ends 0–6 s from the taught end.
- `tools/sim_route.py`: closed loop, legs 1–5 complete, with a box on the facade passed at 0.36 m clearance.

### Flash: partition scheme "No OTA" (needed now)

The app is now 1.31 MB, more than the 1.25 MB app slot of the default scheme. **No OTA (2MB APP/2MB SPIFFS)**
gives 2 MB for the app and 1.9 MB for logs (more than before, about 35 min).

```
arduino-cli compile --fqbn esp32:esp32:esp32:UploadSpeed=115200,PartitionScheme=no_ota cas_box
arduino-cli upload  --fqbn esp32:esp32:esp32:UploadSpeed=115200,PartitionScheme=no_ota -p /dev/cu.usbserial-XXXX cas_box
```

Arduino IDE: Tools → Partition Scheme → "No OTA (2MB APP/2MB SPIFFS)".

**Download any logs still on the box first.** The log storage moves and is formatted at the first boot (a few seconds).

### Using it in the field

1. Box on, then the rover. Phone on CAS-Box-07. Set speed to **0.20**, the speed of the teaching run.
2. Place the rover at the log_009 start: on the raised path, parallel to the B facade, wall on the
   LEFT at about 1.5 m.
3. Drive log **Start**, then **Run route B → H (log_009)**. Keep the page open and in front.
4. Status line: `leg N/10 drive|turn|avoid|back up | effort NN %`.
5. **If it pauses** ("end not found", "blocked", "stuck") or you press a drive key / STOP:
   - **Resume** continues the same leg. The heading is kept, so the gyro must keep running: do not switch the box off.
   - You may drive it by hand to the right spot first, or pick it up and set it down.
   - **Start at this leg** (selector) re-anchors: place the rover parallel to that leg (to its wall).
6. Press **Mark** in the drive log when something unexpected happens. Afterwards:
   `python3 tools/analyze_log.py log_011.bin`. The CSV has `route_leg` and `route_state` columns
   (state 1 turn, 2 drive, 3 avoid, 4 back up, 5 paused, 6 done).

### New teaching run (later)

Drive the route by hand at 0.20 and press **Mark** at every change: wall ends, start of a turn, ramp.
Then:
- Run `python3 tools/make_legs.py log_NNN.csv --check`.
- Adjust the PLAN list (which marks form a leg) until every leg fires.
- Paste the printed `kRoute` table into `RouteFollow.cpp`.
