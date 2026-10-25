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

Adapt the object names to your sketch. Here they are `sensors`, `rover` (RoverLink) and `hmi` (WebHMI).

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

## 4. If the upload says "Sketch too big"

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
