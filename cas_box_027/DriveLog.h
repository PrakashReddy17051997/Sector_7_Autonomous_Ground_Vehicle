#pragma once
// DriveLog - records a drive to the box flash (LittleFS) for offline analysis ("teach" phase).
//
// What: every 250 ms one binary record (224 bytes, v2): time, GPS, IMU, drive command (from the pad and
//       after the safety guard), safety reason, rover battery, 5 ultrasonic, 90 LiDAR points.
//       ~0.9 kB/s -> the default partition scheme (~1.3 MB LittleFS) holds about 25 minutes.
// How:  records are collected in RAM and written in blocks of 16 (every 4 s, one write of ~3.5 kB,
//       typically 10..40 ms). Each "Start" opens a new file /log_NNN.bin.
// Web:  the main page loads /log.js, which adds a "Drive log" card with Start / Stop / Mark /
//       Delete all and download links. "Mark" puts a marker into the log.
// Mark TAGS (log version 2, 2026-10-29): before pressing Mark, choose on the page how the segment
//       that just ENDS was driven and what feature ends it. The tag is stored with the mark, and the
//       box measures the feature at the press (LiDAR left / right / front, IMU pitch) and shows it,
//       so you see at once whether the LiDAR really sees what you tagged. tools/make_legs.py builds
//       the legs from the tags (no hand-written PLAN, no notebook).
//       Teaching drive: press Mark (tag "start") where the route starts, then Mark with a tag at the
//       END of every segment. "Undo" cancels the last mark (a wrong press).
//   tag byte:  bits 0-3 feature that ends the segment
//                0 plain note (does not end a segment)  1 wall ends     2 wall starts
//                3 wall / object ahead                  4 ramp starts   5 ramp ends (flat again)
//                6 static object beside (pole, bench)   7 corner / turn 8 open area, no feature
//                14 start of the route                  15 undo the previous mark
//              bits 4-5 side of the feature: 0 -, 1 left, 2 right, 3 ahead
//              bits 6-7 how the segment was driven: 0 -, 1 along the LEFT wall, 2 along the RIGHT
//                wall, 3 keeping the heading (no wall)
// Threads: web routes only set request flags; all file work happens in update() on loop().
//
//   setup():  logger.begin(hmi.server());      // after hmi.begin(...)
//   loop():   logger.update(sensors, sample);  // every pass, after rover.setDrive(...)
//
// Decode on the laptop with tools/analyze_log.py (file format: see LogHeader / LogRecord below).
#include <Arduino.h>
#include <FS.h>
#include <ESPAsyncWebServer.h>
#include "Sensors.h"

class DriveLog {
public:
  // What loop() commanded in this pass (the rest is taken from Sensors).
  struct Sample {
    float cmdL = 0, cmdR = 0;   // from the drive pad (or your controller), -0.5..0.5
    float outL = 0, outR = 0;   // what was sent to the rover after the safety guard
    uint8_t safety = 0;         // SafetyGuard reason code (0 = ok)
    float batteryV = 0;         // rover.voltage()
    float rttMs = 0;            // rover.rttMs(): box -> rover command round trip
    uint8_t navByte = 0;        // mission.logByte(): 0x80 | route state << 4 | leg (0 = no route)
    uint8_t navLeg = 0;         // mission.logLeg(): route leg 1..255 (0 = no route) - the 4 bits in
                                // navByte only hold legs 1..15
    // v3 debug (2026-10-29): who drove and what the route code believed. GPS is not used any more,
    // so its record fields carry these (analyze_log.py decodes them for version 3):
    uint8_t dbg = 0;            // bits (OR over the 250 ms of a record): 0 turn tap, 1 settle after a tap,
                                // 2 MotorShaper start kick, 3 route step drives itself (wait / back up),
                                // 4 LiDAR located (path mode)  -> LogRecord.flags bits 3..7
    float lpX = 0, lpY = 0;     // position in the leg frame (m)          -> latE7  (cm, 2 x int16)
    float lpThDeg = 0;          // heading in the leg frame (deg)        -> lonE7 hi (0.1 deg)
    float carrotDeg = 0;        // heading error to the look-ahead point -> lonE7 lo (0.1 deg)
    float ctrlDeg = 0;          // heading error the controller acts on  -> hdopX100 (0.1 deg, int16)
    float lpScore = 0;          // LiDAR match score 0..1                -> sats (x 100)
  };

  static const uint16_t kPeriodMs = 250;     // record rate (= sensor snapshot rate)
  static const uint8_t kBufferRecords = 16;  // records per flash write
  static const uint32_t kMinFreeBytes = 32 * 1024;  // stop before the flash is full

  // Mounts LittleFS (formats it on first use: takes a few seconds once) and adds the web routes.
  // Returns false if the file system cannot be mounted; the box keeps working without logging.
  bool begin(AsyncWebServer* server);

  void update(const Sensors& s, const Sample& d);  // call every loop pass

  bool active() const { return _active; }
  String statusText() const;  // one line for the "Dev" view

private:
#pragma pack(push, 1)
  struct LogHeader {          // 16 bytes at the start of each file
    char magic[6];            // "CASLOG"
    uint8_t version;          // 3 (v3: GPS fields carry route debug, see Sample; 2 = GPS; 1 = no mark tags)
    uint8_t lidarPoints;      // 90
    uint16_t recordSize;      // sizeof(LogRecord)
    uint16_t periodMs;        // 250
    uint32_t startMs;         // millis() at start
  };
  struct LogRecord {          // little endian, no padding
    uint32_t ms;              // millis()
    int32_t latE7, lonE7;     // degrees * 1e7 (last valid fix)
    uint16_t hdopX100;
    uint8_t sats;
    uint8_t flags;            // bit0 GPS fix, bit1 IMU ok, bit2 LiDAR ok; v3: bits 3..7 = Sample::dbg
    int16_t yawX100;          // ISO 8855: + = left, 0 = magnetic north
    int16_t pitchX100;        // + = front down
    int16_t rollX100;         // + = right side down
    uint8_t magCal;           // magnetometer calibration 0..3, or (bit 7 set) route info:
                              //   0x80 | state << 4 | leg  (RouteFollow: state 1 turn, 2 drive,
                              //   3 avoid, 4 back up, 5 paused, 6 done; leg 1..10). IMUPLUS mode
                              //   has no magnetometer, so the byte was free.
    uint8_t mark;             // number of "Mark" presses so far
    int8_t cmdL, cmdR;        // drive pad command * 200 (-100..100)
    int8_t outL, outR;        // command after the safety guard * 200
    uint8_t safety;           // SafetyGuard reason code
    uint8_t rttX10ms;         // rover command round trip in 10 ms (0..2550 ms) - was 'reserved'
    uint16_t batteryX100;     // rover battery V * 100
    uint16_t usCm[5];         // front, left, right, back, front angled
    uint16_t lidarCm[Sensors::kLidarPoints];  // index 0 = front, clockwise, 4 deg each, 0 = none
    uint8_t markTag;          // v2: tag of the latest mark (see "Mark TAGS" above)
    uint8_t reserved;         // v2: route leg 1..255 (0 = no route); navByte keeps the state
  };
#pragma pack(pop)
  static_assert(sizeof(LogHeader) == 16, "LogHeader layout");
  static_assert(sizeof(LogRecord) == 44 + 2 * Sensors::kLidarPoints, "LogRecord layout");

  void _start();
  void _stop();
  void _flush();
  void _clear();
  void _fill(LogRecord& r, const Sensors& s, const Sample& d) const;
  void _measureMark(const Sensors& s);
  String _statusJson() const;
  void _refreshList();
  static bool _validName(const String& n);

  bool _fsOk = false;

  // requests from the web task (set there, handled in update())
  volatile bool _reqStart = false, _reqStop = false, _reqClear = false;
  // mark count (high byte) and tag of the latest mark (low byte) in ONE 16-bit value: the web task
  // writes it in one store, loop() reads it in one load -> count and tag always belong together
  volatile uint16_t _markWord = 0;
  uint8_t _marks() const { return (uint8_t)(_markWord >> 8); }
  uint8_t _seenMarks = 0, _seenTag = 0;  // loop(): last mark already measured, and its tag
  char _markInfo[72] = "";     // what the box measured at the latest mark (for the page)

  // state, written by loop(); read by the web task (single 32-bit/8-bit values: atomic)
  volatile bool _active = false;
  volatile uint32_t _records = 0;
  volatile uint32_t _usedKB = 0, _totalKB = 0;
  char _fileName[20] = "";
  char _error[40] = "";
  char _filesJson[360] = "";   // file list for the page, built in loop() (see _refreshList)
  mutable portMUX_TYPE _listMux = portMUX_INITIALIZER_UNLOCKED;

  fs::File _file;
  uint32_t _lastRecMs = 0;
  LogRecord _buf[kBufferRecords];
  uint8_t _bufCount = 0;
  uint8_t _dbgAcc = 0;          // Sample::dbg OR-ed over the record period
};
