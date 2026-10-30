#pragma once
// RouteFollow - teach & repeat: drives the route of the teaching run (now log_013) leg by leg with
// LiDAR + gyro. GPS is NOT used (RULE_COMBINED: leg ends = landmark + LiDAR end scan).
//
// THE ROUTE (kRoute in RouteFollow.cpp, made by tools/make_legs.py from the tagged Mark presses)
//   17 legs (log_013), each:  kind   WALL = follow the wall on side L/R at dSet,  HOLD = keep a heading
//                   ref    direction of the leg in the "route yaw" frame (= gyro yaw of log_009)
//                   end    landmark that ends the leg: wall LOST, wall FOUND (closer than endArg),
//                          FRONT (something ahead closer than endArg), or EFFORT only
//                   effort how much driving the leg took in log_009 = sum(forward wheel cmd * s)
//                   gate   the landmark counts only after gate * effort (no early false end)
//   Between two legs: TURN on the spot (wheels +-0.5) to the next ref.
//
// LOCALISATION (why it works without GPS)
//   heading:  routeYaw = gyroYaw + offset. offset is set at the start from the wall
//             (rover parallel to the B facade: offset = ref - (yaw + wall angle)) and corrected on
//             every WALL leg: measured wall direction vs taught ref, 5 % per scan. log_009: the
//             wall directions of the route stay within 1-3 deg over 10 minutes -> the gyro holds.
//   progress: effort since the leg start (the same rover on the same ground needs the same
//             effort) + the landmark at the end of the leg. A leg that needs much more effort than
//             taught -> AUTOMATIC switch to the next leg (autoAdvance, 2026-10-24): after
//             advanceEffortFrac (1.3) * taught + 2 the leg counts as driven. Also on every leg:
//             after the gate, GPS inside 3 m of the taught end point ends the leg (zone help).
//             Only a stuck rover (5 back-ups) or a blocked way (3 x) still pauses - safety.
//   GPS:      zone check at each leg end (warning if > zoneWarnM from the taught end point), and on
//             EFFORT legs the leg may end early inside 2 m of the taught end (if GPS is not blind).
//             Leg 8 (open diagonal, no walls): heading pulled towards the taught end point, at most
//             +-15 deg from the taught heading.
//
// CONTROL (2026-10-29, second version after the field test log_014 - see _control in RouteFollow.cpp)
//   heading:  every loop pass on the live gyro yaw (50 Hz). Corrections are PIVOTS closed on the
//             gyro (a wheel difference does not turn this rover): error > corrDeg (8) after >= 0.6 s of
//             rolling, or > 25 at once -> pivot until the heading is reached minus what it still
//             turns (rate * 0.15 s). It aims a little past the target by the rover's own drift
//             (learned, ~3 deg/s to the right) so the drift brings it back instead of the next pivot.
//   WALL leg: yawDes = wall direction + approach atan((d - dSet) / lookAheadM 1.5 m), max +-15 deg;
//             the wall direction / distance are the median of the last 5 readings, a reading 8 deg
//             off that median is dropped (single scans fitting a corner or a gap: log_014).
//             No valid wall -> keep ref (gaps, doors).
//   start:    WALL leg: the rover must stand parallel to the wall (+-15 deg), else "turn rover
//             parallel" - log_0014 started at -54 deg and the whole leg was off.
//   closer than hardStopM (0.45 m) ahead: always obstacle handling (the static filter only for
//             0.45 - 0.7 m); a leg that ends at a wall ahead (END_FRONT) ends there.
//   obstacle: corridor ahead < avoidEnterM -> wait 3 s (people pass), then sidestep: turn on the
//             spot to the closest free direction (20..60 deg, never right on the raised path),
//             drive until the leg direction is free again, turn back parallel, hold the wall
//             direction for ~1.5 m, then back to the set distance. No free direction 3x -> PAUSE.
//             (tools/sim_route.py: box on the facade path passed with 0.36 m clearance)
//   stuck:    commanded to move but the LiDAR scan changed < 2.5 cm and yaw < 3 deg in 2 s,
//             twice -> back up 1.5 s; third time in a leg -> PAUSE "stuck".
//   (log_010: -0.10/+0.50 for 25 s without moving - this cannot happen any more)
//
//   Used by Mission (plan "route"); see Mission.h.
//
// LIDAR LOCALISATION (added 2026-10-28, RouteLocalizer + RouteMap.h from tools/make_segments.py)
//   endRule = RULE_COMBINED (default): a leg ends only where the live LiDAR scan looks like the scan
//             recorded at the END of that leg in the teaching run:
//               end = (landmark event && scan looks like the end) || clear dip of the end-scan cost
//             no GPS zone, last resort after lastResortFrac (1.8) x taught effort + 2.
//             Replay on the logs (make_segments.py --check): log_011 median error at the leg ends
//             14.3 -> 8.4 m, worst 20.3 -> 11.6 m; log_010 1.6 -> 5.5 m median, worst 7.4 -> 9.2 m.
//             Not field-tested yet: compare in the next test, endRule = RULE_OLD gives the old rule.
//   staticFilter (default on): something in the corridor ahead starts the obstacle handling only if
//             the teaching run did NOT see it there (walls ahead at turns, railings, ramp edges are
//             static). log_011: 139 events -> 24. SafetyGuard still stops at 25-30 cm for anything.
#include <Arduino.h>
#include <math.h>
#include "Sensors.h"
#include "WallFollow.h"
#include "RouteLocalizer.h"
#include "LocalLocalizer.h"

class RouteFollow {
public:
  enum Kind : uint8_t { WALL = 0, HOLD = 1 };
  enum End : uint8_t { END_LOST = 0, END_FOUND_L = 1, END_FOUND_R = 2, END_FRONT = 3, END_EFFORT = 4 };
  struct Leg {
    uint8_t kind; int8_t side; uint8_t end; float endArg;
    float ref, dSet, effort, gate;
    double lat, lon;          // GPS at the end of the leg in the teaching run (zone only)
    uint8_t gpsBlind;         // GPS did not move in the teaching run here (building shadow)
    uint8_t noRight;          // never avoid to the right (raised path, open edge)
    uint8_t gpsAssist;        // HOLD leg: heading pulled towards the end zone (+-15 deg)
    const char* name;
  };
  enum State : uint8_t { OFF = 0, TURN = 1, DRIVE = 2, AVOID = 3, BACKUP = 4, PAUSED = 5, DONE = 6 };

  // ---- tuning ----
  // heading controller (_control, every loop pass)
  float corrDeg = 8.0f;       // heading error above this -> gyro-closed pivot (after minRollMs rolling)
  float pivotDeg = 25.0f;     // above this: pivot at once
  uint32_t minRollMs = 600;   // roll at least this long between two corrections (no pivot dithering)
  float driftLeadS = 0.6f;    // aim this many seconds of the rover's own drift beyond the target
  float startTolDeg = 15.0f;  // WALL leg start: rover must be parallel to the wall within this
  float hardStopM = 0.45f;    // something this close ahead: always stop / avoid (static filter or not)
  // RAMP legs (bit i = leg i+1): the flat LiDAR beam hits the rising ramp surface and sees a "wall"
  // (log_013: 36 LiDAR-front safety stops on the ramp while you drove it by hand). On these legs the
  // LiDAR obstacle handling and hard stop are OFF, and cas_box.ino switches the SafetyGuard LiDAR
  // stop off; the front ULTRASONIC stop (25 cm), the tilt stop and the stuck detection stay ON.
  // log_013 route: leg 4 (M5-M10, ramp up/down), leg 5 (M10-M13, ramp), leg 11 (M23-M28, ramp).
  uint32_t rampLegs = (1u << 3) | (1u << 4) | (1u << 10);
  bool onRamp() const { return active() && ((rampLegs >> _i) & 1u); }
  // LOCAL PATH MODE (2026-10-29): every leg in its own frame (LocalPath.h, tools/localpath.py).
  // LocalLocalizer gives x / y / heading in the leg frame (gyro + speed guess + LiDAR matched
  // against the leg's taught walls); while it is LOCKED the rover steers to the taught path point
  // pathLookM ahead (pure pursuit, also in TURN) and the leg ends at the end of the taught path.
  // Not locked (8 scans without a match or none for 3 s): the leg-by-leg rules above take over
  // (wall / heading target, landmark + end scan). pathMode = false: always the leg-by-leg rules.
  bool pathMode = true;
  float pathLookM = 1.0f;
  float lpStopM = 0.20f;      // path mode: a MAPPED wall in the rover-wide corridor (0.18 m) closer than this = stop
  bool lineLook = true;       // carrot pathLookM ahead on the STRAIGHT-LINE path (corners of the taught path,
                              // 12 cm Douglas-Peucker); false: carrot on the raw taught path
  bool pathOn() const { return _lpOn; }
  bool pathLocked() const;
  float _pathErrRad() const;
  const LocalLocalizer& local() const { return _lp; }
  float wallJumpDeg = 8.0f;   // wall reading this far off the median of the last 5: dropped
  float deadbandDeg = 4.0f;   // (arcs only) no correction inside +-4 deg
  float turnDoneDeg = 9.0f;   // TURN / avoid turns count as done inside this (> corrDeg)
  float kP = 0.004f;          // wheel difference per deg of error (10 deg -> 0.04)
  float kI = 0.002f;          // (arcs only) per deg*s
  float arcMax = 0.0f;        // 0 = no arcs: a wheel difference does not turn this rover (log_014)
  // tap and settle pivots (log_015/016: answer 0.25-0.5 s late, ~30 deg per short tap)
  float tapGain0 = 100.0f;    // deg turned per second of tap, start value (learned per run: tapGain())
  float tapAim = 0.8f;        // ask a tap for this share of the error (undershoot, never overshoot)
  uint32_t tapMinMs = 40, tapMaxMs = 300;   // tap length limits (a big turn = several taps)
  float boostMax = 0.15f, boostStep = 0.03f;   // no progress while rolling: + this per 0.25 s, up to max
  uint32_t settleMs = 600;    // after a tap: wait this long (and until < 12 deg/s) before deciding again
  float rollBackCmd = 0.2f;   // rolling backwards (obstacle ahead)
  float lookAheadM = 1.5f;    // wall legs: aim at the set distance this far ahead
  float maxApproachDeg = 15.0f;
  float wallTolDeg = 20.0f;   // a wall line counts only if it runs within this of the leg direction
  float anchorGain = 0.05f;   // heading correction from the wall, per scan
  float avoidEnterM = 0.70f;  // corridor ahead shorter -> obstacle handling
  float avoidFreeM = 1.20f;
  bool autoAdvance = true;    // landmark not found -> go on with the next leg by itself (no pause)
  float advanceEffortFrac = 1.3f; // ... after this * taught effort (+2) on the leg
  float maxEffortFrac = 1.6f; // only with autoAdvance = false: more effort (+4) -> PAUSE
  uint8_t maxBackups = 5;     // stuck: back up and retry this often in one leg, then PAUSE
  float zoneWarnM = 15.0f;
  float pivotCmd = 0.5f;      // turning on the spot needs 0.5 (MotorShaper.pivotMin)
  // arcMax = 0 -> no arcs at all: pivots only (if the field test shows arcs stall this rover)
  enum EndRule : uint8_t { RULE_OLD = 0, RULE_COMBINED = 1 };
  uint8_t endRule = RULE_COMBINED;  // how a leg ends (see LIDAR LOCALISATION above)
  bool staticFilter = true;         // obstacle handling only for NEW things ahead
  float lastResortFrac = 1.8f;      // RULE_COMBINED: no end found after this * taught effort + 2 -> next leg
  const RouteLocalizer& localizer() const { return _loc; }

  static uint8_t count();
  static const Leg& leg(uint8_t i);

  // Start at leg i (0-based). anchor: set the heading from the wall / rover direction now
  // (rover placed parallel to the leg). Returns false + msg if it cannot start.
  // singleLeg: drive only leg i and stop at its end (segment test); else continue to the last leg
  bool start(const Sensors& s, const WallFollow& wf, uint8_t i, float speed, bool singleLeg = false);
  bool resume(float speed);               // continue the current leg (heading kept: gyro runs on)
  void stop() { _state = OFF; }
  void pause(const char* why);
  bool active() const { return _state != OFF && _state != DONE && _state != PAUSED; }
  bool canResume() const { return _hasRun && _state != DONE; }
  uint8_t state() const { return _state; }
  uint8_t legIndex() const { return _i; }
  float effortFrac() const { return _cur().effort > 0 ? _eff / _cur().effort : 0; }
  float routeYaw(const Sensors& s) const { return _wrap(s.yawDeg() + _off); }
  const char* msg() const { return _msg; }
  uint8_t forcedSwitches() const { return _forced; }
  bool singleLeg() const { return _stopAfter != 255; }
  const char* lastEnd() const { return _endInfo; }  // how the last leg ended (status / page)
  uint8_t logByte() const { return _hasRun ? (uint8_t)(0x80 | ((_state & 7) << 4) | ((_i + 1) & 15)) : 0; }
  uint8_t logLeg() const { return _hasRun ? (uint8_t)(_i + 1) : 0; }   // full leg number (> 15 legs)

  // every loop pass while active: writes l / r
  void steer(const Sensors& s, const WallFollow& wf, float& l, float& r);
  // every loop pass: what really went to the rover (after guard + shaper) -> effort
  void feedOutput(float l, float r);

private:
  const Leg& _cur() const { return leg(_i); }
  void _step(const Sensors& s, const WallFollow& wf);
  void _nextLeg(const Sensors& s, const char* how);
  bool _wallNow(const Sensors& s, const WallFollow& wf, int8_t side, float& d, float& dirYaw, float& rms, uint8_t& pts) const;
  float _corridor(const Sensors& s, float phiDeg, float half, float look) const;
  float _front(const Sensors& s) const;
  float _scanChangeCm(const Sensors& s) const;
  void _drive(float e, float v, bool fwd);      // sets the target; _control drives
  void _control(const Sensors& s);              // every loop pass: heading controller
  static float _wrap(float a) { a = fmodf(a + 180.0f, 360.0f); if (a < 0) a += 360.0f; return a - 180.0f; }
  static float _clamp(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }

  volatile uint8_t _state = OFF;
  uint8_t _resumeState = DRIVE;
  bool _hasRun = false, _overrun = false;
  uint8_t _forced = 0;                // legs switched without their landmark (status / log)
  uint8_t _i = 0;
  float _speed = 0.2f, _off = 0, _eff = 0, _I = 0;
  float _l = 0, _r = 0;               // command of the current step
  uint32_t _lastStep = 0, _lastOut = 0, _stateMs = 0;
  // end detection
  uint8_t _seen = 0, _lostN = 0, _foundN = 0, _lowN = 0;
  // controller
  bool _ctrlOn = false, _tgtFwd = true, _pivoting = false;
  float _tgtYaw = 0, _tgtV = 0, _yawDesF = 0, _rate = 0, _prevYaw = 0;
  uint32_t _ctrlMs = 0, _pivotMs = 0, _unstallUntil = 0, _arcMs = 0;
  bool _yawDesInit = false;
  float _drift = 0;                   // rover's own turn rate while rolling straight (deg/s, + = left)
  uint32_t _pivotEndMs = 0;
  int8_t _pivotDir = 1;
  uint8_t _tapPh = 0;                 // 0 idle, 1 tap running, 2 settling
  int8_t _tapDir = 1;
  uint32_t _tapUntil = 0, _tapEndMs = 0;
  float _tapYaw0 = 0, _tapReq = 0, _tapGain = 100.0f;
public:
  float tapGain() const { return _tapGain; }
  // which block set the wheels last (debug / log): the heading controller rolling straight, a turn tap,
  // the settle wait after a tap, standing (turn without forward), or the route step itself (obstacle
  // wait, back up, pause)
  enum Why : uint8_t { WHY_NONE = 0, WHY_ROLL = 1, WHY_TAP = 2, WHY_SETTLE = 3, WHY_STAND = 4, WHY_STEP = 5 };
  uint8_t why() const { return _why; }
  float ctrlErrDeg() const { return _ctrlErr; }   // heading error the controller works on (+ = left)
  float driftDegS() const { return _drift; }
  float boost() const { return active() ? _boost : 0.0f; }   // -> MotorShaper::boost every loop pass
  float progressCm() const { return _progCm; }               // median LiDAR change per 0.25 s (99 = unknown)
private:
  uint16_t _prevScan[Sensors::kLidarPoints] = {0};
  uint8_t _noProgN = 0;
  float _boost = 0, _progCm = 99;
  uint8_t _why = 0;
  float _ctrlErr = 0;
  // wall reading filter (median of 5)
  float _wyBuf[5], _dBuf[5];
  uint8_t _wN = 0, _wI = 0, _wRej = 0;
  bool _wallFilter(float& d, float& wy);
  float _stepRy = 0;                  // route yaw (snapshot) of the current _step
  // local path mode
  LocalLocalizer _lp;
  bool _lpOn = false;
  float _lpYawPrev = 0, _lpEffAcc = 0, _pitch0 = 0;
  uint32_t _lpOkMs = 0;
  float _fr[4] = {9, 9, 9, 9}; uint8_t _frN = 0, _frI = 0;
  // obstacle handling
  uint8_t _avPhase = 0, _avTries = 0; float _avTarget = 0, _avEff = 0, _passEff = -99;
  // stuck detection
  uint16_t _ref[Sensors::kLidarPoints];
  float _refYaw = 0; uint32_t _refMs = 0; bool _refMoving = false; uint8_t _stuckN = 0, _backups = 0;
  uint32_t _backupUntil = 0;
  // LiDAR localisation along the taught route
  RouteLocalizer _loc;
  uint16_t _scan[Sensors::kLidarPoints];
  char _msg[48] = "";
  uint8_t _stopAfter = 255;           // segment test: stop after this leg (255 = run to the end)
  char _endInfo[72] = "";
};
