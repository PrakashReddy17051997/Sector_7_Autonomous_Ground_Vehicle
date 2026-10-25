#pragma once
// RouteFollow - teach & repeat: drives the route of the teaching run (log_009) leg by leg with
// LiDAR + gyro. GPS is NOT used to steer - only as an approximate zone (check / end help).
//
// THE ROUTE (kRoute in RouteFollow.cpp, made by tools/make_legs.py from the Mark presses of log_009)
//   10 legs, each:  kind   WALL = follow the wall on side L/R at dSet,  HOLD = keep a heading
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
// CONTROL (changed after log_011, 2026-10-24 - see _drive in RouteFollow.cpp)
//   heading:  e = yawDes - routeYaw. The rover drives STRAIGHT (both wheels equal) and corrects
//             with pivot TAPS while rolling: |e| > tapDeg -> one 0.25 s tap (+-0.5, ~21 deg), two
//             taps if |e| > 40; then roll >= 0.75 s before the next tap. No arcs: in log_011 any
//             wheel difference nearly stalled the rover and left arcs did not turn it at all;
//             pivots from standstill or longer than 0.5 s stalled.
//   TURN:     the same: roll a little, tap, roll, tap ... (log_009: that is how you drove it)
//   WALL leg: yawDes = measured wall direction + approach angle  clamp(kDist*(d - dSet), +-20 deg)
//             towards / away from the wall. No valid wall -> keep ref (gaps, doors).
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
#include <Arduino.h>
#include <math.h>
#include "Sensors.h"
#include "WallFollow.h"

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
  float tapDeg = 12.0f;       // heading error above this -> pivot tap (one tap turns ~21 deg)
  uint8_t rollSteps = 3;      // roll at least 3 steps (0.75 s) between taps (a tap needs rolling)
  float rollBackCmd = 0.2f;   // rolling backwards between taps (obstacle ahead)
  float kDist = 40.0f;        // approach angle per metre of wall distance error
  float maxApproachDeg = 20.0f;
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

  static uint8_t count();
  static const Leg& leg(uint8_t i);

  // Start at leg i (0-based). anchor: set the heading from the wall / rover direction now
  // (rover placed parallel to the leg). Returns false + msg if it cannot start.
  bool start(const Sensors& s, const WallFollow& wf, uint8_t i, float speed);
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
  uint8_t logByte() const { return _hasRun ? (uint8_t)(0x80 | ((_state & 7) << 4) | ((_i + 1) & 15)) : 0; }

  // every loop pass while active: writes l / r
  void steer(const Sensors& s, const WallFollow& wf, float& l, float& r);
  // every loop pass: what really went to the rover (after guard + shaper) -> effort
  void feedOutput(float l, float r);

private:
  const Leg& _cur() const { return leg(_i); }
  void _step(const Sensors& s, const WallFollow& wf);
  void _nextLeg(const Sensors& s);
  bool _wallNow(const Sensors& s, const WallFollow& wf, int8_t side, float& d, float& dirYaw, float& rms, uint8_t& pts) const;
  float _corridor(const Sensors& s, float phiDeg, float half, float look) const;
  float _front(const Sensors& s) const;
  float _scanChangeCm(const Sensors& s) const;
  void _drive(float e, float v, bool fwd);
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
  uint8_t _tapLeft = 0, _rollN = 9; int8_t _tapDir = 1;
  float _fr[4] = {9, 9, 9, 9}; uint8_t _frN = 0, _frI = 0;
  // obstacle handling
  uint8_t _avPhase = 0, _avTries = 0; float _avTarget = 0, _avEff = 0, _passEff = -99;
  // stuck detection
  uint16_t _ref[Sensors::kLidarPoints];
  float _refYaw = 0; uint32_t _refMs = 0; bool _refMoving = false; uint8_t _stuckN = 0, _backups = 0;
  uint32_t _backupUntil = 0;
  char _msg[48] = "";
};
