#pragma once
// LandmarkLeg - legs driven on LANDMARK LINES instead of the taught-path match (v3, 2026-11-10).
// Host-testable (no Arduino calls). Used by RouteFollow for legs whose kind is LMK.
//
// IDEA (from the "Open Sea" / "last leg to Block I" method): in open areas there is nothing to match
// a whole scan against, the gyro drifts ~2 deg/min and GPS is 6-15 m off. But a few LONG STRAIGHT
// things are always there: the wall behind at the start, a store front on one side, a building front,
// the wall ahead at the end. A leg is cut into PHASES by the distance driven (odometry s, metres);
// each phase looks for ONE such line and uses it:
//
//   ANCHOR   line behind / ahead, perpendicular to the course: gives the heading (re-anchors the
//            route yaw -> removes the gyro drift); steering = hold the course.
//   SIDE     line beside the course at a known distance dRef: PID on the distance error
//              e = d - dRef,  turn = kp*e + ki*int(e) + kd*de/dt   (deg, limited to +-sat)
//            yawDes = (course the line stands for) + towards-the-line * turn
//            The line counts only if it runs parallel to the course (+-paraDeg) and dMin <= d <= dMax.
//   EDGE     like SIDE, but dRef = the distance FIRST SEEN in this phase (walkway edge, grass edge,
//            benches - you do not know the exact taught distance, you just keep it).
//   CORRIDOR like EDGE on the facade side, and the phase ends early when the OTHER side also has a
//            line closer than b and the width (d + d_other) is within [a, a2]  (railing corridor).
//   ENTRANCE like EDGE; the facade "steps back" (d jumps up by >= a) or the glass lets the beams
//            through (no-return share in the facade sector >= b), 2 of 3 scans -> stop c metres later.
//   STOP     line AHEAD, perpendicular: steer square to it, leg ends when d <= dRef.
//            Crowd rule (v10): the line must be >= minLen long (a person is ~0.5 m, a group of 5 in a
//            row is not straight to 12 cm), seen in 3 of the last 5 scans, and fixed in space:
//            d + s stays within 0.4 m (people walking towards the rover fail this).
//   HOLD     no landmark: hold the course (route yaw = gyro + offset).
// No valid line in a phase -> HOLD for that scan (the gyro bridges gaps of a few seconds).
// A phase ends at s >= sTo (or its event); after the last phase the leg ends ("distance end").
//
// Frames: route yaw (deg, + = left) as in RouteFollow. ref = the taught course of the leg.
//   line course   lineYaw = ry - (footExp - foot)   (see LineFeat.h); on course lineYaw = ref + lineOff
//   anchor        dOff = anchorGain * wrap(ref + lineOff - lineYaw)    (added to RouteFollow::_off)
//   steering      yawDes = lineYaw - lineOff (+ PID turn)  - the same "nose heading" frame as ref
//   gaps          the line is found on ~1 of 2 scans: between hits the last offset to ref is HELD
//                 (fading out over holdS), so the command does not jump between line and bare ref
#include <stdint.h>
#include <math.h>
#include "LineFeat.h"

struct LmkPhase {
  uint8_t type;        // LandmarkLeg::Type
  float footExp;       // where the line is on course: +90 left, -90 right, 0 ahead, 180 behind
  float sFrom, sTo;    // phase window, m along the leg (sFrom = before it the line is not searched)
  float dRef;          // SIDE: set distance; STOP: stop distance; (EDGE/CORRIDOR/ENTRANCE: 0 = latch)
  float dMin, dMax;    // accepted line distance
  float kp, ki, kd;    // deg per m, deg per (m*s), deg per (m/s)
  float satDeg;        // turn limit
  float paraDeg;       // line course must be within this of the leg course
  float minLen;        // m
  float anchorGain;    // per scan (0.05 = 5 %), 0 = no heading anchor from this line
  float a, a2, b, c;   // type-specific (see above)
  float lineOff;       // taught course of the line MINUS the leg ref (deg). Measured on the teaching log
                       // (tools/landmark_scan.py --table). log_027 leg 2: -9.6 for all three walls = the
                       // rover's crab (nose 7-10 deg left of its track) - the walls themselves are square.
};

class LandmarkLeg {
public:
  enum Type : uint8_t { HOLD = 0, ANCHOR = 1, SIDE = 2, EDGE = 3, CORRIDOR = 4, ENTRANCE = 5, STOP = 6 };
  struct Out {
    float yawDes = 0;    // route yaw to steer to
    float dOff = 0;      // heading anchor correction for RouteFollow::_off (deg)
    bool line = false;   // a valid landmark line was used this scan
    bool done = false;   // leg finished
    const char* why = "";
  };

  void begin(const LmkPhase* ph, uint8_t n) { _ph = ph; _n = n; _p = 0; _resetPhase(); _done = false; _sStop = -1; }
  bool active() const { return _ph && _n; }
  uint8_t phase() const { return _p; }
  uint8_t phases() const { return _n; }
  float lastD() const { return _lastD; }
  float lastE() const { return _lastE; }
  float turnDeg() const { return _turn; }
  float sStop() const { return _sStop; }
  const char* typeName() const;

  // one LiDAR snapshot (4 Hz): scan (cm), route yaw now, leg course ref, distance s (m), dt (s),
  // tilted = on a slope (the flat LiDAR plane hits the ground / sky): no line is trusted, hold
  Out step(const uint16_t* scanCm, float ry, float ref, float s, float dt, bool tilted = false);

private:
  static float _clamp(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }
  float holdS = 2.0f;  // keep the last line-based steering offset this long through gaps
  void _resetPhase() { _holdOff = 0; _holdT = 99; _I = 0; _dPrev = -1; _deF = 0; _latch = -1; _hit = 0; _hitN = 0; _evt = 0; _evtN = 0; _wN = 0; _dropBase = -1; }
  LineFeat::Line _line(const uint16_t* scan, float footExp, float dMin, float dMax, float minLen) const;
  float _pid(float e, float dt, float sat, const LmkPhase& P);

  const LmkPhase* _ph = nullptr;
  uint8_t _n = 0, _p = 0;
  bool _done = false;
  float _I = 0, _dPrev = -1, _deF = 0, _latch = -1, _sStop = -1, _dropBase = -1;
  float _lastD = 0, _lastE = 0, _turn = 0;
  uint8_t _hit = 0, _hitN = 0, _evt = 0, _evtN = 0;   // 5-scan (STOP) / 3-scan (ENTRANCE) bit histories
  float _w[5]; uint8_t _wN = 0;
  float _holdOff = 0, _holdT = 99;                    // last (yawDes - ref) from a line, s since then
  void _gap(Out& o, float ref, float dt) {            // no line this scan: fade the last offset out
    _holdT += dt;
    if (_holdT < holdS) o.yawDes = LineFeat::wrap180(ref + _holdOff * (1.0f - _holdT / holdS));
  }
  void _onHit(Out& o, float ref) { _holdOff = LineFeat::wrap180(o.yawDes - ref); _holdT = 0; }                       // STOP: d + s of the confirming scans
};
