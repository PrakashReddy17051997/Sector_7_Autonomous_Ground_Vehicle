#pragma once
// LocalLocalizer - where the rover is INSIDE the current leg, in that leg's own frame (no GPS, no
// global map). Twin of tools/localpath.py (same numbers, host-testable: no Arduino calls).
//
// LEG FRAME  origin = where the leg starts in the teaching run, x = the rover's heading there,
//            y = left, metres. Data per leg in LocalPath.h (tools/localpath.py build).
// STEP (call once per sensor snapshot, 4 Hz, in every route state):
//   predict  heading += gyro change; position += dist along the heading (dist = 1.0 m/s per
//            forward command * time = K_SPEED * effort change); nothing while tilted or while the
//            scan has not changed for 1 s (stuck, wheels slipping)
//   correct  live scan (0.3-6 m, rear sector = operator left out, >= 12 points) scored against the
//            leg grid: first scan of a route +-1.5 m / +-20 deg, else +-0.4 m in 5 cm and -4..+4 deg,
//            then 2 cm / 1 deg; a weak prior holds the prediction where the walls say nothing.
//            Score > 0.35 -> accepted.
//   lost     8 unaccepted scans or > 1 m off the path: search along the taught path (3 m back to
//            3 m ahead, every 0.5 m, path direction +-20 deg) - the distance along a straight
//            corridor is the uncertain direction. Score > 0.45 -> accepted.
//   end      position along the path >= sEnd - 0.25 m -> atEnd(); the caller hands over with
//            nextLeg() (pose moved into the next leg's frame).
// Replay check: tools/localpath.py replay  (log_012 on the log_013 legs: 17 of 17 legs, 73 %).
#include <stdint.h>
#include <stddef.h>

class LocalLocalizer {
public:
  static const uint8_t kN = 90;
  float kSpeed = 1.0f;          // m/s per forward command (LiDAR-measured, log_012)
  float accept = 0.35f, acceptReloc = 0.45f;
  float thPrior = 40.0f;        // score per rad^2 of heading change against the gyro (2 deg = 0.05, 4 deg = 0.2)

  bool hasMap() const;
  uint8_t legs() const;
  void begin(uint8_t leg);      // start of a route at this leg: pose (0,0,0), first match wide
  // one sensor snapshot: scan (cm, 0 = none), gyro yaw change (rad, + = left), forward distance
  // estimate (m) since the last step, rover tilted (pitch off by >= 5 deg)
  void step(const uint16_t* scanCm, float dyawRad, float distM, bool tilted);
  bool atEnd() const { return _atEnd; }
  void nextLeg();               // hand the pose over to the next leg's frame

  // state (leg frame)
  uint8_t leg() const { return _leg; }
  float x() const { return _x; }
  float y() const { return _y; }
  float th() const { return _th; }        // rad
  float s() const { return _pi * 0.25f; } // m along the taught path of this leg
  float sEnd() const;
  float off() const { return _off; }      // m from the taught path
  float score() const { return _score; }
  bool ok() const { return _ok; }         // this scan matched
  uint8_t lostN() const { return _lost; }
  // pure pursuit: path point lookM ahead of the projection, as heading error (rad, + = left)
  float headingErrorTo(float lookM) const;
  // STRAIGHT-LINE look-ahead: the taught path cut into straight pieces (Douglas-Peucker, 12 cm),
  // the corners = waypoints. Target = the first waypoint at least minAheadM further along the path;
  // heading error to it (rad, + = left). On a straight piece the target stays the same corner for
  // metres -> a steady heading, no chasing of a point that wanders with every scan match.
  float headingErrorToWaypoint(float minAheadM) const;
  void waypoint(float minAheadM, float& lx, float& ly) const;          // that waypoint (leg frame, m)
  void waypointGlobal(float minAheadM, float& gx, float& gy) const;    // ... in the web map frame
  uint8_t waypoints() const { return _nWp; }
  // points in the corridor ahead (|lateral| < half, 0.05 m < along < look, > 25 cm) that are NOT
  // walls of the taught map at the current pose = new obstacles (people, bikes, bins)
  uint8_t newPointsAhead(const uint16_t* scanCm, float half, float look) const;
  // pose in the web map frame (Route map card)
  void global(float& gx, float& gy, float& gth) const;

  // matching core (public for the host test)
  float scoreAt(const float* px, const float* py, int n, float x, float y, float th) const;

private:
  struct Best { float x, y, th, sc; };
  Best _search(const float* px, const float* py, int n, float cx, float cy, float ct, float step, int nh,
               const int8_t* dthDeg, int nth, float prx, float pry, float prth, float lam, float thW) const;
  void _match(const float* px, const float* py, int n, bool wide);
  void _relocate(const float* px, const float* py, int n);
  void _project();
  bool _still(const uint16_t* scanCm);
  void _buildWp();
  uint16_t _wpAt(float minAheadM) const;

  uint8_t _leg = 0;
  float _x = 0, _y = 0, _th = 0, _off = 0, _score = 0;
  uint16_t _pi = 0;
  bool _ok = false, _first = true, _atEnd = false;
  uint8_t _lost = 0;
  uint16_t _hist[4][kN];        // last 4 scans (still detection over 1 s)
  uint8_t _histN = 0, _histI = 0;
  static const uint8_t kMaxWp = 40;
  uint16_t _wp[kMaxWp];         // path indices of the corners of this leg (first = 0, last = end)
  uint8_t _nWp = 0;
};
