#pragma once
// WallFollow - autonomous wall following with the LiDAR (leg 1: raised path along building B).
//
// SAFETY: like GpsNav, the rover only moves while you HOLD the "up" key (dead-man switch);
// release = stop, other keys = manual override. SafetyGuard (30 cm LiDAR / 25 cm ultrasonic stop)
// and the acceleration ramp still act afterwards. The LiDAR cannot see a DROP: start with a
// distance that keeps the rover well away from the open edge of the raised path.
//
// 1. Wall detection (every new LiDAR snapshot, 4x/s):
//    - take the LiDAR points on the chosen side (left = 270 deg, right = 90 deg), closer than 3 m
//    - fit a straight line to the points within +-20 deg of the side direction (principal axis)
//    - keep every point within +-60 deg that lies within 10 cm of that line, fit again (2x)
//    - valid if >= 5 points, line at least 0.6 m long; gives distance d and wall angle theta
//      (theta = 0: driving parallel; < 0: nose turned towards the wall)
//    Tested on log_003: standing at B, d = 1.52 m +- 3.5 cm, fit error ~1 cm.
// 2. Control (side frame: "towards the wall" positive):
//      wanted angle  thetaDes = -clamp(kDist * (d - dSet), +-maxApproachDeg)
//      turn          wToward  = kAngle * (theta - thetaDes), limited to +-turnFrac * speed
//    While a wall is known, the fit starts from the points near the last line ("tracking"), so a
//    door or gap in front of the rover does not lose the wall.
// 3. Obstacle ahead (corridor 70 cm wide, 1.5 m long, LiDAR + front ultrasonic < 0.9 m): AVOID.
//    Tries directions -60..+60 deg and takes the one closest to straight ahead with >= 1.2 m free
//    (never towards the open side unless allowAwayFromWall). Back to FOLLOW when straight is free.
// 4. Wall lost (no valid line, or the wall only continues BEHIND the rover): straight on for up
//    to 2 s (gaps), then stop with state
//    "lost" = end of the facade -> later the switch to the next leg.
// Simulated in tools/sim_wallfollow.py (door gap, box on the path, wall end): wall distance
// 1.02..1.10 m for a set point of 1.10 m, box passed with 0.34 m clearance, stops 0.3-0.4 m
// after the end of the wall.
//
//   setup(): wall.begin(hmi.server());
//   loop():  wall.pollRequests(sensors);
//            if (wall.active()) wall.steer(sensors, padL, padR, padAge, l, r);
#include <Arduino.h>
#include <math.h>
#include <ESPAsyncWebServer.h>
#include "Sensors.h"

class WallFollow {
public:
  enum State : uint8_t { OFF = 0, FOLLOW = 1, BRIDGE = 2, LOST = 3, AVOID = 4 };
  enum Side : int8_t { LEFT = -1, RIGHT = 1 };

  // ---- tuning ----
  float kDist = 40.0f;          // deg of approach angle per metre of distance error
  float maxApproachDeg = 20.0f; // never approach / leave the wall steeper than this
  float kAngle = 0.008f;        // turn command per degree of angle error
  float turnFrac = 0.6f;        // max turn as a fraction of the speed
  float frontSlowM = 0.9f;      // slow down when the corridor ahead is shorter than this
  float frontMinM = 0.35f;      // ... speed 0 here (SafetyGuard stops at 0.30 m)
  float avoidEnterM = 0.9f;     // corridor ahead shorter than this -> AVOID
  float avoidFreeM = 1.2f;      // a direction counts as free with this much corridor
  float halfWidthM = 0.35f;     // corridor half width = rover half width + margin
  float lookAheadM = 1.5f;      // corridor length checked
  bool allowAwayFromWall = false;  // false: never avoid towards the open side (raised path edge)
  float jumpM = 0.35f;          // a new distance this far from the last one is rejected
  uint32_t bridgeMs = 2000;     // keep going straight this long without a wall (gaps)

  bool begin(AsyncWebServer* server);
  void pollRequests(const Sensors& s);  // every loop pass (handles web buttons)
  bool active() const { return _state != OFF; }
  void stop() { _state = OFF; }
  bool start(const Sensors& s, Side side);   // follow this side at the current distance
  uint8_t state() const { return _state; }
  Side side() const { return _side; }
  void steer(const Sensors& s, float padL, float padR, uint32_t padAge, float& l, float& r);
  String statusText() const;

  // last wall measurement (for the page / your own code)
  struct Wall { bool ok; float distM, angleDeg, rmsM; uint8_t points; };
  const Wall& lastWall() const { return _wall; }
  // One wall fit. prior = last wall line (tracking) or nullptr (fresh search).
  Wall measure(const Sensors& s, Side side, const Wall* prior) const;
  float corridorM(const Sensors& s, float phiDeg) const;  // free length ahead in direction phi

private:
  String _statusJson() const;
  static float _clamp(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }

  volatile bool _reqLeft = false, _reqRight = false, _reqOff = false;
  volatile int8_t _reqAdjCm = 0;

  volatile uint8_t _state = OFF;
  Side _side = LEFT;
  float _dSet = 1.0f;
  Wall _wall = {false, 0, 0, 0, 0};
  float _lastGoodD = -1;
  uint32_t _lastGoodMs = 0;
  float _frontM = 0, _wCmd = 0;
  volatile float _liveL = -1, _liveR = -1;  // wall distance seen now on each side, -1 = none
  char _msg[40] = "";
};
