#pragma once
// SafetyGuard - last check on every drive command before it goes to the rover (layer 1 of the
// collision-avoidance strategy: "emergency stop, overrides everything").
//
//   uint8_t why = guard.apply(sensors, l, r);   // changes l / r in place, returns a reason code
//   rover.setDrive(l, r);
//
// Rules (thresholds in Config):
//   TILT         |roll| or |pitch| too large -> stop; only slow straight reversing is allowed,
//                so you can back off an edge. Released when the box is level again (hysteresis).
//   FRONT_*      obstacle ahead (front / front-angled ultrasonic, LiDAR +-20 deg) -> no forward
//                motion; turning on the spot and reversing stay allowed.
//   BACK_US      obstacle behind -> no reversing.
// The command is split into forward speed v = (l+r)/2 and turn w = (l-r)/2; only v is limited,
// so the driver (or a controller) can always turn away from the obstacle.
//
// NOTE: a 2D LiDAR and horizontal ultrasonics cannot see a DROP (edge of the raised path).
//       TILT only catches it once the rover already tips. Check where "front angled" points:
//       if it looks at the floor, it can become a cliff sensor (set useFrontAngled = false here).
#include <Arduino.h>
#include "Sensors.h"

class SafetyGuard {
public:
  enum Reason : uint8_t { OK = 0, TILT = 1, FRONT_US = 2, FRONT_ANGLED = 3, FRONT_LIDAR = 4, BACK_US = 5, SIDE_US = 6 };

  struct Config {
    float maxRollDeg = 12, maxPitchDeg = 15;  // stop above these
    float levelDeg = 8;                       // released again below this (both axes)
    float tiltReverseMax = 0.15f;             // allowed reverse speed while tilted
    int frontStopCm = 50;                     // front ultrasonic (19.11.: was 25)
    int frontUsAloneCm = 25;                  // 19.11.: ... without the LiDAR agreeing only this close (floor echo)
    bool useFrontAngled = true;               // see NOTE above
    // 19.11. (course rule 17.11.: no turning on the spot): when a stop blocks the forward / backward part, the steering
    // part goes too. Before, v = 0 kept w -> left and right wheels opposite = a turn on the spot (and with curveOnly a
    // stop-start loop: RouteFollow zeroes both, asks again, the guard turns it into a spot turn again).
    bool noSpotTurn = true;
    int angledStopCm = 25;                    // front-angled ultrasonic
    int lidarStopCm = 30;                     // LiDAR in the front sector
    int lidarHalfAngleDeg = 20;               // front sector = +-20 deg
    // 24.11. LiDAR BODY STOP (a person stood in front of the rover and it drove into him): every LiDAR point in the
    // strip as wide as the rover (+ margin) ahead of the bumper counts. Closer than lidarBodyStopCm to the BUMPER ->
    // stop. Needs 2 points in one scan (legs, a box, a wall give several) or 1 point for 2 checks in a row (noise
    // drops out). 0 = off (the sketch switches it off on the ramp: the LiDAR sees the ramp surface).
    int lidarBodyStopCm = 30;                 // from the front bumper
    float lidarNoseCm = 12.5f;                // LiDAR centre -> front bumper (sketch: LIDAR_NOSE_CM)
    float lidarSideCm = 12.5f;                // LiDAR centre -> body side (sketch: LIDAR_SIDE_CM)
    float lidarMarginCm = 2.0f;               // strip = body width + this on each side (24.11.: 5 -> 2, log_052)
    int backStopCm = 20;                      // back ultrasonic
    int sideStopCm = 15;                      // 19.11.: left / right ultrasonic (driving forward)
  };

  Config cfg;

  // Limits l / r (each -0.5..0.5) in place. Returns the reason code (OK = nothing changed).
  uint8_t apply(const Sensors& s, float& l, float& r) {
    float v = (l + r) * 0.5f, w = (l - r) * 0.5f;
    uint8_t why = OK;

    if (s.imuOk()) {
      const float ar = fabsf(s.rollDeg()), ap = fabsf(s.pitchDeg());
      if (ar > cfg.maxRollDeg || ap > cfg.maxPitchDeg) _tilted = true;
      else if (ar < cfg.levelDeg && ap < cfg.levelDeg) _tilted = false;
    } else {
      _tilted = false;  // no IMU, no tilt information
    }

    if (_tilted) {
      v = constrain(v, -cfg.tiltReverseMax, 0.0f);
      w = 0;
      why = TILT;
    } else if (v > 0) {
      // 06.11. (log_029: 6 hard stops at 0.30 from single false echoes 4-22 cm while the LiDAR saw 3-7 m free):
      // the front ultrasonic stops only if the LiDAR front agrees (< frontStop + 30 cm) or 300 ms in a row
      const bool usNear = _valid(s.usCm(0)) && s.usCm(0) < cfg.frontStopCm;
      if (!usNear) _usMs = 0; else if (_usMs == 0) _usMs = millis() | 1u;
      // 19.11. (rain, log_043: a wet sensor read 16 cm for seconds): alone it must hold 1 s (was 300 ms)
      // 19.11. (log_048: floor echo 40-70 cm with the LiDAR free) alone only below frontUsAloneCm
      if (usNear && ((s.usCm(0) < cfg.frontUsAloneCm && millis() - _usMs >= 1000) || _lidarFrontMinCm(s) < (uint16_t)(cfg.frontStopCm + 30))) why = FRONT_US;
      else if (cfg.useFrontAngled && _valid(s.usCm(4)) && s.usCm(4) < cfg.angledStopCm) why = FRONT_ANGLED;
      else if (_lidarBodyStop(s)) why = FRONT_LIDAR;                   // 24.11.: body-wide strip, from the bumper
      else if (_lidarFrontMinCm(s) < cfg.lidarStopCm) why = FRONT_LIDAR;
      else if (_sideNear(s, 1) || _sideNear(s, 2)) why = SIDE_US;   // 19.11.
      if (why != OK) v = 0;
    } else if (v < 0) {
      // 06.11. (log_026-028: the back ultrasonic reads a constant 16 cm, the LiDAR sees nothing behind for
      // > 1.4 m -> every back-up was blocked): it only counts if the LiDAR rear sector agrees (or has no data)
      const uint16_t lr = _lidarRearMinCm(s);
      if (_valid(s.usCm(3)) && s.usCm(3) < cfg.backStopCm && (lr == 0xFFFF || lr < cfg.backStopCm + 25)) { why = BACK_US; v = 0; }
    }

    if (why != OK) {
      if (cfg.noSpotTurn && v == 0) w = 0;   // 19.11.: never only the steering part (= a turn on the spot)
      l = constrain(v + w, -0.5f, 0.5f);
      r = constrain(v - w, -0.5f, 0.5f);
    }
    return why;
  }

  static const char* reasonText(uint8_t why) {
    switch (why) {
      case OK: return "ok";
      case TILT: return "STOP: tilt";
      case FRONT_US: return "STOP: front ultrasonic";
      case FRONT_ANGLED: return "STOP: front-angled ultrasonic";
      case FRONT_LIDAR: return "STOP: LiDAR front";
      case BACK_US: return "STOP: back ultrasonic";
      case SIDE_US: return "STOP: side ultrasonic";
      default: return "?";
    }
  }

private:
  // ultrasonic 0 = not measured yet (UsManager start value)
  static bool _valid(int cm) { return cm > 2; }

  // 24.11.: LiDAR points in the rover-wide strip ahead, closer than lidarBodyStopCm to the bumper
  bool _lidarBodyStop(const Sensors& s) {
    if (cfg.lidarBodyStopCm <= 0 || !s.lidarOk()) { _lbN = 0; return false; }
    const float half = cfg.lidarSideCm + cfg.lidarMarginCm, reach = cfg.lidarNoseCm + cfg.lidarBodyStopCm;
    uint8_t pts = 0;
    for (uint8_t k = 0; k < Sensors::kLidarPoints; k++) {
      const uint16_t cm = s.lidarCm(k);
      if (!cm) continue;
      const float a = (k * 4.0f + 2.0f) * DEG_TO_RAD;                // bins clockwise from the front, bin centre
      const float al = cm * cosf(a), lt = cm * sinf(a);
      // 24.11. (log_052, ramp turn): only IN FRONT of the bumper - boards beside the front corners stopped it
      if (al > cfg.lidarNoseCm - 2.0f && al < reach && fabsf(lt) < half) pts++;
    }
    if (pts == 0) { _lbN = 0; return false; }
    if (_lbN < 255) _lbN++;
    return pts >= 2 || _lbN >= 2;
  }

  // closest LiDAR return within +-lidarHalfAngleDeg of the front; 0xFFFF if nothing seen
  uint16_t _lidarFrontMinCm(const Sensors& s) const {
    if (!s.lidarOk()) return 0xFFFF;
    const int degPer = 360 / Sensors::kLidarPoints;                 // 4
    const int n = (cfg.lidarHalfAngleDeg + degPer - 1) / degPer;    // points per side
    uint16_t best = 0xFFFF;
    for (int k = -n; k < n; k++) {  // point i covers [4i, 4i+4) deg: -n..n-1 is symmetric
      const uint8_t i = (uint8_t)((k + Sensors::kLidarPoints) % Sensors::kLidarPoints);
      const uint16_t cm = s.lidarCm(i);
      if (cm && cm < best) best = cm;
    }
    return best;
  }

  // 19.11.: left (us 1) / right (us 2) ultrasonic closer than sideStopCm - counts only if the LiDAR on that side
  // (90 / 270 +- 24 deg) has no data or also sees something within sideStopCm + 25 cm (rain: wet sensors)
  bool _sideNear(const Sensors& s, uint8_t us) const {
    const int u = s.usCm(us);
    if (!_valid(u) || u >= cfg.sideStopCm) return false;
    if (!s.lidarOk()) return true;
    const int c = us == 1 ? 270 : 90;   // LiDAR bins run clockwise from the front: left = 270 deg
    uint16_t best = 0xFFFF;
    for (int a = c - 24; a < c + 24; a += 4) { const uint16_t cm = s.lidarCm((uint8_t)(a / 4)); if (cm && cm < best) best = cm; }
    // 24.11. (log_050, box 10): "nothing seen" no longer confirms it - in the open area a faulty sensor (4 cm) stopped
    // the rover 51 times. Only a real LiDAR return close on that side confirms the ultrasonic.
    return best != 0xFFFF && best < (uint16_t)(cfg.sideStopCm + 25);
  }

  // closest LiDAR return behind (180 +- 28 deg); 0xFFFF if nothing seen / no LiDAR
  uint16_t _lidarRearMinCm(const Sensors& s) const {
    if (!s.lidarOk()) return 0xFFFF;
    uint16_t best = 0xFFFF;
    const int degPer = 360 / Sensors::kLidarPoints;
    for (int a = 152; a < 208; a += degPer) {
      const uint16_t cm = s.lidarCm((uint8_t)(a / degPer));
      if (cm && cm < best) best = cm;
    }
    return best;
  }

  bool _tilted = false;
  uint8_t _lbN = 0;     // 24.11.: checks in a row with a point in the body strip
  uint32_t _usMs = 0;   // since when the front ultrasonic reads close (0 = not close)
};
