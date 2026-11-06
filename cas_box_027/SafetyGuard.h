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
  enum Reason : uint8_t { OK = 0, TILT = 1, FRONT_US = 2, FRONT_ANGLED = 3, FRONT_LIDAR = 4, BACK_US = 5 };

  struct Config {
    float maxRollDeg = 12, maxPitchDeg = 15;  // stop above these
    float levelDeg = 8;                       // released again below this (both axes)
    float tiltReverseMax = 0.15f;             // allowed reverse speed while tilted
    int frontStopCm = 25;                     // front ultrasonic
    bool useFrontAngled = true;               // see NOTE above
    int angledStopCm = 25;                    // front-angled ultrasonic
    int lidarStopCm = 30;                     // LiDAR in the front sector
    int lidarHalfAngleDeg = 20;               // front sector = +-20 deg
    int backStopCm = 20;                      // back ultrasonic
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
      if (usNear && (millis() - _usMs >= 300 || _lidarFrontMinCm(s) < (uint16_t)(cfg.frontStopCm + 30))) why = FRONT_US;
      else if (cfg.useFrontAngled && _valid(s.usCm(4)) && s.usCm(4) < cfg.angledStopCm) why = FRONT_ANGLED;
      else if (_lidarFrontMinCm(s) < cfg.lidarStopCm) why = FRONT_LIDAR;
      if (why != OK) v = 0;
    } else if (v < 0) {
      // 06.11. (log_026-028: the back ultrasonic reads a constant 16 cm, the LiDAR sees nothing behind for
      // > 1.4 m -> every back-up was blocked): it only counts if the LiDAR rear sector agrees (or has no data)
      const uint16_t lr = _lidarRearMinCm(s);
      if (_valid(s.usCm(3)) && s.usCm(3) < cfg.backStopCm && (lr == 0xFFFF || lr < cfg.backStopCm + 25)) { why = BACK_US; v = 0; }
    }

    if (why != OK) {
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
      default: return "?";
    }
  }

private:
  // ultrasonic 0 = not measured yet (UsManager start value)
  static bool _valid(int cm) { return cm > 2; }

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
  uint32_t _usMs = 0;   // since when the front ultrasonic reads close (0 = not close)
};
