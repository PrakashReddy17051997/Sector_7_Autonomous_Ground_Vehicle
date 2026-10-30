#pragma once
#include <Arduino.h>
// host stub: values set by the log replay
class Sensors { public:
  static const uint8_t kLidarPoints = 90;
  uint16_t lidarCm(uint8_t i) const { return i < kLidarPoints ? _lidarCm[i] : 0; }
  float yawDeg() const { return _yaw; } float yawNowDeg() const { return _yawNow; } float pitchDeg() const { return _pitch; }
  bool imuOk() const { return true; } bool gpsFix() const { return false; }
  double lat() const { return 0; } double lon() const { return 0; } float hdop() const { return 0; }
  uint16_t _lidarCm[kLidarPoints] = {0}; float _yaw=0, _yawNow=0, _pitch=0; };
