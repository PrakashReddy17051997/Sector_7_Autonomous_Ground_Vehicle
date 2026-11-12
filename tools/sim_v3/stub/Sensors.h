#pragma once
// Host stub of Sensors for the simulator: the sim writes the fields, the firmware reads them through the same API.
#include <Arduino.h>
class Sensors {
public:
  static const uint8_t kLidarPoints = 90;   // 4 deg bins, index clockwise from ahead, closest return in cm, 0 = none
  static const int kUsMaxCm = 300;
  uint16_t lidarCm(uint8_t i) const { return i < kLidarPoints ? _lidarCm[i] : 0; }
  bool lidarOk() const { return true; }
  float yawDeg() const { return _yaw; }
  float yawNowDeg() const { return _yaw; }
  float pitchDeg() const { return _pitch; }
  float rollDeg() const { return _roll; }
  bool imuOk() const { return true; }
  int usCm(uint8_t i) const { return i < 5 ? _us[i] : kUsMaxCm; }   // 0 front, 1 left, 2 right, 3 back, 4 front angled
  bool gpsFix() const { return false; }
  double lat() const { return 0; }
  double lon() const { return 0; }
  float hdop() const { return 99; }
  uint16_t _lidarCm[kLidarPoints] = {0};
  int _us[5] = {300, 300, 300, 300, 17};
  float _yaw = 0, _pitch = 0, _roll = 0;
};
