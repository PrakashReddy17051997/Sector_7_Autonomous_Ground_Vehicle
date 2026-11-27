#pragma once
// SelfCheck (24.11.2026) - sensor check while the COUNTDOWN runs (rover standing at the start, ~90 s).
//   ultrasonic  front / left / right / back: a sensor that reads 3-12 cm in >= 70 % of the samples while the LiDAR in
//               the same direction (+-16 deg) sees nothing closer than 40 cm is FAULTY (log_050: left read ~4 cm in the
//               open) -> switched off for this run (Sensors::usOffMask), shown on the page.
//   floor US    front-angled: should see the floor at 8-40 cm (drop check of the start turn) -> warning only.
//   LiDAR       at least 20 bins with a return on average -> else warning.
//   IMU         a valid sample at the end -> else warning.
// 27.11. (logs 058-061: a rain cap slipped over the LiDAR, the rover drove on blind) - PRE-START CHECK:
//   BLOCKS the start  LiDAR blind (< kBlindBins bins with a return for 2 s), IMU not ready (no valid sample now).
//                     The sketch holds the start (display "START HELD") and starts as soon as the check passes, at
//                     most kHoldMs after 0 s; a double tap cancels.
//   warns only        IMU gyro calibration < 3/3 (standing still during the countdown calibrates it), battery
//                     below kLowBattV, few LiDAR returns on average, floor US, ultrasonics switched off.
// NOT here: the LiDAR mounting ANGLE. At a random start pose a turned LiDAR and a turned rover look the same; it is
// measured once per box at a long straight wall (Calibration card) and kept in flash.
#include <Arduino.h>
#include "Sensors.h"

class SelfCheck {
public:
  static const uint16_t kBlindBins = 10;     // 27.11.: fewer bins with a return = blind (normal: 50-65 of 90)
  static constexpr float kLowBattV = 11.2f;   // 27.11.: rover pack (3x 18650), warn below
  static const uint32_t kHoldMs = 60000;     // 27.11.: a held start gives up after this
  void reset() { _n = 0; _lidSum = 0; for (uint8_t i = 0; i < 4; i++) _near[i] = 0; _floorOk = 0; _done = false; _bad = 0; _blind = 0; _gyroCal = 0; _batV = 0; }
  // 4x per second while the countdown is active
  void sample(const Sensors& s, float batteryV = 0) {
    const uint32_t now = millis();
    if (now - _lastMs < 250) return;
    _lastMs = now;
    if (_n < 60000) _n++;
    static const int dirDeg[4] = {0, 270, 90, 180};          // us 0 front, 1 left, 2 right, 3 back (LiDAR bins clockwise)
    for (uint8_t i = 0; i < 4; i++) {
      const int u = s.usRawCm(i);
      if (u > 2 && u <= 12 && _lidarMin(s, dirDeg[i]) > 40) _near[i]++;
    }
    const int fa = s.usRawCm(4);
    if (fa >= 8 && fa <= 40) _floorOk++;
    uint16_t bins = 0;
    for (uint8_t k = 0; k < Sensors::kLidarPoints; k++) if (s.lidarCm(k)) bins++;
    _lidSum += bins;
    _imuOk = s.imuOk();
    if (bins < kBlindBins) { if (_blind < 255) _blind++; } else _blind = 0;   // 27.11.
    uint8_t sy, gy, ac, mg;
    s.imu().calibration(sy, gy, ac, mg);
    _gyroCal = gy;
    _batV = batteryV;
  }
  uint32_t samples() const { return _n; }
  bool lidarBlind() const { return _blind >= 8; }          // 2 s
  // 27.11.: why the start is held ("" = ok to start)
  const char* blockWhy() const {
    if (_n < 8) return "check running";
    if (lidarBlind()) return "LiDAR blocked";
    if (!_imuOk) return "IMU not ready";
    return "";
  }
  bool startOk() const { return blockWhy()[0] == 0; }
  // at 0 s (or any time): result. Bits of usOffMask for faulty ultrasonics.
  uint8_t badMask() const {
    uint8_t m = 0;
    if (_n >= 20) for (uint8_t i = 0; i < 4; i++) if (_near[i] * 10 >= _n * 7) m |= (uint8_t)(1u << i);
    return m;
  }
  String json() const {
    static const char* nm[4] = {"front", "left", "right", "back"};
    String bad = "";
    const uint8_t m = badMask();
    for (uint8_t i = 0; i < 4; i++) if (m & (1u << i)) { if (bad.length()) bad += ", "; bad += nm[i]; }
    String w = "";
    if (_n >= 20 && _floorOk * 2 < _n) w += "front-below US does not see the floor; ";
    if (_n >= 20 && _lidSum / _n < 20) w += "LiDAR: few returns; ";
    if (!_imuOk) w += "IMU not ready; ";
    if (_n >= 8 && _gyroCal < 3) w += "IMU gyro cal " + String(_gyroCal) + "/3; ";
    if (_batV > 1.0f && _batV < kLowBattV) w += "battery " + String(_batV, 1) + " V; ";
    String j = "{\"n\":" + String(_n) + ",\"s\":" + String(_n / 4) + ",\"bad\":\"" + bad + "\",\"warn\":\"" + w +
               "\",\"ok\":" + (startOk() ? "true" : "false") + ",\"block\":\"" + blockWhy() + "\"}";
    return j;
  }
private:
  static int _lidarMin(const Sensors& s, int c) {
    int best = 9999;
    for (int a = c - 16; a < c + 16; a += 4) {
      const uint16_t cm = s.lidarCm((uint8_t)(((a + 360) % 360) / 4));
      if (cm && cm < best) best = cm;
    }
    return best;
  }
  uint32_t _lastMs = 0, _n = 0, _lidSum = 0, _near[4] = {0, 0, 0, 0}, _floorOk = 0;
  bool _imuOk = false, _done = false;
  uint8_t _bad = 0, _blind = 0, _gyroCal = 0;
  float _batV = 0;
};
