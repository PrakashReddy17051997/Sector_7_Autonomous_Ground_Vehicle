#pragma once
// SelfCheck (24.11.2026) - sensor check while the COUNTDOWN runs (rover standing at the start, ~90 s).
//   ultrasonic  front / left / right / back: a sensor that reads 3-12 cm in >= 70 % of the samples while the LiDAR in
//               the same direction (+-16 deg) sees nothing closer than 40 cm is FAULTY (log_050: left read ~4 cm in the
//               open) -> switched off for this run (Sensors::usOffMask), shown on the page.
//   floor US    front-angled: should see the floor at 8-40 cm (drop check of the start turn) -> warning only.
//   LiDAR       at least 20 bins with a return on average -> else warning.
//   IMU         a valid sample at the end -> else warning.
// NOT here: the LiDAR mounting ANGLE. At a random start pose a turned LiDAR and a turned rover look the same; it is
// measured once per box at a long straight wall (Calibration card) and kept in flash.
#include <Arduino.h>
#include "Sensors.h"

class SelfCheck {
public:
  void reset() { _n = 0; _lidSum = 0; for (uint8_t i = 0; i < 4; i++) _near[i] = 0; _floorOk = 0; _done = false; _bad = 0; }
  // 4x per second while the countdown is active
  void sample(const Sensors& s) {
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
  }
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
    String j = "{\"n\":" + String(_n) + ",\"s\":" + String(_n / 4) + ",\"bad\":\"" + bad + "\",\"warn\":\"" + w + "\"}";
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
  uint8_t _bad = 0;
};
