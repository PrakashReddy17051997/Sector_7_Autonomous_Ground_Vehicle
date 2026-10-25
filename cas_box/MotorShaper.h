#pragma once
// MotorShaper - makes small wheel commands actually move the rover (replaces the old ramp).
//
// Why (logs 5-8, rotation checked with the LiDAR, not only the IMU):
//   - turning on the spot with |command| <= 0.3: NO rotation at all;
//     0.3 .. 0.5: rotation only about half of the time, 4-6 deg/s
//   - the GPS leg of the mission sent 0.02 .. 0.18 -> the rover moved 0.1-0.4 m in several seconds
//   - you needed 0.5 to start or turn from standstill (static friction of the skid steering)
//
// What it does, per wheel (-0.5 .. 0.5):
//   1. dead band:   |cmd| < zero               -> 0
//   2. moving:      |cmd| -> deadband + (0.5 - deadband) * |cmd| / 0.5   (keeps the order, so a
//                   small difference between the wheels still turns the rover)
//   3. pivot:       wheels in opposite directions -> both scaled up to at least pivotMin
//   4. breakaway:   from standstill or when a wheel changes direction: for kickMs the larger wheel
//                   gets kick (0.5), the other one keeps its ratio -> then back to the shaped value
// A stop (0 / 0) always passes through immediately. No kick while SafetyGuard limits (tilt).
//
//   shaper.apply(l, r, allowKick);   // after SafetyGuard, right before rover.setDrive(l, r)
#include <Arduino.h>
#include <math.h>

class MotorShaper {
public:
  float zero = 0.02f;       // below this a wheel command counts as 0
  float deadband = 0.18f;   // smallest wheel command that moves the rover (tune from the logs)
  float pivotMin = 0.50f;   // turning on the spot: 0.3 never turned, 0.45-0.5 only half the time
  float kick = 0.50f;       // breakaway command
  uint32_t kickMs = 400;    // breakaway duration

  void apply(float& l, float& r, bool allowKick) {
    if (fabsf(l) < zero) l = 0;
    if (fabsf(r) < zero) r = 0;
    if (l == 0 && r == 0) { _sl = _sr = 0; _kickUntil = 0; return; }

    const bool pivot = (l > 0 && r < 0) || (l < 0 && r > 0);
    if (pivot) {
      const float m = fmaxf(fabsf(l), fabsf(r));
      if (m < pivotMin) { l *= pivotMin / m; r *= pivotMin / m; }
    } else {
      l = _shape(l);
      r = _shape(r);
    }

    // breakaway: was standing, or a wheel changed its direction
    const int8_t sl = (l > 0) - (l < 0), sr = (r > 0) - (r < 0);
    const uint32_t now = millis();
    if (allowKick && ((_sl == 0 && _sr == 0) || (sl != _sl && sl != 0) || (sr != _sr && sr != 0))) {
      _kickUntil = now + kickMs;
    }
    _sl = sl;
    _sr = sr;
    if (allowKick && (int32_t)(_kickUntil - now) > 0) {
      const float m = fmaxf(fabsf(l), fabsf(r));
      if (m > 0 && m < kick) { l *= kick / m; r *= kick / m; }
    }
    l = constrain(l, -0.5f, 0.5f);
    r = constrain(r, -0.5f, 0.5f);
  }

  bool kicking() const { return (int32_t)(_kickUntil - millis()) > 0; }

private:
  float _shape(float x) const {
    if (x == 0) return 0;
    const float a = deadband + (0.5f - deadband) * fminf(fabsf(x), 0.5f) / 0.5f;
    return x > 0 ? a : -a;
  }
  int8_t _sl = 0, _sr = 0;
  uint32_t _kickUntil = 0;
};
