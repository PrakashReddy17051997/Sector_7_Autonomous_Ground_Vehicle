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
//   5. battery:     (2026-10-29) every wheel command * vScale = (vRef / battery V)^vGain, limited to
//                   scaleMin..scaleMax and to +-0.5. The motors run at duty * battery voltage, so the
//                   same command is ~8 % faster on a full pack (12.6 V) than at 11.6 V - where the
//                   dead band, pivot minimum and kick were tuned (logs 5-11: 11.2-11.8 V). A full pack
//                   therefore gets smaller commands (no jump at the breakaway kick), a low pack larger
//                   ones (until the rover's 0.5 maximum - kick and pivot are already at 0.5, so at low
//                   voltage only the moving commands can still grow).
//                   Battery V comes from rover.voltage(): the rover reports it only while it stands
//                   (RoverLink polls when idle) = resting voltage = state of charge; it is smoothed.
//                   Unknown voltage (0 / out of 9..13.5 V) -> scale 1 (as before).
//   6. smooth start (2026-10-29, log_017/018): the kick fires ONLY from a real standstill (both wheels 0
//                   for >= kickStandMs) and never for a pivot or right after one (the 0.4 s surge after
//                   every turn tap was the main jerk: 30 % of the driving at 0.48 instead of 0.36);
//                   rising wheel output is rate limited (accelUp / pivotUp per second), falling is not
//   7. slope:       setPitch(pitch, + = front down): driving DOWN a slope -> forward output * down factor
//                   (1 at 4 deg, -5 %/deg, >= downMin), no kick; UP a slope -> up to +15 %
//   8. boost:       added to a forward command by the route when the rover does not move although it
//                   should (rough ground, grass, a ramp foot) - RouteFollow::boost(), 0 otherwise
// A stop (0 / 0) always passes through immediately. No kick while SafetyGuard limits (tilt).
//
//   shaper.setBattery(rover.voltage());  // every loop pass
//   shaper.apply(l, r, allowKick);       // after SafetyGuard, right before rover.setDrive(l, r)
//   shaper.equivalent()                  // vBatt / vRef: out * this = command "at vRef" (effort)
#include <Arduino.h>
#include <math.h>

class MotorShaper {
public:
  float zero = 0.02f;       // below this a wheel command counts as 0
  float deadband = 0.18f;   // smallest wheel command that moves the rover (tune from the logs)
  float pivotMin = 0.50f;   // turning on the spot: 0.3 never turned, 0.45-0.5 only half the time
  float kick = 0.50f;       // breakaway command
  uint32_t kickMs = 400;    // breakaway duration
  // battery compensation
  bool vComp = true;        // false = old behaviour (no voltage scaling)
  float vRef = 11.6f;       // V at which deadband / pivotMin / kick were tuned
  float vGain = 1.0f;       // 1 = speed proportional to voltage; raise (1.5-2) if 12.6 V still jumps
  float scaleMin = 0.80f, scaleMax = 1.25f;
  // smooth start / slope / boost
  uint32_t kickStandMs = 300;   // kick only after standing this long
  float accelUp = 1.5f;         // forward / backward: output may rise by this per second (0 -> 0.36 in 0.24 s)
  float pivotUp = 6.0f;         // pivot: rise per second (0 -> 0.48 in 80 ms: taps stay short and sharp)
  float downStartDeg = 4.0f, downGain = 0.05f, downMin = 0.55f;   // downhill: less forward output
  float upGain = 0.02f, upMax = 1.15f;                            // uphill: a little more
  float boost = 0;              // set every loop pass (RouteFollow::boost()), added to forward commands
  float pitchZero = 1.5f;       // pitch the IMU reads on flat ground (log_018 standing: 1.6 deg)
  // smoothed (0.5 s): the tall body rocks +-1..3.5 deg at every bump while rolling (log_017/018), the
  // slope is what stays
  void setPitch(float pitchDeg) {
    const uint32_t now = millis();
    const float dt = _pMs ? fminf((now - _pMs) / 1000.0f, 0.5f) : 0.5f;
    _pMs = now;
    _pitch += (dt / (0.5f + dt)) * ((pitchDeg - pitchZero) - _pitch);
  }
  float slopePitch() const { return _pitch; }
  float slopeFactor() const { return _slope; }

  void setBattery(float v) {
    if (v < 9.0f || v > 13.5f) return;                 // 0 = not read yet, or nonsense
    const uint32_t now = millis();
    if (_v == 0) { _v = v; _vMs = now; return; }
    if (now - _vMs >= 1000) {                          // once per second (the rover reports ~1 Hz)
      _v += 0.3f * (v - _v);                           // smooth: one bad reading moves it only 30 %
      _vMs = now;
    }
  }
  float battery() const { return _v; }
  float vScale() const {
    if (!vComp || _v == 0) return 1.0f;
    return constrain(powf(vRef / _v, vGain), scaleMin, scaleMax);
  }
  float equivalent() const { return (!vComp || _v == 0) ? 1.0f : _v / vRef; }

  void apply(float& l, float& r, bool allowKick) {
    const uint32_t now = millis();
    const float dt = _tMs ? fminf((now - _tMs) / 1000.0f, 0.1f) : 0.0f;
    _tMs = now;
    if (fabsf(l) < zero) l = 0;
    if (fabsf(r) < zero) r = 0;
    if (l == 0 && r == 0) {
      if (_standSince == 0) _standSince = now;
      _sl = _sr = 0; _kickUntil = 0; _ol = _or = 0; _slope = 1; _wasPivot = false; return;
    }
    const bool pivot = (l > 0 && r < 0) || (l < 0 && r > 0);
    const bool fwd = l > 0 && r > 0, back = l < 0 && r < 0;
    if (pivot) {
      const float m = fmaxf(fabsf(l), fabsf(r));
      if (m < pivotMin) { l *= pivotMin / m; r *= pivotMin / m; }
    } else {
      if (fwd && boost > 0) { l += boost; r += boost; }
      l = _shape(l);
      r = _shape(r);
    }
    // slope: + pitch = front down; going forward down (or backward up) the rover runs away
    const float down = fwd ? _pitch : (back ? -_pitch : 0.0f);
    _slope = 1.0f;
    if (!pivot && down > downStartDeg) _slope = fmaxf(downMin, 1.0f - downGain * (down - downStartDeg));
    else if (!pivot && down < -downStartDeg) _slope = fminf(upMax, 1.0f + upGain * (-down - downStartDeg));
    l *= _slope; r *= _slope;

    // breakaway kick: only from a real standstill, not for pivots, not downhill
    const int8_t sl = (l > 0) - (l < 0), sr = (r > 0) - (r < 0);
    const bool fromStand = _sl == 0 && _sr == 0 && _standSince != 0 && now - _standSince >= kickStandMs;
    if (allowKick && !pivot && fromStand && down <= downStartDeg) _kickUntil = now + kickMs;
    if (pivot || (sl != _sl && _sl != 0) || (sr != _sr && _sr != 0)) _kickUntil = 0;   // a pivot ends a kick
    _sl = sl; _sr = sr; _standSince = 0;
    const bool kicking = allowKick && (int32_t)(_kickUntil - now) > 0;
    if (kicking) {
      const float m = fmaxf(fabsf(l), fabsf(r));
      if (m > 0 && m < kick) { l *= kick / m; r *= kick / m; }
    }
    const float k = vScale();
    l = constrain(l * k, -0.5f, 0.5f);
    r = constrain(r * k, -0.5f, 0.5f);
    // rate limit on RISING output (a kick is a deliberate step); falling / sign change pass at once
    if (_wasPivot != pivot) { _ol = _or = 0; }                     // pivot <-> drive: both wheels from 0
    _wasPivot = pivot;
    if (!kicking && dt > 0) {
      const float up = (pivot ? pivotUp : accelUp) * dt;
      l = _limit(_ol, l, up);
      r = _limit(_or, r, up);
    }
    _ol = l; _or = r;
  }

  bool kicking() const { return (int32_t)(_kickUntil - millis()) > 0; }

private:
  float _shape(float x) const {
    if (x == 0) return 0;
    const float a = deadband + (0.5f - deadband) * fminf(fabsf(x), 0.5f) / 0.5f;
    return x > 0 ? a : -a;
  }
  static float _limit(float prev, float x, float up) {
    if (x == 0 || (prev > 0) != (x > 0) || prev == 0) {           // from 0 / through 0: start at 0
      if (prev != 0 && (prev > 0) != (x > 0)) prev = 0;
      const float a = fminf(fabsf(x), fabsf(prev) + up);
      return x > 0 ? a : -a;
    }
    if (fabsf(x) <= fabsf(prev)) return x;                          // falling: at once
    const float a = fminf(fabsf(x), fabsf(prev) + up);
    return x > 0 ? a : -a;
  }
  int8_t _sl = 0, _sr = 0;
  float _ol = 0, _or = 0, _pitch = 0, _slope = 1;
  bool _wasPivot = false;
  uint32_t _tMs = 0, _standSince = 1, _pMs = 0;
  float _v = 0;
  uint32_t _vMs = 0;
  uint32_t _kickUntil = 0;
};
