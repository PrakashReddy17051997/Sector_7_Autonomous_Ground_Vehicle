#pragma once
// HeadingHold - manual driving / TEACHING DRIVE: while you hold the FORWARD key, the rover keeps the
// heading it had when you pressed it (gyro PD, gentle wheel difference). Turns (left / right keys)
// and reversing are passed on unchanged, a pure turn on the spot is lifted to the soft pivot command.
// Adapted 2026-11-04 from Kethan's field-tested steerForward() (cas_box_kethan): straight legs in the
// drive log instead of the hand-steered zig-zag, so make_legs.py / localpath.py get clean legs.
// (His left-wall guidance is NOT taken over: it pulls the rover to 60 cm from the wall - the taught
//  distance of our wall legs must be the one you drive.)
//
//   e = wrap(target - yaw)                        (+ = target is to the left)
//   c = hKp * e - hKd * yawRate                   e inside +-0.5 deg -> no P part
//   |c| <= min(corrMax, corrFrac * speed),  c changes by <= corrRate per second
//   l = v - c,  r = v + c
// No checked IMU sample (Sensors::imuOk) while forward is held -> 0 (no blind straight driving).
//
//   hold.apply(sensors, l, r);   // manual only, after the pad timeout, before SafetyGuard
#include <Arduino.h>
#include <math.h>
#include "Sensors.h"

class HeadingHold {
public:
  float hKp = 0.045f, hKd = 0.0015f;              // same gains as RouteFollow's rolling PD
  float corrMax = 0.10f, corrFrac = 0.40f, corrRate = 0.30f;
  float pivotCmd = 0.35f;                         // manual turn on the spot (0.3 and less does not turn)

  void apply(const Sensors& s, float& l, float& r) {
    const uint32_t now = millis();
    const bool forward = l > 0 && r > 0 && fabsf(l - r) < 0.0001f;
    const bool pivot = (l > 0 && r < 0) || (l < 0 && r > 0);
    if (!forward) {
      _holding = false; _c = 0; _ms = 0;
      if (pivot) { const float d = r > 0 ? 1.0f : -1.0f; l = -d * pivotCmd; r = d * pivotCmd; }
      return;
    }
    if (!s.imuOk()) { _holding = false; _c = 0; l = r = 0; return; }
    const float yaw = s.yawNowDeg();
    if (!_holding) { _holding = true; _target = _prev = yaw; _rate = 0; _c = 0; _ms = now; }
    if (now - _ms >= 10) {                         // up to 100 Hz
      const float dt = (now - _ms) / 1000.0f;
      _ms = now;
      if (dt < 0.1f) {
        const float keep = powf(0.7f, dt / 0.02f);
        _rate = keep * _rate + (1.0f - keep) * _wrap(yaw - _prev) / dt;
      } else {
        _rate = 0;
      }
      _prev = yaw;
      _err = _wrap(_target - yaw);
      const float lim = fminf(corrMax, corrFrac * l);
      const float want = constrain((fabsf(_err) > 0.5f ? hKp * _err : 0.0f) - hKd * _rate, -lim, lim);
      if (dt >= 0.1f) _c = 0;
      else _c += constrain(want - _c, -corrRate * dt, corrRate * dt);
      _c = constrain(_c, -lim, lim);
    }
    const float v = l;
    l = v - _c;
    r = v + _c;
  }

  bool holding() const { return _holding; }
  float targetDeg() const { return _target; }
  float errorDeg() const { return _holding ? _err : 0; }

private:
  static float _wrap(float a) { a = fmodf(a + 180.0f, 360.0f); if (a < 0) a += 360.0f; return a - 180.0f; }
  bool _holding = false;
  float _target = 0, _prev = 0, _rate = 0, _c = 0, _err = 0;
  uint32_t _ms = 0;
};
