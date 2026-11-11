#pragma once
// MotorShaper - last stage before rover.setDrive(): makes every change of the wheel commands gradual.
//
// 2026-11-05 (step 2): works on SPEED and TURN, not per wheel:
//   v = (l + r) / 2  forward speed      w = (r - l) / 2  turn (+ = left)
//   speed   from standstill: first step vStart (0.05), then LINEAR to the target in riseS (2.5 s),
//           e.g. 0.05 -> 0.30 in 2.5 s. Slower / stop: soft, vStop per second (0.30 -> 0 in 0.5 s).
//           hard = true (SafetyGuard stop: obstacle very close, tilt): the speed drops at once.
//           A change of direction passes through 0.
//   11.11. (v3): speed ramps are now S-SHAPED and take accelS (2 s) to speed up, decelS (2 s) to slow down / stop -
//           manual driving and the mission alike (stop for an obstacle wait, a turn, a decision, localisation).
//           ramp(t) = from + (to - from) * s(t / T),  s(x) = x^2 (3 - 2x): gentle at both ends, no jerk.
//           T = accelS / decelS for a change of >= fullStep (0.25), shorter for small changes (>= 30 %).
//           A new target restarts the ramp from the current speed. No first step (vStart) any more.
//           hard = true (SafetyGuard: obstacle < 30 cm, tilt) still stops AT ONCE.
//   turn    rises by at most wUp per second (soft pivot 0 -> 0.35 in 1 s), falls by wDown per second.
//           The rolling heading correction (<= 0.10, RouteFollow / Kethan) is already gradual.
//   So a heading correction never resets the forward speed, and before a pivot the speed ramps down
//   while the turn ramps up (a smooth curve into the pivot instead of stop - spin - go).
// Removed for good (05.11.): the 0.5 start kick, the dead band lift, the pivot minimum, battery scaling,
// the slope factor - no hidden changes of the command.
// Kept: the route boost (RouteFollow: no progress for 2 s while rolling -> + max 0.05).
//
//   shaper.setBattery(v); shaper.setPitch(p); shaper.boost = ...;   // every loop pass
//   shaper.apply(l, r, hard);                                       // right before rover.setDrive(l, r)
//   shaper.equivalent()   vBatt / vRef for the route effort (it does NOT change the command)
#include <Arduino.h>
#include <math.h>

class MotorShaper {
public:
  float accelS = 2.0f;      // v3: speed up in this time (S-curve)
  float decelS = 2.0f;      // v3: slow down / stop in this time (S-curve)
  // v4 (11.11.): FIRM stop - the route sets firm while it stops for an obstacle / backs up. With the 2 s S-curve
  // the rover rolled ~0.3 m after the route asked to stop and hit a new obstacle that was seen at 0.45 m
  // (tools/v4_test/obstacle_sim, scenario 1). Firm = slow down in firmDecelS (S-curve too, ~0.1 m from 0.30).
  float firmDecelS = 0.6f;
  bool firm = false;
  float fullStep = 0.25f;   // a speed change this big takes the full time; smaller ones take less (min 30 %)
  float vStart = 0.05f;     // (unused since v3)
  float riseS = 2.5f;       // vStart -> target speed in this time (linear)
  float vStop = 0.60f;      // soft slowing / stopping: speed change per second
  float wUp = 0.35f;        // turn: rise per second
  float wDown = 1.00f;      // turn: fall per second
  float zero = 0.01f;       // output below this -> 0
  float boost = 0;          // set every loop pass (RouteFollow::boost()), added to forward commands
  float boostMax = 0.05f;
  bool vComp = false;       // battery: only measured (route effort), the command is NOT scaled
  float vRef = 11.6f;

  void setPitch(float) {}                       // slope factor removed (kept for the interface)
  float slopePitch() const { return 0; }
  float slopeFactor() const { return 1.0f; }

  void setBattery(float v) {
    if (v < 9.0f || v > 13.5f) return;
    const uint32_t now = millis();
    if (_vb == 0) { _vb = v; _vbMs = now; return; }
    if (now - _vbMs >= 1000) { _vb += 0.3f * (v - _vb); _vbMs = now; }
  }
  float battery() const { return _vb; }
  float vScale() const { return 1.0f; }
  float equivalent() const { return _vb == 0 ? 1.0f : _vb / vRef; }

  void apply(float& l, float& r, bool hard) {
    const uint32_t now = millis();
    float dt = _tMs ? (now - _tMs) / 1000.0f : 0.0f;
    _tMs = now;
    if (dt > 0.1f) dt = 0.1f;
    float vT = 0.5f * (l + r), wT = 0.5f * (r - l);
    const bool pivot = (l > 0 && r < 0) || (l < 0 && r > 0);
    if (vT > 0 && !pivot && boost > 0) vT += fminf(boost, boostMax);
    vT = constrain(vT, -0.5f, 0.5f);
    wT = constrain(wT, -0.5f, 0.5f);

    // ---- speed (v3: S-curve ramps, accelS up / decelS down)
    if (_v != 0 && vT != 0 && (_v > 0) != (vT > 0)) vT = 0;            // reverse: through 0 first
    if (hard) {
      _v = vT; _rTo = vT; _rFrom = vT; _rT = 0;                         // guard stop: at once
    } else {
      if (fabsf(vT - _rTo) > 0.004f || (firm && fabsf(vT) < fabsf(_v) && _rDur > firmDecelS + 0.01f && _rT < _rDur)) {   // new target (or firm now): ramp from where we are
        _rFrom = _v; _rTo = vT; _rT = 0;
        const float base = fabsf(vT) > fabsf(_v) ? accelS : (firm ? firmDecelS : decelS);
        _rDur = base * constrain(fabsf(vT - _v) / fullStep, 0.3f, 1.0f);
      }
      if (_rT < _rDur) {
        _rT += dt;
        const float x = constrain(_rT / _rDur, 0.0f, 1.0f);
        _v = _rFrom + (_rTo - _rFrom) * x * x * (3.0f - 2.0f * x);
      } else {
        _v = _rTo;
      }
    }
    if (fabsf(_v) < 0.001f) _v = 0;

    // ---- turn
    float wt = wT;
    if (_w != 0 && wt != 0 && (_w > 0) != (wt > 0)) wt = 0;            // other direction: through 0
    const float wr = (fabsf(wt) > fabsf(_w) ? wUp : wDown) * dt;
    _w += constrain(wt - _w, -wr, wr);
    if (fabsf(_w) < 0.001f) _w = 0;

    l = constrain(_v - _w, -0.5f, 0.5f);
    r = constrain(_v + _w, -0.5f, 0.5f);
    if (fabsf(l) < zero) l = 0;
    if (fabsf(r) < zero) r = 0;
  }

  float v() const { return _v; }
  float w() const { return _w; }
  bool kicking() const { return false; }

private:
  float _v = 0, _w = 0;
  float _rFrom = 0, _rTo = 0, _rT = 0, _rDur = 0;   // current speed ramp
  uint32_t _tMs = 0;
  float _vb = 0;
  uint32_t _vbMs = 0;
};
