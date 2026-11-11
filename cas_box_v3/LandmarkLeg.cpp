#include "LandmarkLeg.h"

using LineFeat::wrap180;

const char* LandmarkLeg::typeName() const {
  static const char* kNames[] = {"hold", "anchor", "side PID", "edge", "corridor", "entrance", "stop wall", "center"};
  if (!_ph || _p >= _n) return "end";
  const uint8_t t = _ph[_p].type;
  return t <= CENTER ? kNames[t] : "?";
}

LineFeat::Line LandmarkLeg::_line(const uint16_t* scan, float footExp, float dMin, float dMax, float minLen) const {
  LineFeat::Cfg c;
  c.centerDeg = footExp;
  c.halfDeg = 45;
  c.footExp = footExp;
  c.footTol = 30;                  // rover heading error + line course; the course check is below
  c.rMin = fmaxf(0.3f, dMin * 0.9f);
  c.rMax = fminf(11.0f, dMax * 1.45f + 0.5f);   // the sector edges are ~1.4 x farther than d
  c.minLen = minLen;
  LineFeat::Line L = LineFeat::extract(scan, c);
  if (L.ok && (L.d < dMin || L.d > dMax)) L.ok = false;
  return L;
}

// PID on the distance error -> turn (deg). D on the measured distance (no kick when dRef is latched),
// integral only while not saturated (anti-windup) and limited so that ki * I <= sat.
float LandmarkLeg::_pid(float e, float dt, float sat, const LmkPhase& P) {
  float de = 0;
  if (_dPrev >= 0 && dt > 0.05f && dt < 1.0f) de = (_lastD - _dPrev) / dt;
  _deF = 0.7f * _deF + 0.3f * de;
  const float u = P.kp * e + P.ki * _I + P.kd * _deF;
  if (fabsf(u) < sat || (e > 0) != (_I > 0)) _I += e * dt;
  if (P.ki > 0) _I = _clamp(_I, -sat / P.ki, sat / P.ki);
  return _clamp(u, -sat, sat);
}

LandmarkLeg::Out LandmarkLeg::step(const uint16_t* scan, float ry, float ref, float s, float dt, bool tilted) {
  Out o;
  o.yawDes = ref;
  if (!_ph || _done) { o.done = true; o.why = "done"; return o; }

  // ---- phase change by distance (an ENTRANCE waiting for its stop point keeps its phase)
  while (_p < _n && s >= _ph[_p].sTo && !(_ph[_p].type == ENTRANCE && _sStop >= 0)) { _p++; _resetPhase(); }
  if (_p >= _n) { _done = true; o.done = true; o.why = "distance end"; return o; }
  const LmkPhase& P = _ph[_p];
  if (s < P.sFrom || P.type == HOLD) { _dPrev = -1; o.hold = true; o.why = "hold"; return o; }
  if (tilted) { _dPrev = -1; _gap(o, ref, dt); o.why = "on slope: hold"; return o; }

  const LineFeat::Line L = _line(scan, P.footExp, P.dMin, P.dMax, P.minLen);
  // course the line stands for, and is it parallel to the leg course? (a bike rack is not)
  const float lineYaw = wrap180(ry - wrap180(P.footExp - L.foot));
  const float refLine = wrap180(ref + P.lineOff);          // where the line runs when on course
  const bool ok = L.ok && fabsf(wrap180(lineYaw - refLine)) <= P.paraDeg;
  const float courseFromLine = wrap180(lineYaw - P.lineOff);  // the leg course as this line sees it
  const float sideSign = P.footExp > 0 ? 1.0f : -1.0f;   // + yaw = towards a LEFT line
  if (ok) _lastD = L.d;

  switch (P.type) {
    case ANCHOR:
      if (ok) { o.line = true; o.dOff = P.anchorGain * wrap180(refLine - lineYaw); o.why = "anchor"; }
      else o.why = "anchor: no line";
      return o;

    case SIDE: case EDGE: case CORRIDOR: case ENTRANCE: {
      // ENTRANCE: has the facade stepped back / gone see-through? (checked before steering on it)
      if (P.type == ENTRANCE && _sStop < 0) {
        const float df = L.dropFrac();
        if (_dropBase < 0) _dropBase = df; else if (df < _dropBase + 0.2f) _dropBase = 0.9f * _dropBase + 0.1f * df;
        const bool recess = L.ok && _latch > 0 && L.d - _latch >= P.a;
        const bool glass = df >= P.b && df - _dropBase >= 0.25f;
        _evt = (uint8_t)(((_evt << 1) | (recess || glass ? 1 : 0)) & 7);
        if (((_evt & 1) + ((_evt >> 1) & 1) + ((_evt >> 2) & 1)) >= 2) {
          _sStop = s + P.c;
          o.why = recess ? "entrance: facade steps back" : "entrance: glass";
        }
      }
      if (_sStop >= 0 && s >= _sStop) { _done = true; o.done = true; o.why = "entrance + stop distance"; return o; }
      // a reading far off the latched distance is the recess / a gap, not the line to steer on
      const bool steerOk = ok && !(_latch > 0 && fabsf(L.d - _latch) > 0.6f * (P.a > 0 ? P.a : 1.0f) + 0.2f);
      if (!steerOk) { _dPrev = -1; _gap(o, ref, dt); if (!o.why[0]) o.why = "no line: hold"; return o; }
      if (_latch < 0) _latch = L.d;
      const float dSet = (P.type == SIDE && P.dRef > 0) ? P.dRef : (P.dRef > 0 ? P.dRef : _latch);
      const float e = L.d - dSet;
      _turn = _pid(e, dt, P.satDeg, P);
      _dPrev = L.d;
      _lastE = e;
      o.yawDes = wrap180(courseFromLine + sideSign * _turn);
      o.line = true;
      _onHit(o, ref);
      if (P.anchorGain > 0) o.dOff = P.anchorGain * wrap180(refLine - lineYaw);
      if (!o.why[0]) o.why = P.type == SIDE ? "side PID" : "edge keep";
      if (P.type == CORRIDOR) {   // other side: railing / facade closer than b, width in [a, a2]
        // v3 (10.11., Arjun v12 after campus_11): the parapet line 0.35..b on the other side, and the width checked
        // only when the facade side has a line (it may be open there); 3 of the last 5 scans (4 in a row missed it)
        const LineFeat::Line R = _line(scan, -P.footExp, 0.35f, P.b, 0.6f);
        const float w = L.d + R.d;
        const bool hitC = R.ok && (!L.ok || (w >= P.a && w <= P.a2));
        _evtN = (uint8_t)(((_evtN << 1) | (hitC ? 1 : 0)) & 31);
        uint8_t k = 0; for (uint8_t q = 0; q < 5; q++) k += (_evtN >> q) & 1;
        if (k >= 3) { _p++; _resetPhase(); o.why = "corridor found"; }
      }
      return o;
    }

    case CENTER: {
      // left line = this phase's line (footExp +90, dRef = taught left distance), right line looked up here
      // (taught right distance in a). Both: middle of the corridor; one: keep its taught distance; none: hold.
      const LineFeat::Line R = _line(scan, -90.0f, 0.3f, P.a + 1.5f, P.minLen);
      const float rYaw = wrap180(ry - wrap180(-90.0f - R.foot));
      const bool rOk = R.ok && fabsf(wrap180(rYaw - refLine)) <= P.paraDeg;
      float e; float course;
      if (ok && rOk) {                       // + e = closer to the right wall than the middle -> steer left
        e = 0.5f * ((L.d - R.d) - (P.dRef - P.a));    // taught asymmetry kept (the taught track was not centred)
        course = wrap180(courseFromLine + 0.5f * wrap180(wrap180(rYaw - P.lineOff) - courseFromLine));   // mean of both
        o.why = "center: both walls";
      } else if (ok) {
        e = L.d - P.dRef; course = courseFromLine; o.why = "center: left wall";
      } else if (rOk) {
        e = P.a - R.d; course = wrap180(rYaw - P.lineOff); o.why = "center: right wall";
      } else { _dPrev = -1; _gap(o, ref, dt); o.why = "center: no wall"; return o; }
      _lastD = e + 100.0f;                   // (_pid's D term takes _dPrev >= 0 as "has a previous value")
      _turn = _pid(e, dt, P.satDeg, P);      // + = turn left (towards the left wall / away from the right)
      _dPrev = e + 100.0f;
      _lastE = e;
      o.yawDes = wrap180(course + _turn);
      o.line = true;
      _onHit(o, ref);
      if (P.anchorGain > 0) o.dOff = P.anchorGain * wrap180(ref - course);
      return o;
    }

    case STOP: {
      // crowd rule: long + straight + 3 of 5 scans + fixed in space (d + s constant)
      _hit = (uint8_t)(((_hit << 1) | (ok ? 1 : 0)) & 31);
      if (ok) { _w[_wN % 5] = L.d + s; _wN++; }
      uint8_t k = 0; for (uint8_t b = 0; b < 5; b++) k += (_hit >> b) & 1;
      bool fixed = false;
      if (_wN >= 3) {
        const uint8_t m = _wN < 5 ? _wN : 5;
        float lo = 1e9f, hi = -1e9f;
        for (uint8_t q = 0; q < m; q++) { lo = fminf(lo, _w[q]); hi = fmaxf(hi, _w[q]); }
        fixed = hi - lo < 0.4f;
      }
      if (k >= 3 && fixed) {
        o.line = ok;
        if (ok) {
          o.yawDes = courseFromLine;          // the course as the wall ahead sees it
          _onHit(o, ref);
          if (P.anchorGain > 0) o.dOff = P.anchorGain * wrap180(refLine - lineYaw);
          if (L.d <= P.dRef) { _done = true; o.done = true; o.why = "end wall reached"; return o; }
        }
        if (!ok) _gap(o, ref, dt);
        o.why = "end wall confirmed";
      } else {
        _gap(o, ref, dt);
        o.why = ok ? "end wall?" : "looking for end wall";
      }
      return o;
    }
  }
  return o;
}
