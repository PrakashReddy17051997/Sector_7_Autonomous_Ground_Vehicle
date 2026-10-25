#include "WallFollow.h"

static const char* kWallState[] = {"off", "follow", "bridge", "lost (wall ended)", "avoid"};

// ---------------------------------------------------------------------------------------------
// (the "Wall following" hold-to-drive card was removed; WallFollow is used by Mission)
// ---------------------------------------------------------------------------------------------
// (web card removed 2026-10-23: the page now has only manual drive + the mission card)

// ---------------------------------------------------------------------------------------------

bool WallFollow::begin(AsyncWebServer* server) {
  if (!server) return false;
  server->on("/wall/status", HTTP_GET, [this](AsyncWebServerRequest* req) {
    req->send(200, "application/json", _statusJson());
  });
  // /wall/left|right|off|plus|minus (hold-to-drive card) removed 2026-10-24: flash was full.
  return true;
}

// Principal-axis line fit of the points (x[i], y[i]). Returns false if degenerate.
static bool fitLine(const float* x, const float* y, int n, float& mx, float& my, float& ux, float& uy) {
  if (n < 2) return false;
  mx = my = 0;
  for (int i = 0; i < n; i++) { mx += x[i]; my += y[i]; }
  mx /= n; my /= n;
  float sxx = 0, syy = 0, sxy = 0;
  for (int i = 0; i < n; i++) {
    const float dx = x[i] - mx, dy = y[i] - my;
    sxx += dx * dx; syy += dy * dy; sxy += dx * dy;
  }
  const float phi = 0.5f * atan2f(2.0f * sxy, sxx - syy);  // direction of largest spread
  ux = cosf(phi);
  uy = sinf(phi);
  if (ux < 0) { ux = -ux; uy = -uy; }                      // line direction points forward
  return true;
}

// One wall fit on one side. "Side frame": x forward, y towards the wall side (so the wall is at
// y > 0 for both sides). Angle < 0 means the nose points towards the wall.
WallFollow::Wall WallFollow::measure(const Sensors& s, Side side, const Wall* prior) const {
  Wall w = {false, 0, 0, 0, 0};
  if (!s.lidarOk()) return w;
  const int n = Sensors::kLidarPoints;
  const float degPer = 360.0f / n;
  const float center = (side == LEFT) ? 270.0f : 90.0f;
  float px[Sensors::kLidarPoints], py[Sensors::kLidarPoints], off[Sensors::kLidarPoints];
  int m = 0;
  for (int i = 0; i < n; i++) {
    const uint16_t cm = s.lidarCm(i);
    if (cm < 20 || cm > 300) continue;  // 0 = no return; beyond 3 m too sparse outdoors
    const float a = (i * degPer + degPer / 2);
    float o = fabsf(fmodf(a - center + 540.0f, 360.0f) - 180.0f);  // angle from side direction
    if (o > 60.0f) continue;
    const float r = cm / 100.0f, ar = a * DEG_TO_RAD;
    px[m] = r * cosf(ar);
    py[m] = r * sinf(ar) * (side == LEFT ? -1.0f : 1.0f);  // y right -> y towards wall
    off[m] = o;
    m++;
  }
  float qx[Sensors::kLidarPoints], qy[Sensors::kLidarPoints];
  int k = 0;
  float mx, my, ux, uy;
  if (prior && prior->ok) {
    // 1a) tracking: start from the points near the last wall line. Points ahead of and behind
    //     the rover still count, so a door or a gap in front of the rover does not lose the wall.
    const float th = prior->angleDeg * DEG_TO_RAD, nx = -sinf(th), ny = cosf(th);
    for (int i = 0; i < m; i++) if (fabsf(nx * px[i] + ny * py[i] - prior->distM) < 0.15f) { qx[k] = px[i]; qy[k] = py[i]; k++; }
    if (k < 5) return w;
  } else {
    // 1b) no prior: core points around the side direction
    for (int i = 0; i < m; i++) if (off[i] <= 20.0f) { qx[k] = px[i]; qy[k] = py[i]; k++; }
    if (k < 4) return w;
  }
  if (!fitLine(qx, qy, k, mx, my, ux, uy)) return w;
  // 2) all points within 10 cm of the line, refit twice
  for (int it = 0; it < 2; it++) {
    k = 0;
    for (int i = 0; i < m; i++) {
      if (fabsf(-uy * (px[i] - mx) + ux * (py[i] - my)) < 0.10f) { qx[k] = px[i]; qy[k] = py[i]; k++; }
    }
    if (k < 5) return w;
    if (!fitLine(qx, qy, k, mx, my, ux, uy)) return w;
  }
  float sMin = 1e9f, sMax = -1e9f, sq = 0, xMax = -1e9f;
  for (int i = 0; i < k; i++) {
    xMax = fmaxf(xMax, qx[i]);
    const float along = qx[i] * ux + qy[i] * uy;
    sMin = fminf(sMin, along);
    sMax = fmaxf(sMax, along);
    const float res = -uy * (qx[i] - mx) + ux * (qy[i] - my);
    sq += res * res;
  }
  if (sMax - sMin < 0.6f) return w;  // too short to be a wall
  if (xMax < -0.1f) return w;         // wall only behind the rover = it has ended
  w.ok = true;
  w.distM = fabsf(-uy * mx + ux * my);
  w.angleDeg = atan2f(uy, ux) * RAD_TO_DEG;
  w.rmsM = sqrtf(sq / k);
  w.points = (uint8_t)k;
  return w;
}

// Starts following the wall on this side at the distance it has now. False if no wall is seen.
bool WallFollow::start(const Sensors& s, Side side) {
  const Wall w = measure(s, side, nullptr);
  if (!w.ok) {
    _state = OFF;
    strlcpy(_msg, side == LEFT ? "no wall seen on the left" : "no wall seen on the right", sizeof(_msg));
    return false;
  }
  _side = side;
  _dSet = _clamp(w.distM, 0.35f, 2.0f);  // keep the distance it has now
  _wall = w;
  _lastGoodD = w.distM;
  _lastGoodMs = millis();
  _state = FOLLOW;
  return true;
}

void WallFollow::pollRequests(const Sensors& s) {
  if (_reqOff) { _reqOff = false; _state = OFF; strlcpy(_msg, "stopped", sizeof(_msg)); }
  if (_reqAdjCm) {
    _dSet = _clamp(_dSet + _reqAdjCm / 100.0f, 0.35f, 2.0f);
    _reqAdjCm = 0;
  }
  if (_reqLeft || _reqRight) {
    const Side side = _reqLeft ? LEFT : RIGHT;
    _reqLeft = _reqRight = false;
    if (start(s, side)) strlcpy(_msg, "hold UP to drive", sizeof(_msg));
  }
  // what the LiDAR sees on both sides right now (for the page, also when off)
  static uint32_t lastLive = 0;
  if (millis() - lastLive >= 250) {
    lastLive = millis();
    const Wall a = measure(s, LEFT, nullptr), b = measure(s, RIGHT, nullptr);
    _liveL = a.ok ? a.distM : -1;
    _liveR = b.ok ? b.distM : -1;
  }
}

// Free length (m) of a corridor 2*halfWidthM wide in direction phiDeg (+ = right of the nose),
// up to lookAheadM. Every LiDAR point inside the corridor shortens it.
float WallFollow::corridorM(const Sensors& s, float phiDeg) const {
  const int n = Sensors::kLidarPoints;
  float best = lookAheadM;
  for (int i = 0; i < n; i++) {
    const uint16_t cm = s.lidarCm(i);
    if (!cm) continue;
    const float a = (i * 360.0f / n + 180.0f / n - phiDeg) * DEG_TO_RAD, r = cm / 100.0f;
    const float along = r * cosf(a), lat = r * sinf(a);
    if (along > 0 && along < best && fabsf(lat) < halfWidthM) best = along;
  }
  return best;
}

void WallFollow::steer(const Sensors& s, float padL, float padR, uint32_t padAge, float& l, float& r) {
  const bool fresh = padAge <= 400;
  const bool hold = fresh && padL > 0 && padR > 0 && fabsf(padL - padR) < 0.001f;  // UP key held
  if (!hold) { l = fresh ? padL : 0; r = fresh ? padR : 0; return; }
  const float spd = padL;
  l = r = 0;
  if (_state == LOST || _state == OFF) return;
  const uint32_t now = millis();

  // ---- wall measurement: track the last line (if recent), plausibility gate on the distance
  const bool recent = _lastGoodD > 0 && now - _lastGoodMs < 2000;
  Wall w = measure(s, _side, recent ? &_wall : nullptr);
  if (w.ok && _lastGoodD > 0 && fabsf(w.distM - _lastGoodD) > jumpM) w.ok = false;  // other object
  if (w.ok) { _wall = w; _lastGoodD = w.distM; _lastGoodMs = now; }

  // ---- obstacle ahead? (corridor as wide as the rover + margin, LiDAR + front ultrasonic)
  _frontM = corridorM(s, 0);
  const int us = s.usCm(0);
  if (us > 2 && us < Sensors::kUsMaxCm) _frontM = fminf(_frontM, us / 100.0f);
  if (_state != AVOID && _frontM < avoidEnterM) { _state = AVOID; strlcpy(_msg, "obstacle - avoiding", sizeof(_msg)); }

  float v = 0, wRight = 0;
  if (_state == AVOID) {
    // VFH-lite: try directions -60..+60 deg; take the one closest to straight with >= 1.2 m free.
    // allowAwayFromWall = false: never steer towards the open side (edge of the raised path).
    float bestPhi = 0, bestClr = -1, maxClr = -1, maxPhi = 0;
    for (int p = -60; p <= 60; p += 10) {
      const bool away = (_side == LEFT) ? (p > 0) : (p < 0);
      if (away && !allowAwayFromWall) continue;
      const float c = corridorM(s, p);
      if (c > maxClr) { maxClr = c; maxPhi = p; }
      if (c >= avoidFreeM && (bestClr < 0 || abs(p) < fabsf(bestPhi))) { bestPhi = p; bestClr = c; }
    }
    if (bestClr < 0) { bestPhi = maxPhi; bestClr = maxClr; }
    if (bestPhi == 0 && _frontM > avoidFreeM) {  // way ahead is free again: back to the wall
      _state = FOLLOW;
      _lastGoodD = -1;  // no jump gate: the distance may have changed while avoiding
      strlcpy(_msg, "obstacle passed", sizeof(_msg));
    } else {
      v = spd * _clamp((bestClr - frontMinM) / (frontSlowM - frontMinM), 0.0f, 1.0f) *
          fmaxf(0.3f, cosf(bestPhi * DEG_TO_RAD));
      wRight = _clamp(0.01f * bestPhi, -spd, spd);  // full turn authority while avoiding
      if (bestClr < frontMinM) strlcpy(_msg, "blocked - waiting", sizeof(_msg));
    }
  }
  if (_state != AVOID) {
    if (w.ok) _state = FOLLOW;
    else if (now - _lastGoodMs < bridgeMs) _state = BRIDGE;  // gap in the wall: straight on
    else {
      _state = LOST;  // end of the wall (e.g. end of the B facade): stop
      strlcpy(_msg, "wall ended - stopped", sizeof(_msg));
      return;
    }
    float wToward = 0;  // + = turn towards the wall
    v = spd;
    if (_state == FOLLOW) {
      const float thetaDes = -_clamp(kDist * (_wall.distM - _dSet), -maxApproachDeg, maxApproachDeg);
      wToward = kAngle * (_wall.angleDeg - thetaDes);
    } else {
      v *= 0.7f;
    }
    wToward = _clamp(wToward, -turnFrac * spd, turnFrac * spd);
    wRight = (_side == RIGHT) ? wToward : -wToward;
    v *= _clamp((_frontM - frontMinM) / (frontSlowM - frontMinM), 0.0f, 1.0f);  // slow near obstacles
  }
  _wCmd = wRight;
  l = _clamp(v + wRight, -0.5f, 0.5f);
  r = _clamp(v - wRight, -0.5f, 0.5f);
}

String WallFollow::statusText() const {
  String t = kWallState[_state];
  if (_state != OFF) { t += _side == LEFT ? " L " : " R "; t += String(_wall.distM, 2); t += "/"; t += String(_dSet, 2); }
  return t;
}

String WallFollow::_statusJson() const {
  String j;
  j.reserve(300);
  j += "{\"state\":\""; j += kWallState[_state];
  j += "\",\"side\":\""; j += _side == LEFT ? "left" : "right";
  j += "\",\"set\":"; j += String(_dSet, 2);
  j += ",\"d\":"; j += _wall.ok ? String(_wall.distM, 2) : String("-1");
  j += ",\"ang\":"; j += String(_wall.angleDeg, 1);
  j += ",\"rms\":"; j += String(_wall.rmsM * 100.0f, 1);
  j += ",\"pts\":"; j += (unsigned)_wall.points;
  j += ",\"front\":"; j += String(_frontM, 2);
  j += ",\"w\":"; j += String(_wCmd, 2);
  j += ",\"liveL\":"; j += String(_liveL, 2);
  j += ",\"liveR\":"; j += String(_liveR, 2);
  j += ",\"msg\":\""; j += _msg; j += "\"}";
  return j;
}
