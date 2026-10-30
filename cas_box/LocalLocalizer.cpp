#include "LocalLocalizer.h"
#include <math.h>
#include <string.h>
#include "LocalPath.h"

using namespace LocalPathMap;

static const float kRes = 0.10f, kStep = 0.25f;
static const float kRmax = 10.0f;   // = localpath.py RMAX (open areas: half the points are 6-11 m away)
static const int8_t kDthWide[] = {-20, -16, -12, -8, -4, 0, 4, 8, 12, 16, 20};
static const int8_t kDthCoarse[] = {-4, -2, 0, 2, 4};
static const int8_t kDthFine[] = {-1, 0, 1};
static const int8_t kDthZero[] = {0};
static const float kDeg = 0.01745329252f;

static inline float wrapPi(float a) {
  a = fmodf(a + 3.14159265f, 6.28318531f);
  if (a < 0) a += 6.28318531f;
  return a - 3.14159265f;
}

bool LocalLocalizer::hasMap() const { return kLegs > 0; }
uint8_t LocalLocalizer::legs() const { return kLegs; }
float LocalLocalizer::sEnd() const { return kLeg[_leg].sEnd; }

void LocalLocalizer::begin(uint8_t leg) {
  _leg = leg < kLegs ? leg : 0;
  _x = _y = _th = 0;
  _pi = 0;
  _first = true;
  _atEnd = false;
  _lost = 0;
  _ok = false;
  _score = 0;
  _off = 0;
  _histN = 0;
  _histI = 0;
  _buildWp();
}

// Douglas-Peucker on the taught path of this leg: keep the point farthest from the chord while it is
// > 12 cm off, recursively (explicit stack, no recursion). Result: corner indices in path order.
void LocalLocalizer::_buildWp() {
  const Leg& L = kLeg[_leg];
  const int n = L.nPath;
  _nWp = 0;
  if (n < 2) { _wp[_nWp++] = 0; return; }
  static uint8_t keep[512];
  const int m = n < 512 ? n : 512;
  memset(keep, 0, m);
  keep[0] = keep[m - 1] = 1;
  int16_t st[64];
  int sp = 0;
  st[sp++] = 0; st[sp++] = (int16_t)(m - 1);
  while (sp >= 2) {
    const int b = st[--sp], a = st[--sp];
    if (b - a < 2) continue;
    const float ax = L.path[2 * a] * 0.01f, ay = L.path[2 * a + 1] * 0.01f;
    const float bx = L.path[2 * b] * 0.01f, by = L.path[2 * b + 1] * 0.01f;
    const float dx = bx - ax, dy = by - ay, len = sqrtf(dx * dx + dy * dy);
    float worst = 0;
    int wk = -1;
    for (int k = a + 1; k < b; k++) {
      const float px = L.path[2 * k] * 0.01f - ax, py = L.path[2 * k + 1] * 0.01f - ay;
      const float d = len > 1e-3f ? fabsf(dx * py - dy * px) / len : sqrtf(px * px + py * py);
      if (d > worst) { worst = d; wk = k; }
    }
    if (wk >= 0 && worst > 0.12f && sp <= 60) {
      keep[wk] = 1;
      st[sp++] = (int16_t)a; st[sp++] = (int16_t)wk;
      st[sp++] = (int16_t)wk; st[sp++] = (int16_t)b;
    }
  }
  for (int k = 0; k < m && _nWp < kMaxWp; k++) if (keep[k]) _wp[_nWp++] = (uint16_t)k;
  if (_wp[_nWp - 1] != m - 1) _wp[_nWp - 1] = (uint16_t)(m - 1);   // always end at the leg end
}

uint16_t LocalLocalizer::_wpAt(float minAheadM) const {
  const int need = (int)_pi + (int)(minAheadM / kStep + 0.5f);
  for (uint8_t k = 0; k < _nWp; k++) if (_wp[k] >= need) return _wp[k];
  return _nWp ? _wp[_nWp - 1] : 0;
}

// carrot on the STRAIGHT-LINE path: the polyline through the corners (waypoints). Project the pose on
// the straight piece it is on, then walk lookM along the straight pieces (round the next corner if the
// piece ends first). The carrot is always on a straight line between two corners -> a steady heading
// on straight pieces (the scan-match noise of the taught path is gone), and it cuts the corners a bit.
void LocalLocalizer::waypoint(float lookM, float& lx, float& ly) const {
  const Leg& L = kLeg[_leg];
  if (_nWp < 2) { lx = L.path[0] * 0.01f; ly = L.path[1] * 0.01f; return; }
  uint8_t k = 0;                                   // piece k = corner k -> corner k+1, holds _pi
  while (k + 2 < _nWp && _wp[k + 1] <= _pi) k++;
  float ax = L.path[2 * _wp[k]] * 0.01f, ay = L.path[2 * _wp[k] + 1] * 0.01f;
  float bx = L.path[2 * _wp[k + 1]] * 0.01f, by = L.path[2 * _wp[k + 1] + 1] * 0.01f;
  float dx = bx - ax, dy = by - ay, len = sqrtf(dx * dx + dy * dy);
  float t = len > 1e-3f ? ((_x - ax) * dx + (_y - ay) * dy) / len : 0;   // along the piece (m)
  if (t < 0) t = 0;
  float rest = lookM;
  for (;;) {
    if (t + rest <= len || k + 2 >= _nWp) {
      // last piece: the carrot goes PAST the leg end along the last direction (never closer than
      // lookM). log_017: the carrot sat on the end point under the rover -> heading to it swung
      // 60-140 deg -> a big wrong turn at every leg hand-over.
      const float u = len > 1e-3f ? (t + rest) / len : 1.0f;
      lx = ax + u * dx; ly = ay + u * dy;
      return;
    }
    rest -= (len - t);
    k++;
    ax = bx; ay = by;
    bx = L.path[2 * _wp[k + 1]] * 0.01f; by = L.path[2 * _wp[k + 1] + 1] * 0.01f;
    dx = bx - ax; dy = by - ay; len = sqrtf(dx * dx + dy * dy); t = 0;
  }
}

float LocalLocalizer::headingErrorToWaypoint(float lookM) const {
  float tx, ty;
  waypoint(lookM, tx, ty);
  const float dx = tx - _x, dy = ty - _y;
  if (dx * dx + dy * dy < 0.04f) return 0;     // on top of the leg end: hold the heading
  return wrapPi(atan2f(dy, dx) - _th);
}

void LocalLocalizer::waypointGlobal(float minAheadM, float& gx, float& gy) const {
  const Leg& L = kLeg[_leg];
  float lx, ly;
  waypoint(minAheadM, lx, ly);
  const float c = cosf(L.origTh), s = sinf(L.origTh);
  gx = L.origX + c * lx - s * ly;
  gy = L.origY + s * lx + c * ly;
}

// sum of the 4-bit likelihood under the points / 15 / n
float LocalLocalizer::scoreAt(const float* px, const float* py, int n, float x, float y, float th) const {
  const Leg& L = kLeg[_leg];
  const float c = cosf(th), s = sinf(th), lx = L.loX * 0.01f, ly = L.loY * 0.01f;
  int sum = 0;
  for (int k = 0; k < n; k++) {
    const float wx = x + c * px[k] - s * py[k], wy = y + s * px[k] + c * py[k];
    const int i = (int)((wx - lx) / kRes), j = (int)((wy - ly) / kRes);
    if (i < 0 || i >= L.nx || j < 0 || j >= L.ny) continue;
    const uint32_t idx = (uint32_t)i * L.ny + j;
    const uint8_t b = L.grid[idx >> 1];
    sum += (idx & 1) ? (b >> 4) : (b & 15);
  }
  return n ? (sum / 15.0f) / n : 0;
}

// grid search around (cx, cy, ct): offsets -nh..nh * step in x and y, headings ct + dth
LocalLocalizer::Best LocalLocalizer::_search(const float* px, const float* py, int n, float cx, float cy,
                                             float ct, float step, int nh, const int8_t* dthDeg, int nth,
                                             float prx, float pry, float prth, float lam, float thW) const {
  Best b{cx, cy, ct, -1e9f};
  for (int t = 0; t < nth; t++) {
    const float th = ct + dthDeg[t] * kDeg;
    const float pth = thW * (th - prth) * (th - prth);
    for (int ix = -nh; ix <= nh; ix++) {
      const float x = cx + ix * step;
      for (int iy = -nh; iy <= nh; iy++) {
        const float y = cy + iy * step;
        const float sc = scoreAt(px, py, n, x, y, th) - lam * ((x - prx) * (x - prx) + (y - pry) * (y - pry)) - pth;
        if (sc > b.sc) b = Best{x, y, th, sc};
      }
    }
  }
  return b;
}

void LocalLocalizer::_match(const float* px, const float* py, int n, bool wide) {
  const float prx = _x, pry = _y, prth = _th;
  float cx = _x, cy = _y, ct = _th;
  Best b;
  if (wide) {
    b = _search(px, py, n, cx, cy, ct, 0.10f, 15, kDthWide, 11, prx, pry, prth, 1.0f, 0.3f);
    cx = b.x; cy = b.y; ct = b.th;
  }
  // heading: the gyro is good to < 1 deg over seconds; log_023 leg 1: weak matches (score 0.5-0.7)
  // turned the heading away from the gyro by 4 deg per scan, -30 deg in 3 s -> wrong path direction.
  // So a heading change against the gyro costs thPrior per rad^2 (4 deg = 0.2 of score); the first
  // (wide) match of a route is free, it finds how the rover was put down.
  const float thW = wide ? 0.3f : thPrior;
  b = _search(px, py, n, cx, cy, ct, 0.05f, 8, kDthCoarse, 5, prx, pry, prth, 1.0f, thW);
  b = _search(px, py, n, b.x, b.y, b.th, 0.02f, 3, kDthFine, 3, prx, pry, prth, 1.0f, thW);
  _score = b.sc;
  if (b.sc > accept) {
    _x = b.x; _y = b.y; _th = b.th;
    _ok = true;
    _first = false;
    _lost = 0;
  } else if (_lost < 255) {
    _lost++;
  }
}

void LocalLocalizer::_relocate(const float* px, const float* py, int n) {
  const Leg& L = kLeg[_leg];
  const int span = 12;  // 3 m / 0.25 m
  Best best{0, 0, 0, -9.0f};
  const int q0 = (int)_pi - span < 0 ? 0 : (int)_pi - span;
  const int q1 = (int)_pi + span + 1 < (int)L.nPath ? (int)_pi + span + 1 : (int)L.nPath;
  for (int q = q0; q < q1; q += 2) {
    const int qa = q + 1 < L.nPath ? q + 1 : L.nPath - 1, qb = q - 1 > 0 ? q - 1 : 0;
    const float ax = (L.path[2 * qa] - L.path[2 * qb]) * 0.01f, ay = (L.path[2 * qa + 1] - L.path[2 * qb + 1]) * 0.01f;
    const float th0 = atan2f(ay, ax);
    for (int d = -20; d <= 20; d += 10) {
      const float cx = L.path[2 * q] * 0.01f, cy = L.path[2 * q + 1] * 0.01f, ct = th0 + d * kDeg;
      const Best b1 = _search(px, py, n, cx, cy, ct, 0.05f, 4, kDthZero, 1, cx, cy, ct, 0.3f, 0.3f);
      const Best b2 = _search(px, py, n, b1.x, b1.y, b1.th, 0.02f, 3, kDthFine, 3, cx, cy, ct, 0.3f, 0.3f);
      if (b2.sc > best.sc) best = b2;
    }
  }
  if (best.sc > acceptReloc) {
    _x = best.x; _y = best.y; _th = best.th;
    _ok = true;
    _score = best.sc;
    _lost = 0;
  }
}

void LocalLocalizer::_project() {
  const Leg& L = kLeg[_leg];
  const int lo = (int)_pi - 8 > 0 ? (int)_pi - 8 : 0;
  const int hi = (int)_pi + 40 < (int)L.nPath ? (int)_pi + 40 : (int)L.nPath;
  float bd = 1e9f;
  int bk = lo;
  for (int k = lo; k < hi; k++) {
    const float dx = L.path[2 * k] * 0.01f - _x, dy = L.path[2 * k + 1] * 0.01f - _y;
    const float d = sqrtf(dx * dx + dy * dy);
    if (d < bd) { bd = d; bk = k; }
  }
  _pi = (uint16_t)bk;
  _off = bd;
}

// scene unchanged for 1 s (median point change < 2.5 cm): not moving, whatever the wheels do
bool LocalLocalizer::_still(const uint16_t* scanCm) {
  bool still = false;
  if (_histN >= 4) {
    const uint16_t* b = _hist[_histI];  // the oldest of the 4 = 4 steps ago
    float d[kN];
    int m = 0;
    for (int i = 0; i < kN; i++) {
      const uint16_t a = scanCm[i], c = b[i];
      if (a > 25 && c > 25 && a < 500 && c < 500) d[m++] = fabsf((float)a - (float)c);
    }
    if (m >= 15) {
      for (int x = 1; x < m; x++) for (int q = x; q > 0 && d[q - 1] > d[q]; q--) { const float t = d[q]; d[q] = d[q - 1]; d[q - 1] = t; }
      const float med = (m & 1) ? d[m / 2] : 0.5f * (d[m / 2 - 1] + d[m / 2]);
      still = med < 2.5f;
    }
  }
  memcpy(_hist[_histI], scanCm, sizeof(_hist[0]));
  _histI = (_histI + 1) & 3;
  if (_histN < 4) _histN++;
  return still;
}

void LocalLocalizer::step(const uint16_t* scanCm, float dyawRad, float effDelta, bool tilted) {
  _ok = false;
  _atEnd = false;
  const bool still = _still(scanCm);
  _th = wrapPi(_th + dyawRad);
  // tilted (ramp): no scan match (the LiDAR plane hits the ground / the sky), but keep counting the
  // distance - log_017: the pose froze for seconds on the ramp while the rover drove on
  if (!still && effDelta > 0) {
    const float d = kSpeed * effDelta;
    _x += d * cosf(_th);
    _y += d * sinf(_th);
  }
  // scan points (rover frame), rear sector (130-230 deg) = the operator: left out
  float px[kN], py[kN];
  int n = 0;
  for (int i = 0; i < kN; i++) {
    const float deg = i * 4 + 2.0f;
    if (deg > 130 && deg < 230) continue;
    const float r = scanCm[i] / 100.0f;
    if (r <= 0.3f || r >= kRmax) continue;
    const float a = -deg * kDeg;
    px[n] = r * cosf(a);
    py[n] = r * sinf(a);
    n++;
  }
  _score = 0;
  if (n >= 12 && !tilted) {
    _match(px, py, n, _first);
    const uint16_t keep = _pi;
    _project();
    const float offNow = _off;
    _pi = keep;
    if ((_lost >= 8 || offNow > 1.0f) && n >= 20) _relocate(px, py, n);
  }
  _project();
  if (_pi * kStep >= kLeg[_leg].sEnd - 0.25f) _atEnd = true;   // last leg too: route done
}

void LocalLocalizer::nextLeg() {
  if (_leg + 1 >= kLegs) return;
  const Leg& L = kLeg[_leg];
  // pose into the next frame (origin = L.next in this frame)
  const float c = cosf(-L.nextTh), s = sinf(-L.nextTh);
  const float dx = _x - L.nextX, dy = _y - L.nextY;
  _x = c * dx - s * dy;
  _y = s * dx + c * dy;
  _th = wrapPi(_th - L.nextTh);
  _leg++;
  _pi = 0;
  _atEnd = false;
  _buildWp();
}

uint8_t LocalLocalizer::newPointsAhead(const uint16_t* scanCm, float half, float look) const {
  const Leg& L = kLeg[_leg];
  const float c = cosf(_th), s = sinf(_th), lx = L.loX * 0.01f, ly = L.loY * 0.01f;
  uint8_t n = 0;
  for (int i = 0; i < kN; i++) {
    if (scanCm[i] <= 25) continue;
    const float a = -(i * 4 + 2.0f) * kDeg, r = scanCm[i] / 100.0f;
    const float al = r * cosf(a), la = r * sinf(a);
    if (al <= 0.05f || al >= look || fabsf(la) >= half) continue;
    const float wx = _x + c * al - s * la, wy = _y + s * al + c * la;
    const int gi = (int)((wx - lx) / kRes), gj = (int)((wy - ly) / kRes);
    int v = 0;
    if (gi >= 0 && gi < L.nx && gj >= 0 && gj < L.ny) {
      const uint32_t idx = (uint32_t)gi * L.ny + gj;
      const uint8_t b = L.grid[idx >> 1];
      v = (idx & 1) ? (b >> 4) : (b & 15);
    }
    if (v < 6 && n < 255) n++;   // < 6/15: no taught wall within ~20 cm
  }
  return n;
}

float LocalLocalizer::headingErrorTo(float lookM) const {
  const Leg& L = kLeg[_leg];
  int q = _pi + (int)(lookM / kStep + 0.5f);
  if (q >= L.nPath) q = L.nPath - 1;
  const float tx = L.path[2 * q] * 0.01f, ty = L.path[2 * q + 1] * 0.01f;
  return wrapPi(atan2f(ty - _y, tx - _x) - _th);
}

void LocalLocalizer::global(float& gx, float& gy, float& gth) const {
  const Leg& L = kLeg[_leg];
  const float c = cosf(L.origTh), s = sinf(L.origTh);
  gx = L.origX + c * _x - s * _y;
  gy = L.origY + s * _x + c * _y;
  gth = wrapPi(L.origTh + _th);
}
