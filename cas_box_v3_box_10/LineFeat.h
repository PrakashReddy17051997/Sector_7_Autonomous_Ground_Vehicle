#pragma once
// LineFeat - robust straight-line extraction from the 90-point LiDAR snapshot (v3, 2026-11-10).
// Host-testable: no Arduino calls (tools/replay_fw builds it on the laptop).
//
// WHY: the landmark legs (Open Sea, last leg to Block I) steer on long straight things - a facade,
// a store front, the wall behind / ahead. Trees, poles, people and snow give extra points close to
// such a line; a plain least-squares fit would be pulled towards them. So:
//   1. RANSAC (deterministic, ALL point pairs - a sector has <= 30 points, so <= 435 pairs):
//      the line through the pair that has the most points within tol(r) = tol0 + tolK * r
//      (4 deg bins: a point at 7 m sits up to ~25 cm "beside" its true direction -> tol grows with r)
//   2. total-least-squares refit on those inliers (principal axis of the 2x2 covariance), twice.
//   3. checks: >= minPts inliers, span along the line >= minLen, rms <= maxRms.
//
// LINE FORM (rover frame, x forward, y left, metres):   cos(foot)*x + sin(foot)*y = d,  d >= 0
//   foot = direction from the rover to the closest point of the line, deg, CCW (+ = left):
//          wall on the left  -> foot ~ +90     wall on the right -> foot ~ -90
//          wall ahead        -> foot ~ 0       wall behind       -> foot ~ 180
//   d    = perpendicular distance (m)
// If the rover turns LEFT by e deg, every foot angle drops by e. So the rover's heading against the
// course the line stands for is   e = footExpected - footMeasured   (wrapped to +-180).
#include <stdint.h>
#include <math.h>

namespace LineFeat {

static const int kN = 90;               // snapshot points, 4 deg each, index 0 = front, CLOCKWISE
static const float kDegRad = 0.01745329252f;

inline float wrap180(float a) { a = fmodf(a + 180.0f, 360.0f); if (a < 0) a += 360.0f; return a - 180.0f; }

// scan bin i -> CCW angle in the rover frame (deg). Same convention as LocalLocalizer (bin centre).
inline float binDegCcw(int i) { return wrap180(-(i * 4 + 2.0f)); }

struct Cfg {
  float centerDeg = 90;   // sector centre (CCW, + = left)
  float halfDeg = 45;     // sector half width
  float rMin = 0.3f, rMax = 10.0f;
  float footExp = 90;     // expected foot direction ...
  float footTol = 25;     // ... a candidate further off is ignored (a diagonal fence is not the facade)
  float tol0 = 0.05f, tolK = 0.025f;   // inlier tolerance tol0 + tolK * r  (m)
  float minSep = 0.4f;    // a RANSAC pair must be at least this far apart
  uint8_t minPts = 5;
  float minLen = 1.0f;    // span of the inliers along the line (m)
  float maxRms = 0.12f;
};

struct Line {
  bool ok = false;
  float d = 0;            // m
  float foot = 0;         // deg CCW
  float len = 0;          // m, span of the inliers
  float rms = 0;          // m
  uint8_t pts = 0;        // inliers
  uint8_t n = 0;          // points in the sector
  uint8_t drop = 0;       // bins in the sector with NO return (glass, open space, snow)
  uint8_t bins = 0;       // bins in the sector
  float dropFrac() const { return bins ? (float)drop / bins : 0; }
};

// TLS fit of the flagged points; returns false if fewer than 2
inline bool _fit(const float* x, const float* y, const uint8_t* use, int n, float& foot, float& d, float& rms) {
  float sx = 0, sy = 0; int m = 0;
  for (int k = 0; k < n; k++) if (use[k]) { sx += x[k]; sy += y[k]; m++; }
  if (m < 2) return false;
  const float cx = sx / m, cy = sy / m;
  float sxx = 0, syy = 0, sxy = 0;
  for (int k = 0; k < n; k++) if (use[k]) {
    const float dx = x[k] - cx, dy = y[k] - cy;
    sxx += dx * dx; syy += dy * dy; sxy += dx * dy;
  }
  // direction of the major axis, normal = +90 deg
  const float th = 0.5f * atan2f(2.0f * sxy, sxx - syy);
  float nx = -sinf(th), ny = cosf(th);
  float dd = nx * cx + ny * cy;
  if (dd < 0) { nx = -nx; ny = -ny; dd = -dd; }
  // smallest eigenvalue = mean squared distance to the line
  const float tr = sxx + syy, det = sxx * syy - sxy * sxy;
  const float lmin = 0.5f * (tr - sqrtf(fmaxf(tr * tr - 4.0f * det, 0.0f)));
  foot = atan2f(ny, nx) / kDegRad;
  d = dd;
  rms = sqrtf(fmaxf(lmin, 0.0f) / m);
  return true;
}

// Extract the best line in the sector. scanCm: 90 points, cm, 0 = no return.
inline Line extract(const uint16_t* scanCm, const Cfg& c) {
  Line L;
  float x[kN], y[kN], r[kN];
  int n = 0;
  for (int i = 0; i < kN; i++) {
    const float a = binDegCcw(i);
    if (fabsf(wrap180(a - c.centerDeg)) > c.halfDeg) continue;
    L.bins++;
    const float rr = scanCm[i] * 0.01f;
    if (scanCm[i] == 0) { L.drop++; continue; }
    if (rr < c.rMin || rr > c.rMax) continue;
    x[n] = rr * cosf(a * kDegRad); y[n] = rr * sinf(a * kDegRad); r[n] = rr; n++;
  }
  L.n = (uint8_t)n;
  if (n < c.minPts) return L;

  // 1. RANSAC over all pairs
  int bestCnt = 0; float bestSpan = 0, bnx = 0, bny = 0, bd = 0;
  for (int i = 0; i < n; i++) for (int j = i + 1; j < n; j++) {
    float tx = x[j] - x[i], ty = y[j] - y[i];
    const float sep = sqrtf(tx * tx + ty * ty);
    if (sep < c.minSep) continue;
    tx /= sep; ty /= sep;
    float nx = -ty, ny = tx, d = nx * x[i] + ny * y[i];
    if (d < 0) { nx = -nx; ny = -ny; d = -d; }
    if (fabsf(wrap180(atan2f(ny, nx) / kDegRad - c.footExp)) > c.footTol) continue;
    int cnt = 0; float smin = 1e9f, smax = -1e9f;
    for (int k = 0; k < n; k++) {
      if (fabsf(nx * x[k] + ny * y[k] - d) <= c.tol0 + c.tolK * r[k]) {
        cnt++;
        const float s = -ny * x[k] + nx * y[k];
        if (s < smin) smin = s;
        if (s > smax) smax = s;
      }
    }
    const float span = smax - smin;
    if (cnt > bestCnt || (cnt == bestCnt && span > bestSpan)) { bestCnt = cnt; bestSpan = span; bnx = nx; bny = ny; bd = d; }
  }
  if (bestCnt < c.minPts) return L;

  // 2. refit (twice: the refit line can take in / drop a point or two)
  uint8_t use[kN];
  float foot = atan2f(bny, bnx) / kDegRad, d = bd, rms = 0;
  for (int it = 0; it < 2; it++) {
    const float nx = cosf(foot * kDegRad), ny = sinf(foot * kDegRad);
    for (int k = 0; k < n; k++) use[k] = fabsf(nx * x[k] + ny * y[k] - d) <= c.tol0 + c.tolK * r[k];
    if (!_fit(x, y, use, n, foot, d, rms)) return L;
  }
  const float nx = cosf(foot * kDegRad), ny = sinf(foot * kDegRad);
  int cnt = 0; float smin = 1e9f, smax = -1e9f;
  for (int k = 0; k < n; k++) if (use[k]) {
    cnt++;
    const float s = -ny * x[k] + nx * y[k];
    if (s < smin) smin = s;
    if (s > smax) smax = s;
  }
  L.d = d; L.foot = foot; L.rms = rms; L.pts = (uint8_t)cnt; L.len = cnt ? smax - smin : 0;
  L.ok = cnt >= c.minPts && L.len >= c.minLen && rms <= c.maxRms &&
         fabsf(wrap180(foot - c.footExp)) <= c.footTol;
  return L;
}

}  // namespace LineFeat
