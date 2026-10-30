#include "RouteLocalizer.h"
#include <math.h>
#include "RouteMap.h"

static inline int wrapIdx(int i) { i %= RouteLocalizer::kN; return i < 0 ? i + RouteLocalizer::kN : i; }

bool RouteLocalizer::hasMap() const { return enabled && _leg < RouteMap::kLegs; }
uint8_t RouteLocalizer::mapLegs() { return RouteMap::kLegs; }

void RouteLocalizer::beginLeg(uint8_t leg) {
  _leg = leg;
  _n = 0;
  _best = 9e9f;
  _hasBest = false;
  _rising = 0;
  _now = -1;
  _tick = false;
}

// mean per-point range error (cm); keyframe point j = key[j] * 4 cm, compared at index i + shift
float RouteLocalizer::scanCost(const uint16_t* c, const uint8_t* key, int shift) const {
  float sum = 0;
  int n = 0;
  for (int i = 0; i < kN; i++) {
    const float ci = c[i];
    const bool vc = ci > 20 && ci < maxRangeCm;
    const float k0 = key[wrapIdx(i + shift)] * 4.0f;
    const bool vk = k0 > 20 && k0 < maxRangeCm;
    if (vc && vk) {
      float e = truncCm;
      for (int d = -1; d <= 1; d++) {
        const float kk = key[wrapIdx(i + shift + d)] * 4.0f;
        if (kk > 20 && kk < maxRangeCm) {
          const float x = fabsf(ci - kk);
          if (x < e) e = x;
        }
      }
      sum += e;
      n++;
    } else if ((vc && !vk && ci < nearCm) || (!vc && vk && k0 < nearCm)) {
      sum += truncCm;  // structure close by in only one of the two scans
      n++;
    }
  }
  return n < minBins ? -1.0f : sum / n;
}

float RouteLocalizer::endCost(const uint16_t* c, const uint8_t* key) const {
  float best = -1;
  for (int s = -rotPts; s <= rotPts; s++) {
    const float v = scanCost(c, key, s);
    if (v >= 0 && (best < 0 || v < best)) best = v;
  }
  return best;
}

float RouteLocalizer::base() const {
  if (_n == 0) return -1;
  uint8_t m = _n / 2;
  if (m < 4) m = 4;
  if (m > _n) m = _n;
  float a[64];
  for (uint8_t i = 0; i < m; i++) a[i] = _hist[i];
  for (uint8_t i = 1; i < m; i++)  // insertion sort (m <= 32)
    for (uint8_t j = i; j > 0 && a[j - 1] > a[j]; j--) { const float t = a[j]; a[j] = a[j - 1]; a[j - 1] = t; }
  return (m & 1) ? a[m / 2] : 0.5f * (a[m / 2 - 1] + a[m / 2]);
}

void RouteLocalizer::endStep(const uint16_t* scanCm, float frac, bool gate) {
  if (!hasMap() || frac < 0.3f) return;
  _tick = !_tick;
  if (!_tick) return;                                  // 2 Hz
  const float c = endCost(scanCm, RouteMap::kEnd[_leg]);
  if (c < 0) return;
  _now = c;
  if (_n < 64) _hist[_n++] = c;
  if (!gate) return;
  if (c < _best) { _best = c; _hasBest = true; _rising = 0; }
  else if (_hasBest && c > _best + dropCm / 2) { if (_rising < 255) _rising++; }
}

bool RouteLocalizer::dip() const {
  const float b = base();
  return _hasBest && b >= 0 && _best < b - dropCm && _rising >= 3;
}

bool RouteLocalizer::looksLikeEnd() const {
  const float b = base();
  return _now < 0 || b < 0 || _now <= b - dropCm / 3;
}

uint8_t RouteLocalizer::newPointsAhead(const uint16_t* scanCm, float half, float look) const {
  uint8_t nNew = 0;
  for (int b = 0; b < kN; b++) {
    const float cm = scanCm[b];
    if (cm <= 25) continue;
    const float a = (b * 4.0f + 2.0f) * 0.01745329252f, r = cm / 100.0f;
    const float along = r * cosf(a), lat = r * sinf(a);
    if (along <= 0.05f || along >= look || fabsf(lat) >= half) continue;  // > 5 cm ahead, not beside
    bool isStatic = false;
    for (uint8_t k = 0; k < RouteMap::kKeys && !isStatic; k++) {
      const uint8_t lg = RouteMap::kKeyInfo[k].leg;
      if (lg != _leg && lg != _leg + 1) continue;    // taught scene of this leg and the next one
      for (int d = -staticRotPts; d <= staticRotPts; d++) {
        const float kb = RouteMap::kKey[k][wrapIdx(b + d)] * 4.0f;
        if (kb > 0 && fabsf(kb - cm) < staticTolCm) { isStatic = true; break; }
      }
    }
    if (!isStatic && nNew < 255) nNew++;
  }
  return nNew;
}
