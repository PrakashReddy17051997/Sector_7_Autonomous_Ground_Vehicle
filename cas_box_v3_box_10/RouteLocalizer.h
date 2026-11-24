#pragma once
// RouteLocalizer - LiDAR + IMU localisation along the taught route (teach & repeat), used by RouteFollow.
//
// WHY: the leg (segment) used to end on effort (how much the rover drove) plus one landmark event
// (wall lost / found / something ahead) plus a GPS zone. Effort drifts (log_011: legs ended 10-20 m too
// early - "exits the corridor prematurely", "does not go to the ramp"), a landmark event also fires in
// a gap or a doorway, and GPS is 6-15 m off between days. The LiDAR scan at the END of every leg was
// recorded in the teaching run (RouteMap.h), and it is distinctive: facade end, ramp entry, corner,
// passage. So the leg now ends only where the live scan looks like that taught end scan.
//
// 1) END OF LEG  (endStep, 2x per second after 30 % of the taught effort)
//      cost   = mean range error (cm) between the live scan and the leg's end keyframe, best of
//               +-24 deg rotation; each point compared with 3 neighbouring keyframe points; error capped
//               at 80 cm; a structure within 3 m seen in only one of the scans counts 80 cm.
//      base   = median cost of the first half of the leg (how different "not yet there" looks)
//      dip    = after the gate: the cost had a clear minimum (< base - 12 cm) and rose again (3 samples)
//               -> the rover passed the taught end point: end the leg.
//      looksLikeEnd = cost now <= base - 4 cm: a landmark event (wall lost ...) is only accepted then.
//    RouteFollow (endRule = RULE_COMBINED):  end = (landmark && looksLikeEnd) || dip,
//                                           no GPS zone, last resort 1.8 x taught effort + 2.
//
// 2) STATIC vs NEW OBSTACLES  (newPointsAhead)
//      A point in the corridor ahead is STATIC if the teaching run saw something at the same range
//      (+-30 cm) and direction (+-12 deg) in any route keyframe of this leg or the next one: walls ahead
//      at a turn, railings, ramp edges. Only NEW points (people, bikes, bins) start the obstacle
//      handling. log_011: 139 "something within 0.7 m" events, only 24 new. SafetyGuard still stops for
//      anything closer than 25-30 cm.
//
// Python twin with the same numbers: tools/lidarmatch.py; map + replay check: tools/make_segments.py.
// Host-testable: no Arduino calls, scans are passed in as 90 values in cm (Sensors::lidarCm).
#include <stdint.h>
#include <stddef.h>

class RouteLocalizer {
public:
  static const uint8_t kN = 90;

  // ---- tuning (same values as tools/lidarmatch.py)
  float dropCm = 12.0f;        // dip must be this far below the leg's base level
  float maxRangeCm = 600.0f;   // farther returns are not compared
  float truncCm = 80.0f;       // per-point error cap
  float nearCm = 300.0f;       // structure only in one scan counts if closer than this
  uint8_t minBins = 12;        // fewer comparable points: no answer
  int8_t rotPts = 6;           // rotation search +- 6 points (+- 24 deg)
  float staticTolCm = 30.0f;   // obstacle point explained by a taught return within this range ...
  int8_t staticRotPts = 3;     // ... and +- 3 points (12 deg) of direction

  bool enabled = true;         // RouteFollow switches it off if RouteMap.h does not fit kRoute
  static uint8_t mapLegs();    // number of legs in RouteMap.h

  void beginLeg(uint8_t leg);
  // every RouteFollow DRIVE step (4 Hz): frac = effort / taught effort, gate = landmark gate passed
  void endStep(const uint16_t* scanCm, float frac, bool gate);
  bool dip() const;
  bool looksLikeEnd() const;
  // number of NEW points (not in the taught scans) in the corridor ahead (half width, look in m)
  uint8_t newPointsAhead(const uint16_t* scanCm, float half, float look) const;

  // for status / log
  float costNow() const { return _now; }
  float base() const;
  float best() const { return _best; }
  uint8_t samples() const { return _n; }
  bool hasMap() const;

  // matching core (public for the host test)
  float scanCost(const uint16_t* c, const uint8_t* key, int shift) const;  // < 0: no answer
  float endCost(const uint16_t* c, const uint8_t* key) const;              // < 0: no answer

private:
  uint8_t _leg = 0;
  float _hist[64];
  uint8_t _n = 0;
  float _best = 9e9f, _now = -1;
  bool _hasBest = false;
  uint8_t _rising = 0;
  bool _tick = false;
};
