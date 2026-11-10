#pragma once
// LandmarkRoute - the phase tables of the landmark legs (v3, 2026-11-10). Which route leg uses which
// table is set in cas_box_v3.ino (OPEN_SEA_LEG, BLOCK_I_LEG).
//
// Field order of every row (LandmarkLeg.h explains each type):
//  {type, footExp, sFrom, sTo, dRef, dMin, dMax, kp, ki, kd, sat, para, minLen, anchor, a, a2, b, c, lineOff}
//   lineOff  taught course of the line minus the leg ref (deg) - landmark_scan.py measures it
//   footExp  +90 line on the LEFT, -90 RIGHT, 0 AHEAD, 180 BEHIND
//   s        metres driven on this leg (RouteFollow::legMetres, 1.03 m per unit of effort, log_027)
//   kp/ki/kd deg per m / deg per (m*s) / deg per (m/s), sat = turn limit (deg), para = parallel check (deg)
#include "LandmarkLeg.h"

// ---- OPEN SEA = our leg 2 (M15-M34, 34 m) -----------------------------------------------------------
// Measured with tools/landmark_scan.py on both teaching runs (lineOff on log_027, the kRoute frame:
// behind -9.5, store front -9.6, ahead -9.9 deg, quartiles within 1-3 deg -> the walls are square to
// each other; the 9.6 deg is the rover's crab, so no extra crab term on landmark legs):
//   wall behind     log_026 3.2-5.9 m, log_027 3.1-8.3 m, s 0-9 m           -> heading anchor
//   store front L   log_026 6.5-7.2 m (s 12-25), log_027 6.2-6.9 m (s 10-21) -> side PID at 6.75 m
//                   (the batchmate's 6.65 m)
//   wall ahead      log_027 8.6 m at s 24 -> 3.1 m at the leg end (M34)      -> stop at 3.0 m
//   ramp            s 0-6 m (pitch +5 deg) and a step at ~28 m: lines are ignored while tilted
// log_029 (mission, path pursuit on a weak match): the store front distance grew 2.3 -> 6.9 m over
// s 10-18 m = the rover turned 50 deg off. Here the course is held on the gyro + the anchors, and a line
// that is not parallel to the course (+-8 deg) is never used.
static const LmkPhase kOpenSea[] = {
  {LandmarkLeg::ANCHOR, 180, 0.0f, 9.0f, 0, 1.0f, 9.0f, 0, 0, 0, 0, 8, 2.0f, 0.10f, 0, 0, 0, 0, -9.5f},
  {LandmarkLeg::SIDE, 90, 9.0f, 25.0f, 6.75f, 4.5f, 9.0f, 10, 1.5f, 20, 8, 8, 2.5f, 0.05f, 0, 0, 0, 0, -9.6f},
  {LandmarkLeg::STOP, 0, 25.0f, 43.0f, 3.0f, 1.0f, 9.5f, 0, 0, 0, 0, 10, 2.0f, 0.10f, 0, 0, 0, 0, -9.9f},
};

// ---- LAST LEG TO BLOCK I (Arjun's v12 numbers, NOT our measurements) ----------------------------
// Our taught route (log_027) ends at M72 before this stretch. Teach it, run
//   python3 tools/landmark_scan.py cas_box_log/log_0xx.csv --marks <from> <to>
// and put our own distances / windows / lineOff in before setting BLOCK_I_LEG in cas_box_v3.ino
// (lineOff is left at 0 here = unknown).
static const LmkPhase kBlockI[] = {
  // wall follow 1.0 m on the left, 0-13 m
  {LandmarkLeg::SIDE, 90, 0.0f, 13.0f, 1.0f, 0.4f, 2.5f, 10, 1.5f, 20, 8, 10, 1.0f, 0.05f, 0, 0, 0, 0},
  // walkway: keep the left line (benches, planter, building base) at 2.1 m (Arjun v12: taught from c102_3, which
  // reached Block I at 1.8-2.3 m; "keep the first-seen distance" let campus_11 run at 1.5-1.8 m on the cobbles)
  {LandmarkLeg::SIDE, 90, 13.0f, 30.0f, 2.1f, 1.0f, 3.0f, 10, 0, 0, 6, 10, 0.6f, 0.0f, 0, 0, 0, 0},
  // railing corridor: left facade kept; found when the right parapet is 0.35-1.5 m (v12: was 1.3) and, if the left
  // has a line, the width is 2.4-3.3 m - on 3 of the last 5 scans (v12). His parapet appears 16-20 m after the left wall ends - re-measure on ours
  {LandmarkLeg::CORRIDOR, 90, 30.0f, 40.0f, 0, 1.2f, 3.0f, 10, 0, 0, 6, 10, 1.0f, 0.03f, 2.4f, 3.3f, 1.5f, 0},
  // KARL entrance: facade steps back >= 0.5 m or glass (>= 50 % no return) -> stop 1.5 m later (47.0 -> 48.5 m)
  {LandmarkLeg::ENTRANCE, 90, 36.0f, 52.0f, 0, 1.2f, 3.5f, 10, 0, 0, 6, 10, 1.0f, 0.0f, 0.5f, 0, 0.5f, 1.5f},
};

#define LMK_COUNT(t) ((uint8_t)(sizeof(t) / sizeof((t)[0])))
