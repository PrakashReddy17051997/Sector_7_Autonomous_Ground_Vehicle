#pragma once
// RouteFollow - teach & repeat: drives the route of the teaching run (now log_013) leg by leg with
// LiDAR + gyro. GPS is NOT used (RULE_COMBINED: leg ends = landmark + LiDAR end scan).
//
// THE ROUTE (kRoute in RouteFollow.cpp, made by tools/make_legs.py from the tagged Mark presses)
//   17 legs (log_013), each:  kind   WALL = follow the wall on side L/R at dSet,  HOLD = keep a heading
//                   ref    direction of the leg in the "route yaw" frame (= gyro yaw of log_009)
//                   end    landmark that ends the leg: wall LOST, wall FOUND (closer than endArg),
//                          FRONT (something ahead closer than endArg), or EFFORT only
//                   effort how much driving the leg took in log_009 = sum(forward wheel cmd * s)
//                   gate   the landmark counts only after gate * effort (no early false end)
//   Between two legs: TURN on the spot (wheels +-0.5) to the next ref.
//
// LOCALISATION (why it works without GPS)
//   heading:  routeYaw = gyroYaw + offset. offset is set at the start from the wall
//             (rover parallel to the B facade: offset = ref - (yaw + wall angle)) and corrected on
//             every WALL leg: measured wall direction vs taught ref, 5 % per scan. log_009: the
//             wall directions of the route stay within 1-3 deg over 10 minutes -> the gyro holds.
//   progress: effort since the leg start (the same rover on the same ground needs the same
//             effort) + the landmark at the end of the leg. A leg that needs much more effort than
//             taught -> AUTOMATIC switch to the next leg (autoAdvance, 2026-10-24): after
//             advanceEffortFrac (1.3) * taught + 2 the leg counts as driven. Also on every leg:
//             after the gate, GPS inside 3 m of the taught end point ends the leg (zone help).
//             Only a stuck rover (5 back-ups) or a blocked way (3 x) still pauses - safety.
//   GPS:      zone check at each leg end (warning if > zoneWarnM from the taught end point), and on
//             EFFORT legs the leg may end early inside 2 m of the taught end (if GPS is not blind).
//             Leg 8 (open diagonal, no walls): heading pulled towards the taught end point, at most
//             +-15 deg from the taught heading.
//
// CONTROL (2026-10-29, second version after the field test log_014 - see _control in RouteFollow.cpp)
//   heading:  every loop pass on the live gyro yaw (50 Hz). Corrections are PIVOTS closed on the
//             gyro (a wheel difference does not turn this rover): error > corrDeg (8) after >= 0.6 s of
//             rolling, or > 25 at once -> pivot until the heading is reached minus what it still
//             turns (rate * 0.15 s). It aims a little past the target by the rover's own drift
//             (learned, ~3 deg/s to the right) so the drift brings it back instead of the next pivot.
//   WALL leg: yawDes = wall direction + approach atan((d - dSet) / lookAheadM 1.5 m), max +-15 deg;
//             the wall direction / distance are the median of the last 5 readings, a reading 8 deg
//             off that median is dropped (single scans fitting a corner or a gap: log_014).
//             No valid wall -> keep ref (gaps, doors).
//   start:    WALL leg: the rover must stand parallel to the wall (+-15 deg), else "turn rover
//             parallel" - log_0014 started at -54 deg and the whole leg was off.
//   closer than hardStopM (0.45 m) ahead: always obstacle handling (the static filter only for
//             0.45 - 0.7 m); a leg that ends at a wall ahead (END_FRONT) ends there.
//   obstacle: corridor ahead < avoidEnterM -> wait 3 s (people pass), then sidestep: turn on the
//             spot to the closest free direction (20..60 deg, never right on the raised path),
//             drive until the leg direction is free again, turn back parallel, hold the wall
//             direction for ~1.5 m, then back to the set distance. No free direction 3x -> PAUSE.
//             (tools/sim_route.py: box on the facade path passed with 0.36 m clearance)
//   stuck:    commanded to move but the LiDAR scan changed < 2.5 cm and yaw < 3 deg in 2 s,
//             twice -> back up 1.5 s; third time in a leg -> PAUSE "stuck".
//   (log_010: -0.10/+0.50 for 25 s without moving - this cannot happen any more)
//
//   Used by Mission (plan "route"); see Mission.h.
//
// LIDAR LOCALISATION (added 2026-10-28, RouteLocalizer + RouteMap.h from tools/make_segments.py)
//   endRule = RULE_COMBINED (default): a leg ends only where the live LiDAR scan looks like the scan
//             recorded at the END of that leg in the teaching run:
//               end = (landmark event && scan looks like the end) || clear dip of the end-scan cost
//             no GPS zone, last resort after lastResortFrac (1.8) x taught effort + 2.
//             Replay on the logs (make_segments.py --check): log_011 median error at the leg ends
//             14.3 -> 8.4 m, worst 20.3 -> 11.6 m; log_010 1.6 -> 5.5 m median, worst 7.4 -> 9.2 m.
//             Not field-tested yet: compare in the next test, endRule = RULE_OLD gives the old rule.
//   staticFilter (default on): something in the corridor ahead starts the obstacle handling only if
//             the teaching run did NOT see it there (walls ahead at turns, railings, ramp edges are
//             static). log_011: 139 events -> 24. SafetyGuard still stops at 25-30 cm for anything.
#include <Arduino.h>
#include <math.h>
#include "Sensors.h"
#include "WallFollow.h"
#include "RouteLocalizer.h"
#include "LocalLocalizer.h"
#include "LandmarkLeg.h"

class RouteFollow {
public:
  enum Kind : uint8_t { WALL = 0, HOLD = 1 };   // v3: a leg with a landmark table (setLandmarkLeg) drives LMK

  enum End : uint8_t { END_LOST = 0, END_FOUND_L = 1, END_FOUND_R = 2, END_FRONT = 3, END_EFFORT = 4 };
  struct Leg {
    uint8_t kind; int8_t side; uint8_t end; float endArg;
    float ref, dSet, effort, gate;
    double lat, lon;          // GPS at the end of the leg in the teaching run (zone only)
    uint8_t gpsBlind;         // GPS did not move in the teaching run here (building shadow)
    uint8_t noRight;          // never avoid to the right (raised path, open edge)
    uint8_t gpsAssist;        // HOLD leg: heading pulled towards the end zone (+-15 deg)
    const char* name;
  };
  enum State : uint8_t { OFF = 0, TURN = 1, DRIVE = 2, AVOID = 3, BACKUP = 4, PAUSED = 5, DONE = 6 };

  // ---- tuning ----
  // heading controller (_control, every loop pass)
  float corrDeg = 8.0f;       // heading error above this -> gyro-closed pivot (after minRollMs rolling)
  float pivotDeg = 25.0f;     // above this: pivot at once
  uint32_t minRollMs = 600;   // roll at least this long between two corrections (no pivot dithering)
  float driftLeadS = 0.6f;    // aim this many seconds of the rover's own drift beyond the target
  float startTolDeg = 15.0f;  // WALL leg start: rover must be parallel to the wall within this
  float hardStopM = 0.45f;    // something this close ahead: always stop / avoid (static filter or not)
  // RAMP legs (bit i = leg i+1): the flat LiDAR beam hits the rising ramp surface and sees a "wall"
  // (log_013: 36 LiDAR-front safety stops on the ramp while you drove it by hand). On these legs the
  // LiDAR obstacle handling and hard stop are OFF, and cas_box.ino switches the SafetyGuard LiDAR
  // stop off; the front ULTRASONIC stop (25 cm), the tilt stop and the stuck detection stay ON.
  // log_013 route: leg 4 (M5-M10, ramp up/down), leg 5 (M10-M13, ramp), leg 11 (M23-M28, ramp).
  // log_026 (06.11.): leg 4 (M16-M37) holds the ramp (160 s above 4 deg pitch, up to 18 deg)
  uint32_t rampLegs = (1u << 1) | (1u << 5);   // legs 2 (log_027 M15-M34) and 6 (log_056 M47-M48, ramp 3) hold the ramps
  // YAW-ONLY legs (bit i = leg i+1), 06.11.: open area / ramp, where the LiDAR match is weak (replay of
  // log_025 on the log_026 map located legs 1-4 at 90-100 %; legs 2 and 11 are the open areas). On these legs the
  // rover does NOT steer on the LiDAR path: it holds the taught heading ref in the route yaw frame
  // (relative gyro yaw + the offset corrected on the wall legs before). The leg ends on the taught
  // distance (effort) + the LiDAR end-scan match. Obstacle handling and SafetyGuard stay on; the LiDAR
  // localiser keeps running so it can lock again on the next leg.
  // 19.11. (log_042): landmark legs that use ODOMETRY only - the locked localizer neither sets their distance nor
  // ends them (it jumped 4 m ahead on legs 8-9 and ended them early). Bit i = leg i+1.
  uint32_t lmkOdoLegs = 0;   // 26.11.: legs 8/9 (pole, log_027) are gone - legs 5-11 are log_056 path legs
  // 26.11. (log_055, box 10): leg 9 drove its first 3 m on the bare IMU course, ~20 deg right into the kerb, and the
  // wall ahead was never seen -> it ended on distance and leg 10 turned 70 deg right in the open area. Now:
  //   lmkPathLegs     the taught path STEERS whenever no landmark line steers (HOLD phase, no line, on slope) and the
  //                   localizer is locked with score >= minPathScore - even on an odometry-only leg (it still neither
  //                   sets the distance nor ends the leg: lmkOdoLegs)
  //   lmkNeedEndLegs  a DISTANCE end counts only when the locked localizer agrees (near the taught path end);
  //                   otherwise PAUSE (no 70 deg turn in the wrong place). After Resume the end is taken.
  //   restartLegs     the leg begins like a fresh start: stand, LiDAR start check (alignSearch +-0.8 m) -> new heading
  //                   offset + localizer pose. Not sure -> drive on with the gyro, never turn on a doubtful check.
  //                   Leg 9 = after the pole, on flat ground (leg 8 begins still tilted at the ramp end).
  // 26.11.: 0 since the log_056 route (legs 5-11 are taught-path legs, not landmark legs); set a bit to use them again
  uint32_t lmkPathLegs = 0;
  uint32_t lmkNeedEndLegs = 0;
  uint32_t restartLegs = 0;
  uint32_t parkLegs = (1u << 9);   // 27.11.: leg 10 = parking turn (180 deg, right, in steps) - see _nextLeg
  // 27.11. (log_063): then reverse to the wall behind (_parkReverse): stop at LiDAR rear <= parkRearCm + parkStopLeadCm
  // (the 2 s soft stop rolls ~0.2 m) or back US <= parkBackUsCm; speeds (0.15 does not move rover 7 on the ground)
  float parkRearCm = 38.0f, parkStopLeadCm = 20.0f, parkRevV = 0.25f, parkRevSlowV = 0.20f, parkMaxBackM = 8.0f;
  int parkBackUsCm = 27;
  // 26.11. (logs 042 / 055): the pitch/roll "level" was taken only at start(): box 10 read +0.5 deg at building B and
  // -2.5..-3 deg on the flat ground after ramp 3 -> 27-58 % of legs 9-10 counted as "on a slope" and the landmark legs
  // ignored the wall ahead. Now a new level is taken in every leg: median of the first relevelS of DRIVE, only if steady
  // (max - min <= relevelSpreadDeg); never on a ramp leg or the leg right after one (begins on the ramp end).
  // 26.11. (log_057): OFF - leg 5 of the log_056 route begins on the slope before ramp 3 (pitch +4.2, steady), that
  // became the level, the flat ground after it read 5-7 deg off -> the localizer saw "tilted" for 37 % of leg 5, lost
  // the path and missed the left turn into the ramp. It was only needed for the log_027 landmark legs 8-9 (gone).
  bool relevelLegs = false;
  float relevelS = 1.5f, relevelSpreadDeg = 1.5f;
  uint32_t yawLegs = 0;   // 26.11.: none (log_027 leg 15 is gone); the legs follow the path when the LiDAR match holds
  bool onYawLeg() const { return active() && ((yawLegs >> _i) & 1u); }
  uint32_t turnTimeoutMs = 12000;  // TURN between legs: after this long drive the leg anyway (no hang)
  float noTurnAfterTimeoutEff = 1.5f;  // 06.11. (log_029 deadlock): after a turn time-out drive at least this
                                       // far before the path may ask for a turn on the spot again
  // 06.11. (log_029 leg 2: weak LiDAR match in the open area, score 0.3-0.5, the path pursuit turned the
  // rover 50 deg off and still believed it was on the path): on HOLD legs the path may steer only
  // within +-holdTolDeg of the leg's IMU heading (ref = direction start -> end), and only with a match
  // score >= minPathScore - otherwise the rover holds the IMU heading (relative yaw).
  float holdTolDeg = 15.0f, minPathScore = 0.5f;
  // 27.11. (log_065): 0.5 -> 0.7 (logs 062-064: legs ended at 0.82-2.4 of their taught effort); a weak match (score
  // < minPathScore) needs pathEndWeakFrac - a relocation jump made "path end" at 0.48 and the route ran ahead
  float pathEndMinFrac = 0.7f;   // "path end" ends a leg only after this share of the taught effort
  float pathEndWeakFrac = 0.9f;
  // 06.11. (log_028: the rover hit a pole 10 m into ramp leg 2 - the whole 33 m leg had no obstacle
  // handling): ramp mode only while the rover is REALLY on the slope - pitch off the start level by
  // more than slopeDeg, and slopeHoldMs after that. Before / after the slope the obstacle handling is on.
  float slopeDeg = 4.0f;
  uint32_t slopeHoldMs = 3000;
  bool onRamp() const { return active() && ((rampLegs >> _i) & 1u) && _slopeMs != 0 && millis() - _slopeMs < slopeHoldMs; }
  // LOCAL PATH MODE (2026-10-29): every leg in its own frame (LocalPath.h, tools/localpath.py).
  // LocalLocalizer gives x / y / heading in the leg frame (gyro + speed guess + LiDAR matched
  // against the leg's taught walls); while it is LOCKED the rover steers to the taught path point
  // pathLookM ahead (pure pursuit, also in TURN) and the leg ends at the end of the taught path.
  // Not locked (8 scans without a match or none for 3 s): the leg-by-leg rules above take over
  // (wall / heading target, landmark + end scan). pathMode = false: always the leg-by-leg rules.
  bool pathMode = true;
  float pathLookM = 1.0f;     // (fixed look-ahead before 06.11.; now the minimum is lookMinM)
  // 06.11. ADAPTIVE LOOK-AHEAD (pure pursuit): Ld = lookTimeS * v (v = forward output, 1 m/s per unit),
  // limited to lookMinM..lookMaxM, + lookScoreM * (1 - match score): a weak LiDAR match looks further
  // ahead (its pose noise turns less into heading noise). Pure pursuit: damping 0.71 at every speed,
  // lateral error decays over ~Ld/v seconds - so Ld grows with v and the decay TIME stays the same.
  float lookTimeS = 3.0f, lookMinM = 0.6f, lookMaxM = 1.6f, lookScoreM = 0.6f;
  // CRAB: this rover moves crabDeg off its nose (+ = left). log_026 -5.6, log_027 -7.6 deg (tools/make_legs.py
  // GEO bias): pointing the nose at the carrot it settles tan(7.6) * Ld = 0.13 m right of the path
  // (log_028: 0.12-0.19 m right). The nose now aims -crabDeg further, so the TRACK goes to the carrot.
  float crabDeg = -7.6f;
  // the rest (other ground, battery): integral of the signed cross-track error, xteKi deg per m and s,
  // limited to +-xteIMaxDeg. Only while rolling forward on the locked path.
  float xteKi = 3.0f, xteIMaxDeg = 6.0f;
  float lookAhead() const { return _Ld; }
  float crossTrack() const { return _ey; }
  float xteI() const { return _xteI; }
  float lpStopM = 0.20f;      // path mode: a MAPPED wall in the rover-wide corridor (0.18 m) closer than this = stop
  bool lineLook = true;       // carrot pathLookM ahead on the STRAIGHT-LINE path (corners of the taught path,
                              // 12 cm Douglas-Peucker); false: carrot on the raw taught path
  bool pathOn() const { return _lpOn; }
  bool pathLocked() const;
  bool frontMapped(const Sensors& s) const;   // 24.11.: everything close ahead is a taught wall (no new points)
  bool _pathSteer() const;    // pathLocked() and not a yaw-only leg
  float _pathErrRad() const;
  const LocalLocalizer& local() const { return _lp; }
  float wallJumpDeg = 8.0f;   // wall reading this far off the median of the last 5: dropped
  float deadbandDeg = 4.0f;   // (arcs only) no correction inside +-4 deg
  float turnDoneDeg = 9.0f;   // TURN / avoid turns count as done inside this (> corrDeg)
  float kP = 0.004f;          // wheel difference per deg of error (10 deg -> 0.04)
  float kI = 0.002f;          // (arcs only) per deg*s
  float arcMax = 0.0f;        // 0 = no arcs: a wheel difference does not turn this rover (log_014)
  // tap and settle pivots (log_015/016: answer 0.25-0.5 s late, ~30 deg per short tap)
  float tapGain0 = 100.0f;    // deg turned per second of tap, start value (learned per run: tapGain())
  float tapAim = 0.8f;        // ask a tap for this share of the error (undershoot, never overshoot)
  uint32_t tapMinMs = 40, tapMaxMs = 300;   // tap length limits (a big turn = several taps)
  float boostMax = 0.05f, boostStep = 0.01f;   // no progress while rolling: + this per 0.25 s, up to max
  uint32_t settleMs = 600;    // after a tap: wait this long (and until < 12 deg/s) before deciding again
  float rollBackCmd = 0.2f;   // rolling backwards (obstacle ahead)
  float lookAheadM = 1.5f;    // wall legs: aim at the set distance this far ahead
  float maxApproachDeg = 15.0f;
  // 27.11. (log_064, leg 9): the K wall was 3.2-4.7 m away (taught 2.1 m) - beyond the old 3 m wall-fit range, so
  // the leg only held the IMU heading and ran 6 deg off the wall into the sand. Route wall legs fit up to this.
  uint16_t wallMaxCm = 450;
  float wallTolDeg = 20.0f;   // a wall line counts only if it runs within this of the leg direction
  float anchorGain = 0.05f;   // heading correction from the wall, per scan
  float avoidEnterM = 0.70f;  // corridor ahead shorter -> obstacle handling
  float avoidFreeM = 1.20f;
  bool autoAdvance = true;    // landmark not found -> go on with the next leg by itself (no pause)
  float advanceEffortFrac = 1.3f; // ... after this * taught effort (+2) on the leg
  float maxEffortFrac = 1.6f; // only with autoAdvance = false: more effort (+4) -> PAUSE
  uint8_t maxBackups = 5;     // stuck: back up and retry this often in one leg, then PAUSE
  float zoneWarnM = 15.0f;
  float pivotCmd = 0.35f;     // soft pivot (05.11.: Kethan's 90-degree turns at 0.35; was 0.5 taps)
  // rolling heading hold (05.11., Kethan's steerForward - it drove log_025 straight, hold error < 2.3 deg)
  float hKp = 0.045f;         // wheel difference per deg of error
  float hKd = 0.0015f;        // per deg/s of yaw rate (damping)
  float corrMax = 0.125f;     // |wheel difference / 2| limit (11.11.: 0.10 -> 0.125, Kethan's curve: inner wheel 50 %)
  float corrFrac = 0.50f;     // ... and at most this share of the speed (0.40 -> 0.50 = inner wheel at 50 %)
  float corrRate = 0.30f;     // the correction changes by at most this per second
  float pivotCoastS = 0.45f;  // pivot released |rate| * this early (rover lag + MotorShaper turn ramp-down)
  float pivotDoneDeg = 5.0f;  // pivot ends inside this (the rolling PD does the rest)
  float backupCmd = 0.20f;    // stuck: back up at this (was 0.3)
  float stuckMinOut = 0.15f;  // stuck check only counts while the REAL wheel output is at least this
  float stuckMinTurn = 0.20f; // ... or while pivoting with at least this REAL turn output (06.11.)
  float pivotBoostMax = 0.15f, pivotBoostRate = 0.05f;   // pivot not turning: + 0.05/s up to + 0.15 = 0.50 (06.11.;
                              // log_028: 0.45 did not turn it at the ramp foot, the teacher turned there at 0.50)
  uint32_t swingAfterMs = 4500;   // pivot at full boost and still not turning -> SWING turn: inner wheels stop,
                                  // outer wheels drive (log_028 ramp foot: pivot on the spot did not turn at all)
  float usStopCm = 50;        // front ULTRASONIC this close in 2 scans in a row -> obstacle handling (06.11.; 19.11.: 50, was 20)
  uint8_t stuckWindows = 3;   // scene unchanged in this many 2 s windows in a row -> back up (was 2)
  uint8_t boostAfterSteps = 8;  // no progress for 2 s while rolling -> gentle push (was 0.75 s)
  // arcMax = 0 -> no arcs at all: pivots only (if the field test shows arcs stall this rover)
  enum EndRule : uint8_t { RULE_OLD = 0, RULE_COMBINED = 1 };
  uint8_t endRule = RULE_COMBINED;  // how a leg ends (see LIDAR LOCALISATION above)
  bool staticFilter = true;         // obstacle handling only for NEW things ahead
  float lastResortFrac = 1.8f;      // RULE_COMBINED: no end found after this * taught effort + 2 -> next leg
  const RouteLocalizer& localizer() const { return _loc; }

  // ---- v3 (2026-11-10) -------------------------------------------------------------------------
  // START ALIGN: the rover may be put down facing ANY direction at the start. start() matches the
  // full-circle scan against the leg map (LocalLocalizer::alignSearch), sets the heading from it and
  // first TURNS on the spot to the leg direction, then drives. If the match is not clear (score /
  // margin too low) it falls back to the old rule (rover parallel to the wall) with a message.
  // TRIM (v3, 10.11., Arjun's DriveStraight): wheel trim l = v(1 - trim), r = v(1 + trim) while rolling, learned
  // from the steady heading correction. Start value set in cas_box_v3.ino (ROVER_TRIM_START). Kept across legs,
  // resumes and missions until the box restarts.
  float trim = 0.0f;
  bool trimLearn = true;
  float trimTauS = 15.0f, trimTauFastS = 6.0f, trimMax = 0.60f, trimLearnBelowDeg = 6.0f;
  float driveSeconds() const { return _driveS; }
  float trimLearnSeconds() const { return _trimS; }   // 19.11.: seconds the trim was learned (since boot)
  // TURN TACTICS (v3, 11.11. after log_030 - see RouteFollow.cpp): pivot full power, swing, shift, arc in a cycle
  bool turnTactics = true;            // false = the old soft pivot 0.35 -> 0.50 + swing
  float turnCmd = 0.50f;              // wheel command of swing / arc, and of the pivot when 0.35 does not turn it
  float kethanCmd = 0.35f;            // pivot first at Kethan's 0.35 (ramped 0 -> 0.35 in 1 s)
  // ROLLING START (12.11., logs 030-033 of rover 7): a pivot started FROM STANDSTILL turned >= 10 deg in 2 s in 8 of
  // 117 cases (7 %), started WHILE ROLLING in 21 of 41 (51 %) - e.g. log_033 738.9 s: rolling back at 0.3, pad pivot
  // -> 20 deg in 1 s. So every turn on the spot starts by rolling straight (back if the rear is free, else forward)
  // and goes into the 0.5 pivot WITHOUT stopping (MotorShaper: the speed fades over 2 s while the turn rises in 1 s).
  // Stalls -> roll again (alternating back / forward, the rover stays within the back / forward limits).
  // K-TURN (12.11. evening, logs 030-034 of rover 7, 7900 samples sorted by wheel direction):
  //   wheels OPPOSITE (every pivot, also the rolling start): median 0 deg/s - rover 7 cannot turn on the spot
  //   both wheels FORWARD  (curve): 16.7 deg/s per unit w = (r - l)/2   -> l 0.05 / r 0.50: ~3.6 deg/s
  //   both wheels BACKWARD (curve): 31 deg/s per unit w (16 samples)     -> l -0.50 / r -0.05: ~7 deg/s
  // So every turn is made like a car turning in a narrow street: curve FORWARD with the inner wheel slow, then curve
  // BACKWARD with the other wheel slow (the body keeps rotating the same way), and again - until the heading is
  // reached. Each piece ends after kSegM, kSegMs or when the LiDAR / ultrasonics see no room in that direction; the
  // pieces alternate so the rover stays near where it started (net forward / backward limits as for the shifts).
  // Used for the start turn, every corner pivot and the obstacle detour turns. kTurn = false: the old tactics.
  bool kTurn = true;
  // 18.11. (course rule 17.11., gearbox protection): NO turning on the spot - every direction change is a curve while
  // driving. curveOnly = true: a turn that needs more than the rolling PD goes straight to the K-turn curves (both
  // wheels the same direction); pivot / swing / roll+pivot / shift are never used.
  bool curveOnly = true;
  float kOuter = 0.50f;               // outer wheel (the fast one), forward or backward
  float kInner = 0.05f;               // inner wheel, same direction (>= 0: never opposite = never a pivot)
  float kSegM = 0.40f;                // one piece at most this far (odometry, 1 m/s per unit)
  uint32_t kSegMs = 5000;             // ... or this long
  bool rollStart = true;
  float rollCmd = 0.30f;              // rolling speed before the pivot
  float rollMaxM = 0.50f;             // roll at most this far per try (it pivots as soon as it moves at rollGoV)
  float rollGoV = 0.28f;              // ... or once the real wheel speed reaches this
  uint16_t rollCarryMs = 0;   // 13.11.: 0 = pivot asked at once, the MotorShaper fades the speed (= log_037 M15)
  // was 1200:        // the pivot keeps the rolling speed in its command, fading to 0 over this
  float _rollV = 0;                   // signed rolling speed carried into the pivot (0 = pivot from rest)
  float climbPow = 0.40f;             // stuck: push straight at this first (no steering), climbMs long
  uint32_t climbMs = 3000;            // (the 2 s S-curve speed ramp needs ~2 s to reach climbPow)
  uint32_t turnTacticTimeoutMs = 60000;   // a turn may take this long before it gives up (start: PAUSE)
  float backShiftMaxM = 0.6f;         // backward shifts in total per turn (no drop sensor behind the rover)
  float fwdMoveMaxM = 1.5f;           // forward rolling (swing / shift / arc) in total per turn on the route
  float fwdMoveStartM = 0.6f;         // ... in the start turn (raised path at M1, edge not visible to the LiDAR)
  uint8_t startMaxRounds = 4;
  float startDoneDeg = 15.0f;         // start turn: this close to the leg direction after a check = drive
  uint32_t startTurnTimeoutMs = 150000;   // start turn: give up (PAUSE, turn by hand) after this (log_031: ~5 deg per shift)
  // a locked path localizer must agree that the leg is (nearly) done: s >= lpEndAgreeFrac * sEnd - 0.5 m (log_031)
  float lpEndAgreeFrac = 0.85f;         // start alignment: turn steps (<= 90 deg each) + LiDAR re-checks
  bool tacticActive() const { return active() && _pivoting && turnTactics; }   // -> fast turn ramp in MotorShaper
  bool wheelRamp() const { return (tacticActive() && _tk == TK_KTURN) || (_state == AVOID && _avPhase == 41); }   // v6: backward curve too   // -> MotorShaper ramps each WHEEL (keeps the curve)
  const char* tacticMsg() const { return _tacMsg; }
  // ---- v4 (11.11., tools/v4_test/obstacle_sim on the real route code) ------------------------------
  // SMART OBSTACLES. v3 saw a NEW thing on the locked path only inside hardStopM (0.45 m) and then braked with the
  // 2 s S-curve -> it rolled into it (sim: 3 of 3 boxes hit). Now:
  //   - new things (not in the leg map) count from avoidPlanM (1.5 m) ahead; the stop for them is FIRM (firmStop()
  //     -> MotorShaper 0.6 s); mapped walls keep the old rules (lpStopM / hardStopM / END_FRONT)
  //   - standing, the obstacle is watched: it MOVES (person, bike: > avMoveM in staticMs) -> wait as before (avoid
  //     after 3 s at the latest); it STANDS still for staticMs -> detour at once
  //   - detour: the obstacle's width from the scan (neighbouring points < 0.3 m apart), side with the smaller
  //     shift whose 45 deg diagonal and side ultrasonic are free (never right on noRight legs), OUT at 45 deg until
  //     beside it + avClearM, PASS parallel until it is behind, then BACK to the line (path mode: the pursuit
  //     takes the rover back; otherwise 45 deg back until the dead-reckoned offset is gone)
  //   smartAvoid = false: the v3 handling (wait 3 s, sidestep to the closest free direction).
  bool smartAvoid = true;
  float avoidPlanM = 1.5f;        // new things closer than this in the corridor (2 x 0.35 m): obstacle
  float avClearM = 0.45f;         // rover centre passes this far from the obstacle edge (half width 0.15 + 0.3)
  float avMinClearM = 0.30f;      // ... at least this far when the diagonal runs out of room (wall beside)
  float avMaxShiftM = 1.2f;       // never further off the line than this (open edges: noRight legs, raised path)
  float avMaxWidthM = 1.5f;       // wider: a wall across -> no detour (wait / pause as before)
  float avAngleDeg = 30.0f;          // 12.11.: was 45 - the K-turn back to parallel drifts ~0.2 m towards the
                                      // obstacle; 30 deg needs less turning (sim: box on the path passes, 45 hit it); OUT / BACK direction vs the line
  float avMoveM = 0.10f;          // obstacle moved more than this between scans while we stand: it is moving
  uint32_t staticMs = 750;        // ... not moved for this long: static -> detour now
  float avMinScore = 0.6f;        // early (1.5 m) detection only with a LiDAR match at least this good
  bool firmStop() const { return active() && _state == AVOID && (_avPhase < 10 || _avPhase >= 20); }   // v5: + the simple detour   // 18.11.: emergency only (obstacle stop), back-up ramps softly   // -> MotorShaper.firm

  // ---- v5 VINAY (16.11.): SIMPLE SENSOR DETOUR - LiDAR + ultrasonics, 35 cm rule ----------------------------------
  // Replaces the v4 smart detour / smooth reroute while simpleAvoid = true (cas_box: SIMPLE_AVOID). Problem it fixes:
  // the v4 detour needed a free 30-45 deg diagonal, did not find one close to the obstacle, backed up ~0.5 m, failed
  // again and PAUSED ("moves slightly back and stops"); the SafetyGuard hold in DRIVE also only backed up.
  //   DETECT  front distance = min(front ultrasonic, LiDAR rover-wide corridor - lidarNoseCm). Below obsSlowCm the
  //           speed is capped at obsSlowV; at obsStopCm + obsLeadCm (both sensors agree: 1 scan, one sensor: 2 scans)
  //           -> FIRM stop, the rover stands ~obsStopCm (35 cm) from the obstacle. Walls of the taught map (path
  //           locked, nothing NEW ahead) keep the old rules (END_FRONT leg end, lpStopM) - corners are not obstacles.
  //   20 STOP     stand, wait obsWaitMs (a person walks on -> drive on)
  //               v5b: look at the obstacle with the LiDAR (front face distance, left / right edges) and work out
  //               per side: the sideways shift (edge + half body + 35 cm) and the turn ANGLE that clears the near
  //               corner, atan(edge + half body + obsCornerM, face distance), obsMinDeg..obsMaxDeg - so a thin pole
  //               gets ~30 deg, a 0.5 m box ~45 deg, a wide one ~60 deg. The LiDAR must show that diagonal free,
  //               else 15 deg steeper (up to 90). Side: the smaller shift, then the flatter angle, then more room;
  //               never right on noRight legs. No way: back up once and look again, then PAUSE.
  //   21 TURN     turn to that angle (gyro, the normal turn tactics)
  //   22 OUT      diagonal at that angle. Corner comes within the stop distance -> 15 deg steeper (back to 21).
  //               Done when the LiDAR shows the line direction free for body + 35 cm (2 scans), or for body
  //               + 12 cm with the planned shift reached
  //   23 ARC IN   curve back to the line direction while rolling (no stop / pivot); too close -> straight on.
  //               It starts early by the outward drift of the curve, R (1 - cos angle); R is learned per curve
  //   24 PASS     parallel, keep 35 cm: side distance = LiDAR beside the rover + side ultrasonic (median of 3,
  //               single wrong echoes dropped). Passed = the LiDAR sees nothing beside the body from behind the
  //               rear to 0.3 m ahead (2 scans) and the filtered ultrasonic agrees
  //   25 RETURN   curve to the line at obsBackDeg (45, 30 if the LiDAR shows 45 blocked), flattening out as
  //               the offset goes (path localiser cross-track, else dead reckoning) -> DRIVE lined up
  //   29 HOLD     something ahead during the detour (< obsStopCm + obsLeadCm): stand; free again -> go on,
  //               still blocked after obsBlockedWaitMs -> PAUSE
  bool simpleAvoid = true;
  // ---- v6 (19.11., Vinay): CURVED AVOIDANCE for the 25 cm box rover, curve-only (no turn on the spot, no K-turn) ----
  // Replaces the stop-and-turn detour (phases 19-29) while curveAvoid = true. Every LiDAR scan in DRIVE:
  //   lanes   headings around the route direction (+-gapMaxDevDeg, 4 deg steps) are scored by how far a strip as
  //           wide as the BODY is free: first body + obsSideKeepCm (35 cm gap, open ground), else body + gapMarginM
  //           (corridors / narrow gaps: building walls alongside never block a strip only as wide as the body)
  //   steer   the route direction if its strip is free for gapNeedM; else the nearest free heading - the rover
  //           CURVES there while rolling (wheel difference only, never opposite wheels), and back when it is past
  //   back    on legs without the taught path the sideways offset is dead-reckoned and steered back to the line
  //   walls   nothing free anywhere (a wall across), a leg that ends at a wall ahead, mapped walls on the taught
  //           path, ramps: no deviation - the old rules / the 35 cm stop handle them
  //   close   something inside obsStopCm (35 cm) in the body strip anyway (it stepped in late): soft stop, wait
  //           obsWaitMs (people), then a BACKWARD CURVE (both wheels backward, the nose swings to the free side)
  //           until a free heading is reachable by a forward curve, then curve past (phases 40-42)
  bool curveAvoid = true;
  float gapMarginM = 0.05f;       // narrowest pass: body + this on each side
  float gapLookM = 3.5f;          // LiDAR look-ahead of the lanes
  float gapNeedM = 3.0f;          // a heading counts as free when its strip is free this far (from the LiDAR):
                                  // early - a rolling curve of the rover turns only a few deg per second
  float gapMaxDevDeg = 60;
  // 27.11. (log_064 leg 1: detours went right towards the drop): equal bends left / right used to go RIGHT (the
  // search ran from the right). Now the side with more LiDAR room (beside the route, > 0.2 m more) is preferred by
  // gapSideBiasDeg; Leg::noRight = never right of the route (edges / drops the LiDAR cannot see).
  float gapSideBiasDeg = 8.0f;        // furthest it looks off the route direction
  float gapWideMaxDeg = 25;       // a 35 cm-gap heading beats a narrow gap only within this of the route
  float obsGapPastM = 0.8f;
  float gapLaneMaxM = 1.2f;       // narrow-gap lanes: sideways search range       // narrow-gap mode: the heading's strip must be free this far past the obstacle
  float gapEngageDeg = 30;        // avoidance only while the route direction is within this of the nose
  float gapRollMaxDeg = 22;       // heading error given to the controller while avoiding (< pivotDeg: rolls)
  float gapRetDegPerM = 30, gapRetMaxDeg = 15;   // back to the line (legs without the taught path)
  float gapRevMaxM = 1.2f;        // close obstacle: backward curve at most this far
  uint8_t gapRecoverMax = 3;
  float obsCornerStopM = 0.015f;  // v6: something this close beside the front half of the body -> stop at once      // close-obstacle recoveries per obstacle before a PAUSE
  bool gapActive() const { return _gapDeviating; }
  uint32_t gapRecentMs = 4000;    // after avoiding: this long no K-turn back to the path, a curve instead
  float gapDevDeg() const { return _gapDevDeg; }
  float obsStopCm = 35;           // stop this far from the obstacle (rover front)
  float obsLeadCm = 6;            // trigger this much earlier: what the rover still rolls during the firm stop
  float obsSlowCm = 100;          // slow down below this
  // 19.11. (log_048): the front ultrasonic sees the FLOOR at 40-70 cm about half the time (LiDAR > 1.5 m free) -> 2 such
  // readings started a detour (43 AVOID starts, back and forth). Alone (LiDAR does not agree) it now counts only closer
  // than this (a shoe the LiDAR plane misses is still caught here).
  float obsUsAloneCm = 25;
  // 19.11. (log_048, leg 6 D-E): 6 min pressed against the left wall (LiDAR left 22 cm, US 14 cm), the side stop and the
  // detour fought it. In DRIVE the heading is now bent away from a wall closer than wallKeepM to the body side
  // (wallKeepGain deg per metre too close, at most wallKeepMaxDeg). 0 = off.
  float wallKeepM = 0.12f, wallKeepGain = 80.0f, wallKeepMaxDeg = 12.0f;   // sim leg 6: 0.25 m bent it off near the ramp columns
  float obsSlowV = 0.20f;         // ... to this (and the detour speed). Rover 7 moves at >= 0.15
  float obsSideKeepCm = 35;       // side gap held by the side ultrasonic while going round
  float obsSeeCm = 75;            // side reading below this = obstacle beside the rover
  float obsSideNeedM = 0.70f;     // a side counts as free with this much room (LiDAR 90 deg corridor + side US)
  float obsHalfWM = 0.175f;       // v6: 25 cm box -> body 0.125 + 5 cm (was 0.20 for rover 7's 0.26 m)        // half width of the corridor checked ahead / to the side (rover 0.13 + 7 cm)
  float obsMinDeg = 30, obsMaxDeg = 75;   // v5b: turn-out angle limits (from the obstacle's shape, 90 only if needed)
  float obsBackDeg = 45;          // angle back to the line (30 if the LiDAR shows 45 blocked)
  float obsCornerM = 0.20f;       // the diagonal passes the near corner with this much room (gives ~35 cm body gap)
  float obsArcDeg = 20;           // heading error per step while curving (below pivotDeg: rolls, does not pivot)
  float obsSeeWithinM = 1.5f;     // never seen beside after this far: count it as passed
  float obsMaxShiftM = 1.8f;      // wider than this sideways: PAUSE (wall across the way)
  float obsKeepKp = 0.5f, obsKeepMaxDeg = 15;   // side keeping: deg per cm off obsSideKeepCm, limit
  uint32_t obsWaitMs = 1500, obsBlockedWaitMs = 4000, obsTurnTimeoutMs = 45000, obsMaxMs = 240000;   // 19.11.: K-turns are slow
  float obsRoomM = 0.90f, obsRoomBackM = 0.60f;   // 19.11. curve-only: reverse straight until the face is this far (at most)
  float obsKturnLossM = 0.10f;      // 19.11.: a K-turn back to the line direction ends this much nearer the line
  uint8_t obsReplanMax = 2;         // 19.11.: blocked while passing -> turn out again this often before a pause
  float obsMinKeepCm = 15;          // 19.11. corridor mode: smallest side gap tried when 35 cm does not fit
  float obsSingleM = 0.50f;         // 19.11.: closer than this a single LiDAR point counts as an obstacle ahead
  uint32_t obsStallMs = 3000;       // 19.11.: asking forward but not moving this long -> back a little, out again
  float obsLaneLookM = 1.0f;        // 19.11.: the lane beside the obstacle must be free this far past its face
  float lidarNoseCm = 12.5f;         // LiDAR centre -> rover front (cm). MEASURE on the rover (cas_box: LIDAR_NOSE_CM)
  float lidarSideCm = 12.5f;         // LiDAR centre -> rover side (cm). MEASURE on the rover (cas_box: LIDAR_SIDE_CM)
  float lidarRearCm = 12.5f;         // LiDAR centre -> rear bumper (cm). MEASURE on the rover (cas_box: LIDAR_REAR_CM)
  float obsFrontCm() const { return _obsFront; }   // last fused front distance (page)
  float obsSideCm() const { return _obsSide; }     // last side distance used for keeping (page, 999 = none)
  // SMOOTH REROUTE (v4b, 11.11.): a static thing seen EARLY (new points up to rrLookM ahead on the locked path,
  // 2 scans) is passed on a planned smooth curve without stopping:
  //   lateral offset y(s) along the line (dead-reckoned frame of the moment it was seen):
  //     ramp out   y = y0 + (G - y0) * u^2 (3 - 2u),  u = (s - s0) / Lr      (smoothstep: no kink, no turn on the spot)
  //     hold       y = G                                until the obstacle is 0.6 m behind and nothing is beside
  //     ramp back  y = G * (1 - u^2 (3 - 2u))           back onto the line, then the normal leg rules
  //   G = obstacle edge + avClearM (+ LiDAR bin) on the side with the smaller shift; the ramp is as long as the
  //   rover's rolling turn rate needs: max curvature of the smoothstep = 6 |G| / Lr^2 <= w / v
  //     -> Lr = sqrt(6 |G| v / w)    (|G| 0.7 m, v 0.20 m/s, w 6 deg/s -> 2.8 m)
  //   The whole curve is checked against EVERY LiDAR point (rover 0.4 x 0.26 m swept along it, rrClearM from the
  //   centre line), at the plan and again on every scan; blocked later -> firm stop and the 45 deg detour above.
  //   The rover follows the curve by pure pursuit (rrLd ahead) at rrSpeed, wheel difference up to rrCorrMax.
  //   Not enough room before the obstacle (seen too late) -> the stop + 45 deg detour above.
  bool smoothReroute = true;
  float rrLookM = 3.5f;           // look this far ahead for new things
  float rrSpeed = 0.20f;          // speed on the curve (slower = tighter curve for the same turn rate)
  float rrTurnDegS = 6.0f;        // the curve never needs more than this turn rate while rolling (rover: ~6-8 deg/s)
  float rrMinRampM = 1.5f, rrMaxRampM = 4.0f;
  float rrClearM = 0.30f;         // swept rover centre line >= this from every LiDAR point (body half width 0.13)
  float rrCorrMax = 0.20f;        // wheel-difference limit on the curve (rolling hold: corrMax 0.10)
  float rrLd = 0.8f;              // pure-pursuit look-ahead on the curve
  float rrMaxWidthM = 0.8f;       // wider obstacles: the stop + 45 deg detour (sim: walls beside are handled better)
  // a curve needs the obstacle >= 0.7 * ramp + 0.5 m ahead (the rover lags the curve); seen later: stop + 45 deg detour
  bool rerouting() const { return (_state == AVOID && _avPhase >= 10) || _gapDeviating; }   // v6: avoiding curve -> bigger wheel difference
  uint8_t avoidPhase() const { return _state == AVOID ? _avPhase : 0; }   // 1-3 detour, 10-12 smooth reroute, 20-29 v5

  float _roll0 = 0;                   // (12.11.) roll at the leg start: side lines are checked against the ROLL
  bool alignStart = true;
  float alignMaxOffM = 2.0f;          // rover put down within this of the taught start point (18.11.: random start
                                      // pose in front of the post office - was 0.6)
  float alignDeg() const { return _alignDeg; }       // how far the rover was turned off at the start
  float alignScore() const { return _alignSc; }
  float alignSecond() const { return _alignSec; }
  // LANDMARK LEGS (LandmarkLeg.h): leg i is driven on landmark lines (phases by distance) instead of
  // the taught-path match. Up to 4 legs. lmkMPerEff = metres per unit of effort (log_027: 1.03).
  // lenM = the leg length in the table's metres (teaching odometer); > 0: while the localizer is locked the phase
  // position is the localizer's s scaled to it (effort can be 40 % off: log_031), else effort metres
  bool setLandmarkLeg(uint8_t legIdx, const LmkPhase* ph, uint8_t n, float lenM = 0);
  float lmkMPerEff = 1.03f;
  float lmkTiltDeg = 3.5f;            // pitch off the start level by more: lines AHEAD / BEHIND not trusted (ramp)
  float lmkParaExtraDeg = 10.0f;      // 12.11.: side lines accepted this much less parallel (LandmarkLeg)
  float lmkRollDeg = 5.0f;            // 12.11.: roll off the start level by more: SIDE lines not trusted either
  bool onLmkLeg() const { return active() && _lmkOn; }
  const LandmarkLeg& lmk() const { return _lmk; }
  float legMetres() const { return _eff * lmkMPerEff; }
  const char* lmkWhy() const { return _lmkWhy; }

  static uint8_t count();
  static const Leg& leg(uint8_t i);

  // Start at leg i (0-based). anchor: set the heading from the wall / rover direction now
  // (rover placed parallel to the leg). Returns false + msg if it cannot start.
  // singleLeg: drive only leg i and stop at its end (segment test); else continue to the last leg
  bool start(const Sensors& s, const WallFollow& wf, uint8_t i, float speed, bool singleLeg = false);
  bool resume(float speed);               // continue the current leg (heading kept: gyro runs on)
  void stop() { _state = OFF; }
  void pause(const char* why);
  bool active() const { return _state != OFF && _state != DONE && _state != PAUSED; }
  bool canResume() const { return _hasRun && _state != DONE; }
  uint8_t state() const { return _state; }
  uint8_t legIndex() const { return _i; }
  float effortFrac() const { return _cur().effort > 0 ? _eff / _cur().effort : 0; }
  float routeYaw(const Sensors& s) const { return _wrap(s.yawDeg() + _off); }
  const char* msg() const { return _msg; }
  uint8_t forcedSwitches() const { return _forced; }
  bool singleLeg() const { return _stopAfter != 255; }
  const char* lastEnd() const { return _endInfo; }  // how the last leg ended (status / page)
  uint8_t logByte() const { return _hasRun ? (uint8_t)(0x80 | ((_state & 7) << 4) | ((_i + 1) & 15)) : 0; }
  uint8_t logLeg() const { return _hasRun ? (uint8_t)(_i + 1) : 0; }   // full leg number (> 15 legs)

  // every loop pass while active: writes l / r
  void steer(const Sensors& s, const WallFollow& wf, float& l, float& r);
  // every loop pass: what really went to the rover (after guard + shaper) -> effort
  void feedOutput(float l, float r);

private:
  const Leg& _cur() const { return leg(_i); }
  void _step(const Sensors& s, const WallFollow& wf);
  void _nextLeg(const Sensors& s, const char* how);
  bool _wallNow(const Sensors& s, const WallFollow& wf, int8_t side, float& d, float& dirYaw, float& rms, uint8_t& pts) const;
  float _corridor(const Sensors& s, float phiDeg, float half, float look) const;
  bool _wallAcross(const Sensors& s, float maxM) const;   // 27.11.: a wall ACROSS the way (not one foot / post)
  float _front(const Sensors& s) const;
  float _scanChangeCm(const Sensors& s) const;
  void _drive(float e, float v, bool fwd);      // sets the target; _control drives
  void _control(const Sensors& s);              // every loop pass: heading controller
  static float _wrap(float a) { a = fmodf(a + 180.0f, 360.0f); if (a < 0) a += 360.0f; return a - 180.0f; }
  static float _clamp(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }

  volatile uint8_t _state = OFF;
  uint8_t _resumeState = DRIVE;
  bool _hasRun = false, _overrun = false;
  uint8_t _forced = 0;                // legs switched without their landmark (status / log)
  uint8_t _i = 0;
  float _speed = 0.2f, _off = 0, _eff = 0, _I = 0;
  float _l = 0, _r = 0;               // command of the current step
  uint32_t _lastStep = 0, _lastOut = 0, _stateMs = 0;
  // end detection
  uint8_t _seen = 0, _lostN = 0, _foundN = 0, _lowN = 0;
  // controller
  bool _ctrlOn = false, _tgtFwd = true, _pivoting = false;
  float _hc = 0;                      // rolling heading correction (half wheel difference, + = left)
  float _outV = 0;                    // forward output that really went to the rover (feedOutput)
  float _outW = 0;                    // turn output that really went to the rover (feedOutput)
  float _pivBoost = 0;                // extra pivot command while the pivot does not turn the rover
  uint32_t _slopeMs = 0;              // last time the pitch was off the start level (ramp mode)
  float _Ld = 1.0f, _ey = 0, _xteI = 0, _peF = 0;   // look-ahead (m), cross-track (m, + = left), integral (deg), filtered pursuit error
  bool _peInit = false;
  float _noTurnEff = -99;             // effort at the last turn time-out
  float _tgtYaw = 0, _tgtV = 0, _yawDesF = 0, _rate = 0, _prevYaw = 0;
  uint32_t _ctrlMs = 0, _pivotMs = 0, _unstallUntil = 0, _arcMs = 0;
  bool _yawDesInit = false;
  float _drift = 0;                   // rover's own turn rate while rolling straight (deg/s, + = left)
  uint32_t _pivotEndMs = 0;
  int8_t _pivotDir = 1;
  uint8_t _tapPh = 0;                 // 0 idle, 1 tap running, 2 settling
  int8_t _tapDir = 1;
  uint32_t _tapUntil = 0, _tapEndMs = 0;
  float _tapYaw0 = 0, _tapReq = 0, _tapGain = 100.0f;
public:
  float tapGain() const { return _tapGain; }
  // which block set the wheels last (debug / log): the heading controller rolling straight, a turn tap,
  // the settle wait after a tap, standing (turn without forward), or the route step itself (obstacle
  // wait, back up, pause)
  enum Why : uint8_t { WHY_NONE = 0, WHY_ROLL = 1, WHY_TAP = 2, WHY_SETTLE = 3, WHY_STAND = 4, WHY_STEP = 5 };
  uint8_t why() const { return _why; }
  float ctrlErrDeg() const { return _ctrlErr; }   // heading error the controller works on (+ = left)
  float driftDegS() const { return _drift; }
  float boost() const { return active() ? _boost : 0.0f; }   // -> MotorShaper::boost every loop pass
  float progressCm() const { return _progCm; }               // median LiDAR change per 0.25 s (99 = unknown)
private:
  uint16_t _prevScan[Sensors::kLidarPoints] = {0};
  uint8_t _noProgN = 0;
  float _boost = 0, _progCm = 99;
  uint8_t _why = 0;
  float _ctrlErr = 0;
  // wall reading filter (median of 5)
  float _wyBuf[5], _dBuf[5];
  uint8_t _wN = 0, _wI = 0, _wRej = 0;
  bool _wallFilter(float& d, float& wy);
  float _stepRy = 0;                  // route yaw (snapshot) of the current _step
  // local path mode
  LocalLocalizer _lp;
  bool _lpOn = false;
  float _lpYawPrev = 0, _lpEffAcc = 0, _pitch0 = 0;
  float _pitchStart = 0;              // 26.11.: level at start() - the localizer's tilt gate always uses this one
  uint32_t _lpOkMs = 0;
  float _fr[4] = {9, 9, 9, 9}; uint8_t _frN = 0, _frI = 0;
  // obstacle handling
  uint8_t _avPhase = 0, _avTries = 0; float _avTarget = 0, _avEff = 0, _passEff = -99;
  // v4 smart obstacle handling
  bool _avMid = false;               // re-planning in the middle of a detour (keep line + offset)
  float _avGoal = 0, _avGoalMin = 0;
  int8_t _avDir = 1;                  // +1 left, -1 right
  float _avBase = 0, _avShift = 0, _avAlong = 0, _avLat = 0, _avS = 0, _avEffPrev = 0;   // line yaw (deg); shift, obstacle along, offset (+ left), progress (m)
  float _avObA = 99, _avObL = 0, _avBackS = 0;   // obstacle (front, centre m) at the last scan; progress at BACK start
  uint32_t _avStillMs = 0;
  bool _avPlan(const Sensors& s, const Leg& L, float& shift, int8_t& dir, float& along);
  bool _avCluster(const Sensors& s, float& front, float& lo, float& hi) const;
  // v4b smooth reroute
  struct Curve { float s0, y0, g, l, sb, l2; };   // see SMOOTH REROUTE; sb < 0 = still holding
  Curve _rc = {0, 0, 0, 1, -1, 1};
  uint8_t _rrN = 0;
  static float _curveY(const Curve& c, float sx);
  bool _rrFree(const Sensors& s, const Curve& c, float rel, float sFrom, float sTo, float clear) const;
  bool _rrTry(const Sensors& s, const Leg& L, float ry, float front, float latL, uint32_t now);
  void _rerouteStep(const Sensors& s, const Leg& L, float ry, uint32_t now);
  bool _besideLine(const Sensors& s) const;
  float _lineYaw(const Leg& L, float ry) const;
  bool _avPhase0Out() const;
  int8_t _avBackDir = 0;
  bool _rrEdges(const Sensors& s, float front, float latHintCw, float& lo, float& hi) const;
  float _rrObY = 0;
  void _avoidSmart(const Sensors& s, const Leg& L, float ry, uint32_t now);
  float _newAhead(const Sensors& s, float look, float* lat);
  // v5 simple sensor detour
  bool _obsDetect(const Sensors& s, const Leg& L, uint32_t now, float& vCap);
  void _obsStart(uint32_t now, bool force, const char* why);
  void _avoidSimple(const Sensors& s, const Leg& L, float ry, uint32_t now);
  float _obsFrontCmNow(const Sensors& s, bool* agree, float half = 0, float* lidOut = nullptr) const;   // half 0 = obsHalfWM
  float _obsSideCmNow(const Sensors& s, int8_t os, float alMax) const;
  float _obsLidarSide(const Sensors& s, int8_t os, float alMin, float alMax, float latMax, uint8_t* n) const;
  float _obsLaneFree(const Sensors& s, float relDeg, float offM, float half, float look) const;
  // v6 curved avoidance
  float _gapFree(float devDeg, float half, float look) const;
  float _gapSide(int8_t os) const;
  bool _gapCentre(float relDeg, float look, float& lo) const;
  float _gapLaneFree(float relDeg, float offM, float half, float look) const;
  bool _gapPick(float e, float half, float need, float& out) const;
  void _gapSteer(float& e, float& v);
  void _avoidCurve(const Sensors& s, const Leg& L, float ry, uint32_t now);
  bool _gapOk = false, _gapDeviating = false;
  float _wallBias = 0;   // 19.11.: wall keep-off (deg, + = left), set by _obsDetect each DRIVE step
  int8_t _gapPref = 0;   // 27.11.: preferred detour side this step (+1 left, -1 right, 0 none)
  float _gapDevDeg = 0, _gapLat = 0, _gapOdo = 0, _gapRevOdo = 0, _gapRecEff = 0;
  int8_t _gapRevDir = 0;
  uint8_t _gapRec = 0;
  uint32_t _gapLastMs = 0, _gapDevMs = 0;
  float _gapLineYaw = 0, _gapCrab = 0, _gapRefYaw = 0;
  bool _gapRefOn = false, _gapLaneOn = false, _gapBaseOn = false;
  float _gapBaseYaw = 0;
  bool _gapRecent(uint32_t now) const { return _gapLastMs && now - _gapLastMs < gapRecentMs; }
  void _obsUsUpdate(const Sensors& s);
  void _obsUsReset(const Sensors& s);
  float _usHist[2][3] = {{999, 999, 999}, {999, 999, 999}}, _usSide[2] = {999, 999};
  uint8_t _usHi = 0, _obsClrN = 0;
  bool _usReset = true;
  bool _obsRoomDone = false;                        // 19.11.: curve-only room reversal done for this obstacle
  uint8_t _obsReplanN = 0;
  float _keepCm = 35;                               // 19.11.: side gap of this detour (corridor mode)
  uint32_t _obsStallMs = 0;
  float _obsStallOdo = 0;                           // 19.11.: stall check window (odometry)
  uint8_t _obsRevRet = 20;                          // phase after the reversal (19)
  float _obsRevM = 0.6f, _obsRevOdo = 0;
  float _arcR = 0.8f, _arcLat0 = 0, _arcAng0 = 0;   // v5b: curve radius (m), learned on every curve back
  float _avAng = 45, _avRet = 45;
  int8_t _obsChooseSide(const Sensors& s, const Leg& L);
  float _obsKeep(float sideCm, int8_t os, bool awayOnly) const;
  bool _obsForce = false, _obsSeen = false;
  uint8_t _obsMissN = 0, _obsRet = 0, _obsResumePh = 0;
  float _obsLeg0 = 0, _obsClearAt = -1, _obsFront = 999, _obsSide = 999;
  uint32_t _obsT0 = 0, _obsHoldMs = 0;

  // stuck detection
  uint16_t _ref[Sensors::kLidarPoints];
  float _refYaw = 0; uint32_t _refMs = 0; bool _refMoving = false; uint8_t _stuckN = 0, _backups = 0;
  uint32_t _backupUntil = 0;
  // LiDAR localisation along the taught route
  RouteLocalizer _loc;
  uint16_t _scan[Sensors::kLidarPoints];
  char _msg[48] = "";
  uint8_t _stopAfter = 255;           // segment test: stop after this leg (255 = run to the end)
  char _endInfo[72] = "";
  // v3
  float _alignDeg = 0, _alignSc = 0, _alignSec = 0;
  float _driveS = 0;
  float _trimS = 0;   // 19.11.
  // v3 turn tactics + start turn
  enum Tac : uint8_t { TK_NONE = 0, TK_PIVOT = 1, TK_SWING = 2, TK_SHIFT = 3, TK_ARC = 4, TK_REST = 5, TK_ROLL = 6, TK_KTURN = 7 };
  uint8_t _tk = TK_NONE, _tkNext = 0, _tkCycles = 0, _floorH = 0;
  uint32_t _tkMs = 0, _tkWinMs = 0, _settleMs = 0;
  float _tkYaw = 0, _shiftM = 0, _netM = 0, _stepTgt = NAN;
  uint32_t _dropMs = 0;
  uint8_t _usN = 0; int _usFirst = 0;   // 19.11.: front ultrasonic plausibility (rain)
  bool _vetoLogged = false;
  float _settleYaw = 0;
  bool _shiftBack = false, _floorOk = false, _startTurn = false, _tkBoosted = false;
  float _odo = 0;                     // signed travel (forward +, backward -), effort units: detour dead reckoning
  uint32_t _guardMs = 0;             // since when the safety stop holds the wheels while DRIVE asks forward
  int8_t _kDir = 0;                   // K-turn: direction of the last segment (+1 forward, -1 backward, 0 none yet)
  float _kYaw0 = 0;                   // K-turn: gyro at the start of the turn (progress check)
  uint8_t _climbN = 0;
  uint32_t _climbUntil = 0;
  uint8_t _startRounds = 0;
  char _tacMsg[32] = "";
  bool _floorAhead(const Sensors& s);
  bool _rearFree(const Sensors& s) const;
  void _tacStart(uint8_t t, const Sensors& s, uint32_t now, bool first);
  bool _kFwdRoom(const Sensors& s, float margin) const;
  bool _kBackRoom(const Sensors& s, float margin) const;
  void _tactic(const Sensors& s, float dt, uint32_t now);
  void _startTurnStep(const Sensors& s, float ry, uint32_t now);                  // rolling time since boot (trim learning: fast for the first 40 s)
  LandmarkLeg _lmk;
  bool _lmkOn = false;
  const char* _lmkWhy = "";
  struct LmkSlot { uint8_t leg; const LmkPhase* ph; uint8_t n; float lenM; };
  LmkSlot _lmkSlot[16];
  uint8_t _lmkN = 0;
  float _lmkLen = 0;
  uint32_t _parkMs = 0;              // 27.11.: park reverse running since (0 = not)
  float _parkOdo0 = 0, _parkYaw = 0;
  void _parkReverse(const Sensors& s, float ry, uint32_t now);
  bool _restartCheck = false;         // 26.11.: this TURN is a restart check (restartLegs), not the route start
  bool _needEndPaused = false;        // 26.11.: lmkNeedEndLegs: paused once on a doubtful distance end -> next time take it
  float _lvP[8], _lvR[8];             // 26.11.: pitch / roll samples for the per-leg level
  uint8_t _lvN = 0;
  bool _lvDone = true;
  bool _lmkPath = false;              // last step steered on the taught path (HOLD phase / after the end)
  void _lmkBegin();
  bool _lmkDrive(const Sensors& s, float ry, float v);
};
