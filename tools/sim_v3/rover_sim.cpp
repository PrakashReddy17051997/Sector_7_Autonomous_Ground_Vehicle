// rover_sim - closed-loop simulation of the CAS box route code (v3, 12.11.2026).
//
// The SAME code as on the box runs here: RouteFollow (state machine, turns, obstacles, landmark legs),
// LocalLocalizer (start align + scan matching on the log_027 maps), LandmarkLeg, WallFollow::measure and
// MotorShaper (2 s ramps, turn ramp, +-0.5 limit). Only the hardware is simulated:
//   world     the walls of the teaching run log_027 (LocalPath.h grids, occupied 10 cm cells) + your obstacles
//   LiDAR     90 bins of 4 deg (index clockwise from ahead), closest return in cm, 10 Hz, optional mounting error
//   ultrasonic front / left / right / back cones (+-12 deg, <= 3 m, 300 = nothing), front-angled = floor 17 cm
//   IMU       yaw = true heading (+ optional drift); no slope (pitch / roll 0): ramp effects are NOT simulated
//   rover     a turn model MEASURED on rover 7 (logs 030-034), see Model below; collisions stop the rover
// Output: one self-contained HTML file (open it in a browser) with the map, the drive and every firmware message.
//
//   ./rover_sim --model rover7 --start-off 180 --out sim.html
//   ./rover_sim --model free --obst 1:6:0:0.25 --out obst.html          (leg 1, 6 m along, on the path, r 0.25 m)
//   ./rover_sim --help
#include "RouteFollow.h"
#include "WallFollow.h"
#include "MotorShaper.h"
#include "LandmarkRoute.h"
#include "LocalPath.h"
#include "sim_ramps.h"
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cmath>
#include <string>
#include <vector>
#include <unordered_set>
#include <fstream>
#include <algorithm>
#include <sstream>

uint32_t g_ms = 0;
SerialT Serial;
static std::vector<std::pair<uint32_t, std::string>> gEvents;
void simLog(const char* line) {
  std::string s(line);
  while (!s.empty() && (s.back() == '\n' || s.back() == '\r')) s.pop_back();
  if (!s.empty()) gEvents.push_back({g_ms, s});
}

static float wrapDeg(float a) { a = fmodf(a + 180.0f, 360.0f); if (a < 0) a += 360.0f; return a - 180.0f; }

// ---------------------------------------------------------------------------------------------------------------
// world
struct Obst { float x, y, r, t0, t1; };   // circle (m), present from t0 to t1 (s)
struct World {
  float res = 0.10f;
  std::unordered_set<int64_t> occ;
  std::vector<Obst> obst;
  static int64_t key(int i, int j) { return ((int64_t)(i + 1000000) << 32) | (uint32_t)(j + 1000000); }
  void add(float x, float y) { occ.insert(key((int)floorf(x / res), (int)floorf(y / res))); }
  bool wall(float x, float y) const { return occ.count(key((int)floorf(x / res), (int)floorf(y / res))) != 0; }
  bool obstAt(float x, float y, float t) const {
    for (const Obst& o : obst) if (t >= o.t0 && t <= o.t1 && (x - o.x) * (x - o.x) + (y - o.y) * (y - o.y) <= o.r * o.r) return true;
    return false;
  }
  // distance along a ray (m), 0 = nothing within maxR
  float ray(float x, float y, float a, float minR, float maxR, float t) const {
    const float c = cosf(a), s = sinf(a);
    float best = 0;
    for (float r = minR; r <= maxR; r += 0.03f) if (wall(x + c * r, y + s * r)) { best = r; break; }
    for (const Obst& o : obst) {
      if (t < o.t0 || t > o.t1) continue;
      const float dx = o.x - x, dy = o.y - y, b = dx * c + dy * s, q = dx * dx + dy * dy - o.r * o.r;
      const float disc = b * b - q;
      if (disc < 0) continue;
      const float r = b - sqrtf(disc);
      if (r >= minR && r <= maxR && (best == 0 || r < best)) best = r;
    }
    return best;
  }
};

static void legToGlobal(const LocalPathMap::Leg& L, float lx, float ly, float& gx, float& gy) {
  const float c = cosf(L.origTh), s = sinf(L.origTh);
  gx = L.origX + c * lx - s * ly;
  gy = L.origY + s * lx + c * ly;
}

// The maps hold everything the teaching run saw - also the people walking next to the rover (log_027 start: someone
// 0.3-0.5 m front-right of it). Cells closer than clearM to the taught path are dropped: the taught rover drove there,
// so nothing fixed can be there. clearM 0.30 (rover half width 0.13 + margin); the leg-1 start zone (first 1.5 m) 0.6.
// Where the teaching rover was tilted (pitch off its leg median by > 1.5 deg, sim_ramps.h) its flat LiDAR plane hit the
// ramp surface: those "walls" are cleared within gClearRampM of the path.
static float gClearM = 0.50f, gClearStartM = 0.60f, gClearRampM = 0.60f;
static float legMedianPitch(int k) {
  std::vector<float> v(kSimPitch[k].p, kSimPitch[k].p + kSimPitch[k].n);
  std::sort(v.begin(), v.end());
  return v.empty() ? 0 : v[v.size() / 2];
}
static bool tiltedAt(int k, float s) {
  const int m = (int)s;
  if (m < 0 || m >= kSimPitch[k].n) return false;
  return fabsf(kSimPitch[k].p[m] - legMedianPitch(k)) > 1.5f;
}
// which leg's taught path is closest to a point (each leg map only knows its own surroundings: a cell is taken from the
// map of the leg it belongs to, so a leg never sees cells another leg's map has that its own does not = "new things")
static int nearestLeg(float gx, float gy) {
  using namespace LocalPathMap;
  int bk = 0; float bd = 1e18f;
  for (int k = 0; k < kLegs; k++) {
    const Leg& L = kLeg[k];
    for (int i = 0; i < L.nPath; i += 2) {
      float px, py;
      legToGlobal(L, L.path[2 * i] * 0.01f, L.path[2 * i + 1] * 0.01f, px, py);
      const float d2 = (px - gx) * (px - gx) + (py - gy) * (py - gy);
      if (d2 < bd) { bd = d2; bk = k; }
    }
  }
  return bk;
}
static bool nearPath(float gx, float gy) {
  using namespace LocalPathMap;
  for (int k = 0; k < kLegs; k++) {
    const Leg& L = kLeg[k];
    for (int i = 0; i < L.nPath; i++) {
      float px, py;
      legToGlobal(L, L.path[2 * i] * 0.01f, L.path[2 * i + 1] * 0.01f, px, py);
      const float d2 = (px - gx) * (px - gx) + (py - gy) * (py - gy);
      const float lim = (k == 0 && i < 6) ? gClearStartM : (tiltedAt(k, i * 0.25f) ? gClearRampM : gClearM);
      if (d2 < lim * lim) return true;
    }
  }
  return false;
}

static void buildWorld(World& W) {
  using namespace LocalPathMap;
  for (int k = 0; k < kLegs; k++) {
    const Leg& L = kLeg[k];
    for (int i = 0; i < L.nx; i++)
      for (int j = 0; j < L.ny; j++) {
        const uint32_t idx = (uint32_t)i * L.ny + j;
        const uint8_t b = L.grid[idx >> 1];
        const uint8_t v = (idx & 1) ? (b >> 4) : (b & 15);
        if (v < 15) continue;                                      // 15 = a LiDAR wall point fell in this cell
        float gx, gy;
        legToGlobal(L, L.loX * 0.01f + (i + 0.5f) * 0.1f, L.loY * 0.01f + (j + 0.5f) * 0.1f, gx, gy);
        if (nearestLeg(gx, gy) == k && !nearPath(gx, gy)) W.add(gx, gy);
      }
  }
}

// a point on the taught path of leg k at distance sM (m) along it, and the path direction there
static bool pathPoint(int k, float sM, float& gx, float& gy, float& gth) {
  using namespace LocalPathMap;
  if (k < 0 || k >= kLegs) return false;
  const Leg& L = kLeg[k];
  float acc = 0;
  for (int i = 1; i < L.nPath; i++) {
    const float x0 = L.path[2 * i - 2] * 0.01f, y0 = L.path[2 * i - 1] * 0.01f, x1 = L.path[2 * i] * 0.01f, y1 = L.path[2 * i + 1] * 0.01f;
    const float d = hypotf(x1 - x0, y1 - y0);
    if (acc + d >= sM || i == L.nPath - 1) {
      const float f = d > 0 ? fminf(1.0f, fmaxf(0.0f, (sM - acc) / d)) : 0;
      legToGlobal(L, x0 + f * (x1 - x0), y0 + f * (y1 - y0), gx, gy);
      gth = L.origTh + atan2f(y1 - y0, x1 - x0);
      return true;
    }
    acc += d;
  }
  return false;
}

// ---------------------------------------------------------------------------------------------------------------
// rover model: yaw rate in deg/s per unit of turn command w = (r - l) / 2 (wheel outputs after the MotorShaper).
// rover7 = MEASURED on rover 7, logs 030-034 (7900 samples), sorted by the wheel directions:
//   wheels in OPPOSITE directions (pivot on the spot, also "rolling" pivots with one wheel still forward):
//       median 0 deg/s, turned > 5 deg/s in 10 % of 3540 samples          -> kOpp = 0
//   both wheels FORWARD (arc / car-like curve, |w| >= 0.12): median 16.7 deg/s per unit w (1195 samples), e.g.
//       l 0.15 / r 0.45 (w 0.15): 2.5 deg/s at ~0.3 m/s = radius ~7 m;  l 0 / r 0.5: ~5.7 deg/s, radius ~2.6 m
//                                                                         -> kFwd = 16
//   both wheels BACKWARD (reverse arc): median 31 deg/s per unit w (only 16 samples - to be confirmed) -> kBack = 30
// free = a rover that pivots on the spot (Kethan's rover / smooth floor): 40 deg/s at 0.5 whatever the wheels do.
struct Model {
  std::string name = "rover7";
  float kOpp = 0, kFwd = 16, kBack = 30;       // deg/s per unit w: wheels opposite, both forward, both backward
  float mPerS = 1.03f;                          // m/s per unit forward command (log_027 effort)
  float pull = 0;                               // natural drift (deg/s per unit v, + = left) - rover 7 pulls right
  float crabDeg = -9.0f;                        // rolling forward the rover moves this far off its nose (deg, + = left of
                                                // the nose). Teaching run log_027: nose 7-10 deg LEFT of the track = -9.
                                                // The route headings (taught nose headings) already contain it.
  float halfL = 0.17f, halfW = 0.13f;           // footprint (m) for collisions
  void set(const std::string& n) {
    name = n;
    if (n == "free") { kOpp = 80; kFwd = 80; kBack = 80; }
    else if (n == "rover7") { kOpp = 0; kFwd = 16; kBack = 30; }
    else if (n == "weak") { kOpp = 20; kFwd = 30; kBack = 40; }   // between the two: a pivot turns, slowly
    else { fprintf(stderr, "unknown model %s (rover7 | weak | free)\n", n.c_str()); exit(1); }
  }
};

struct Rover { float x = 0, y = 0, th = 0; int bumps = 0; bool touching = false; };

static bool footprintHits(const World& W, const Model& M, float x, float y, float th, float t) {
  const float c = cosf(th), s = sinf(th);
  for (int k = 0; k < 16; k++) {
    float u, v;
    if (k < 5) { u = M.halfL; v = -M.halfW + k * M.halfW / 2; }
    else if (k < 10) { u = -M.halfL; v = -M.halfW + (k - 5) * M.halfW / 2; }
    else if (k < 13) { u = -M.halfL + (k - 10 + 1) * M.halfL / 2; v = M.halfW; }
    else { u = -M.halfL + (k - 13 + 1) * M.halfL / 2; v = -M.halfW; }
    const float px = x + c * u - s * v, py = y + s * u + c * v;
    if (W.wall(px, py) || W.obstAt(px, py, t)) return true;
  }
  return false;
}

// ---------------------------------------------------------------------------------------------------------------
static void sense(const World& W, const Rover& R, Sensors& S, float t, float lidarErrDeg, float yawDrift) {
  // LiDAR bin i covers -(4i) .. -(4i+4) deg (clockwise from the nose); 3 rays per bin, the closest wins
  for (int i = 0; i < 90; i++) {
    float best = 0;
    for (float f : {0.6f, 2.0f, 3.4f}) {
      const float a = R.th + (-(i * 4.0f + f) + lidarErrDeg) * (float)DEG_TO_RAD;
      const float d = W.ray(R.x, R.y, a, 0.12f, 10.0f, t);
      if (d > 0 && (best == 0 || d < best)) best = d;
    }
    S._lidarCm[i] = best > 0 ? (uint16_t)lroundf(best * 100.0f) : 0;
  }
  const float usDir[4] = {0, 90, -90, 180};     // front, left, right, back (deg, + = left)
  for (int k = 0; k < 4; k++) {
    float best = 0;
    for (float f = -12; f <= 12; f += 4) {
      const float d = W.ray(R.x, R.y, R.th + (usDir[k] + f) * (float)DEG_TO_RAD, 0.10f, 3.0f, t);
      if (d > 0 && (best == 0 || d < best)) best = d;
    }
    S._us[k] = best > 0 ? (int)lroundf(best * 100.0f) : 300;
  }
  S._us[4] = (S._us[0] < 40) ? S._us[0] : 17;   // front-angled: sees the floor at 17 cm
  S._yaw = wrapDeg(R.th * (float)RAD_TO_DEG + yawDrift * t);
  // IMU pitch: the teaching run's pitch at the nearest taught path point (ramps), roll 0
  {
    using namespace LocalPathMap;
    float best = 1e9f, p = 0;
    for (int k = 0; k < kLegs; k++) {
      const Leg& L = kLeg[k];
      for (int i = 0; i < L.nPath; i += 2) {
        float px, py;
        legToGlobal(L, L.path[2 * i] * 0.01f, L.path[2 * i + 1] * 0.01f, px, py);
        const float d2 = (px - R.x) * (px - R.x) + (py - R.y) * (py - R.y);
        if (d2 < best) { best = d2; const int m = (int)(i * 0.25f); p = kSimPitch[k].p[m < kSimPitch[k].n ? m : kSimPitch[k].n - 1]; }
      }
    }
    S._pitch = best < 4.0f ? p : 2.5f;
  }
  S._roll = 0;
}

// ---------------------------------------------------------------------------------------------------------------
static void usage() {
  printf(
      "rover_sim - closed-loop simulation of the box route code (same code as the firmware)\n"
      "  --model rover7|weak|free   rover turn model (default rover7 = measured: no turn on the spot from standstill)\n"
      "  --start-leg N              start at leg N (1..15, default 1)\n"
      "  --start-off DEG            rover nose turned DEG from the path at the start (+ = left, e.g. 90, 180)\n"
      "  --start-xy DX DY           start position offset from the taught start (m, x along the path, y left)\n"
      "  --obst LEG:S:LAT:R[:T0:T1] obstacle: leg LEG, S m along its taught path, LAT m to the left (- = right),\n"
      "                             radius R m, present from T0 to T1 s (default always). Repeat for more.\n"
      "  --obst-xy X:Y:R[:T0:T1]    obstacle at map coordinates (m)\n"
      "  --legs N                   stop after N legs (default all)\n"
      "  --time S                   time limit (s, default 900)\n"
      "  --speed V                  mission speed (default 0.30 = the page maximum)\n"
      "  --lidar-err DEG            LiDAR mounting error vs the maps (box 7 vs box 4: about 7-9)\n"
      "  --yaw-drift DEGS           gyro drift (deg/s)\n"
      "  --pull DEGS                rover pulls this much (deg/s at full forward, - = right); trim starts at 0\n"
      "  --crab DEG                 rolling forward, the rover moves DEG off its nose (default -9 = like the teaching run:\n"
      "                             nose 9 deg left of the track; the taught headings contain it)\n"
      "  --resume S                 after a PAUSE, press Resume after S s (default: stay paused, sim ends)\n"
      "  --set NAME=VALUE           change a RouteFollow tuning value (pivotDeg, rollStart, turnCmd, kethanCmd, rollCmd,\n"
      "                             avoidPlanM, avMinScore, smartAvoid, stuckWindows, maxBackups, lmkParaExtraDeg ...)\n"
      "  --shaper NAME=VALUE        change a MotorShaper value (accelS, decelS, wUp, wDown)\n"
      "  --no-lmk                   legs 2 and 6-15 on the taught path instead of the landmark tables\n"
      "  --clear M                  map cells closer than M to the taught path are dropped (people in the teaching\n"
      "                             run, ramp surface hits; default 0.50)\n"
      "  --csv PREFIX               result files PREFIX_frames.csv, _walls.csv, _path.csv, _obst.csv, _events.txt\n"
      "                             (default sim) -> python3 plot_sim.py PREFIX  makes PREFIX.png\n"
      "  --out FILE.html            also write the interactive viewer (optional)\n");
}

static bool setRoute(RouteFollow& rf, const std::string& kv) {
  const size_t e = kv.find('=');
  if (e == std::string::npos) return false;
  const std::string n = kv.substr(0, e);
  const float v = atof(kv.substr(e + 1).c_str());
#define F(x) if (n == #x) { rf.x = v; return true; }
#define B(x) if (n == #x) { rf.x = v != 0; return true; }
#define U(x) if (n == #x) { rf.x = (decltype(rf.x))v; return true; }
  F(pivotDeg) F(turnDoneDeg) F(pivotDoneDeg) F(turnCmd) F(kethanCmd) F(rollCmd) F(rollMaxM) F(rollGoV) F(avoidPlanM)
  F(avMinScore) F(avoidEnterM) F(hardStopM) F(lmkParaExtraDeg) F(lmkRollDeg) F(lmkTiltDeg) F(climbPow) F(trimMax)
  F(avAngleDeg) F(avClearM) F(avMinClearM) F(avMaxShiftM) B(kTurn) F(kOuter) F(kInner) F(kSegM) U(kSegMs) F(fwdMoveStartM) F(fwdMoveMaxM) F(backShiftMaxM)
  B(rollStart) B(smartAvoid) B(turnTactics) B(alignStart) B(trimLearn) B(smoothReroute) B(staticFilter)
  U(stuckWindows) U(maxBackups) U(rollCarryMs)
#undef F
#undef B
#undef U
  return false;
}
static bool setShaper(MotorShaper& m, const std::string& kv) {
  const size_t e = kv.find('=');
  if (e == std::string::npos) return false;
  const std::string n = kv.substr(0, e);
  const float v = atof(kv.substr(e + 1).c_str());
  if (n == "accelS") m.accelS = v; else if (n == "decelS") m.decelS = v; else if (n == "wUp") m.wUp = v;
  else if (n == "wDown") m.wDown = v; else if (n == "firmDecelS") m.firmDecelS = v; else return false;
  return true;
}

static std::string jsonStr(const std::string& s) {
  std::string o = "\"";
  for (char c : s) { if (c == '"' || c == '\\') { o += '\\'; o += c; } else if ((unsigned char)c < 32) o += ' '; else o += c; }
  return o + "\"";
}

int main(int argc, char** argv) {
  Model M;
  int startLeg = 1, legsMax = 99;
  float startOff = 0, sdx = 0, sdy = 0, tMax = 900, speed = 0.30f, lidarErr = 0, yawDrift = 0, resumeS = -1;
  bool lmk = true;
  std::string out, csv = "sim";
  std::vector<std::string> routeSets, shaperSets, obstSpecs, obstXY;
  for (int i = 1; i < argc; i++) {
    const std::string a = argv[i];
    auto next = [&]() { return i + 1 < argc ? std::string(argv[++i]) : std::string(); };
    if (a == "--help" || a == "-h") { usage(); return 0; }
    else if (a == "--model") M.set(next());
    else if (a == "--start-leg") startLeg = atoi(next().c_str());
    else if (a == "--start-off") startOff = atof(next().c_str());
    else if (a == "--start-xy") { sdx = atof(next().c_str()); sdy = atof(next().c_str()); }
    else if (a == "--obst") obstSpecs.push_back(next());
    else if (a == "--obst-xy") obstXY.push_back(next());
    else if (a == "--legs") legsMax = atoi(next().c_str());
    else if (a == "--time") tMax = atof(next().c_str());
    else if (a == "--speed") speed = atof(next().c_str());
    else if (a == "--lidar-err") lidarErr = atof(next().c_str());
    else if (a == "--yaw-drift") yawDrift = atof(next().c_str());
    else if (a == "--pull") M.pull = atof(next().c_str());
    else if (a == "--crab") M.crabDeg = atof(next().c_str());
    else if (a == "--resume") resumeS = atof(next().c_str());
    else if (a == "--set") routeSets.push_back(next());
    else if (a == "--shaper") shaperSets.push_back(next());
    else if (a == "--no-lmk") lmk = false;
    else if (a == "--out") out = next();
    else if (a == "--csv") csv = next();
    else if (a == "--clear") gClearM = atof(next().c_str());
    else { fprintf(stderr, "unknown option %s (see --help)\n", a.c_str()); return 1; }
  }

  World W;
  buildWorld(W);
  for (const std::string& sp : obstSpecs) {
    int leg = 1; float s = 0, lat = 0, r = 0.25f, t0 = 0, t1 = 1e9f;
    if (sscanf(sp.c_str(), "%d:%f:%f:%f:%f:%f", &leg, &s, &lat, &r, &t0, &t1) < 4) { fprintf(stderr, "bad --obst %s\n", sp.c_str()); return 1; }
    float gx, gy, gth;
    if (!pathPoint(leg - 1, s, gx, gy, gth)) { fprintf(stderr, "bad leg in --obst %s\n", sp.c_str()); return 1; }
    W.obst.push_back({gx - sinf(gth) * lat, gy + cosf(gth) * lat, r, t0, t1});
  }
  for (const std::string& sp : obstXY) {
    Obst o{0, 0, 0.25f, 0, 1e9f};
    if (sscanf(sp.c_str(), "%f:%f:%f:%f:%f", &o.x, &o.y, &o.r, &o.t0, &o.t1) < 3) { fprintf(stderr, "bad --obst-xy %s\n", sp.c_str()); return 1; }
    W.obst.push_back(o);
  }

  // rover at the start of leg startLeg (the taught start), turned by startOff, shifted by sdx / sdy
  Rover R;
  {
    const LocalPathMap::Leg& L = LocalPathMap::kLeg[startLeg - 1];
    const float c = cosf(L.origTh), s = sinf(L.origTh);
    R.x = L.origX + c * sdx - s * sdy;
    R.y = L.origY + s * sdx + c * sdy;
    R.th = L.origTh + startOff * (float)DEG_TO_RAD;
  }

  // the box: the same set-up as cas_box_v3.ino setup()
  Sensors S;
  WallFollow wf;
  RouteFollow rf;
  MotorShaper shaper;
  rf.alignStart = true;
  rf.trim = 0;                         // the sim rover does not pull unless --pull (rover 7 on the box: +0.25)
  rf.smartAvoid = true;
  rf.kethanCmd = 0.50f;
  if (lmk) {
    rf.setLandmarkLeg(2 - 1, kOpenSea, LMK_COUNT(kOpenSea));
    for (const LmkLegReg& g : kLmkLegs) rf.setLandmarkLeg(g.leg - 1, g.ph, g.n, g.lenM);
  }
  for (const std::string& kv : routeSets) if (!setRoute(rf, kv)) { fprintf(stderr, "unknown --set %s\n", kv.c_str()); return 1; }
  for (const std::string& kv : shaperSets) if (!setShaper(shaper, kv)) { fprintf(stderr, "unknown --shaper %s\n", kv.c_str()); return 1; }
  shaper.wUp = 0.50f;
  for (const std::string& kv : shaperSets) setShaper(shaper, kv);

  g_ms = 1000;
  sense(W, R, S, 0, lidarErr, yawDrift);
  const bool ok = rf.start(S, wf, startLeg - 1, speed, false);
  simLog(ok ? "[sim] mission started" : "[sim] start refused");

  // frames: t, x, y, heading, state, leg, cmdL, cmdR, outL, outR, tactic/msg index, bump; scans every frame
  struct Frame { float t, x, y, th; int st, leg; float cl, cr, ol, orr; int msg; int bump; float err; };
  std::vector<Frame> frames;
  std::vector<std::vector<uint16_t>> scans;
  std::vector<std::string> msgs;
  auto msgIdx = [&](const std::string& m) { for (size_t i = 0; i < msgs.size(); i++) if (msgs[i] == m) return (int)i; msgs.push_back(m); return (int)msgs.size() - 1; };

  const float dt = 0.02f;
  uint32_t pausedMs = 0;
  const int legAtStart = rf.legIndex();
  int bumpEvents = 0;
  std::string endWhy = "time limit";
  for (int k = 0; ok; k++) {
    const float t = k * dt;
    g_ms = 1000 + (uint32_t)lroundf(t * 1000.0f);
    if (t > tMax) break;
    if (k % 5 == 0) sense(W, R, S, t, lidarErr, yawDrift);          // 10 Hz sensors
    else S._yaw = wrapDeg(R.th * (float)RAD_TO_DEG + yawDrift * t);  // the gyro every pass
    float l = 0, r = 0;
    rf.steer(S, wf, l, r);
    const float cl = l, cr = r;
    // SafetyGuard (simplified): LiDAR closer than 30 cm in the front +-30 deg, or front ultrasonic < 25 cm -> no forward
    // SafetyGuard (as cas_box_v3.ino + SafetyGuard.h, simplified): forward blocked when the LiDAR front sector (+-20 deg)
    // has a return closer than 30 cm, or the front ultrasonic < 25 cm; on a ramp (RouteFollow::onRamp) 0 / 10 cm
    bool hard = false;
    if (l + r > 0) {
      const bool ramp = rf.onRamp();
      const int lidStop = ramp ? 0 : 30, usStop = ramp ? 10 : 25;
      uint16_t fmin = 9999;
      for (int i = -5; i < 5; i++) { const uint16_t cm = S._lidarCm[(i + 90) % 90]; if (cm > 0 && cm < fmin) fmin = cm; }
      if (fmin < lidStop || (S._us[0] > 2 && S._us[0] < usStop)) { l = r = 0; hard = true; }
    }
    shaper.firm = rf.firmStop();
    shaper.boost = rf.boost();
    shaper.wheelRamp = rf.wheelRamp();
    shaper.apply(l, r, hard);
    rf.feedOutput(l, r);

    // rover physics
    const float v = 0.5f * (l + r), w = 0.5f * (r - l);
    const float mv = fabsf(v) < 0.03f ? 0.0f : M.mPerS * v;
    const bool opposite = l * r < -0.0004f;                     // one wheel forward, the other backward
    float rate = opposite ? M.kOpp * w : (v > 0.02f ? M.kFwd * w : (v < -0.02f ? M.kBack * w : M.kOpp * w));
    rate += M.pull * v;
    const float nth = R.th + rate * (float)DEG_TO_RAD * dt;
    const float mvDir = nth + (mv > 0 ? M.crabDeg * (float)DEG_TO_RAD : 0.0f);
    const float nx = R.x + cosf(mvDir) * mv * dt, ny = R.y + sinf(mvDir) * mv * dt;
    if (!footprintHits(W, M, nx, ny, nth, t)) { R.x = nx; R.y = ny; R.th = nth; R.touching = false; }
    else {
      if (!R.touching) { bumpEvents++; char b[96]; snprintf(b, sizeof(b), "[sim] BUMP: the rover touches a wall / obstacle (%d)", bumpEvents); simLog(b); }
      R.touching = true;
      if (!footprintHits(W, M, R.x, R.y, nth, t)) R.th = nth;    // can still turn if the turn is free
    }

    if (k % 10 == 0) {   // 5 Hz frames
      std::string m = rf.tacticActive() ? std::string(rf.tacticMsg()) : std::string(rf.msg());
      if (rf.onLmkLeg() && rf.state() == RouteFollow::DRIVE) m += std::string(" | lmk: ") + rf.lmkWhy();
      frames.push_back({t, R.x, R.y, R.th * (float)RAD_TO_DEG, (int)rf.state(), (int)rf.legIndex() + 1, cl, cr, l, r, msgIdx(m), R.touching ? 1 : 0, rf.ctrlErrDeg()});
      scans.emplace_back(S._lidarCm, S._lidarCm + 90);
    }
    const uint8_t st = rf.state();
    if (st == RouteFollow::DONE) { endWhy = "route DONE"; break; }
    if ((int)rf.legIndex() - legAtStart >= legsMax) { endWhy = "leg limit"; break; }
    if (st == RouteFollow::PAUSED) {
      if (!pausedMs) pausedMs = g_ms;
      if (resumeS >= 0 && g_ms - pausedMs >= resumeS * 1000) { simLog("[sim] Resume pressed"); rf.resume(speed); pausedMs = 0; }
      else if (resumeS < 0 && g_ms - pausedMs > 3000) { endWhy = std::string("PAUSED: ") + rf.msg(); break; }
    } else pausedMs = 0;
  }
  const float tEnd = frames.empty() ? 0 : frames.back().t;
  char sum[256];
  snprintf(sum, sizeof(sum), "end after %.0f s: %s | leg %d of 15 | bumps %d | model %s", tEnd, endWhy.c_str(),
           (int)rf.legIndex() + 1, bumpEvents, M.name.c_str());
  printf("%s\n", sum);

  // ---- CSV results (plot with plot_sim.py)
  {
    FILE* f = fopen((csv + "_frames.csv").c_str(), "w");
    fprintf(f, "t_s,x_m,y_m,heading_deg,state,leg,ask_l,ask_r,wheel_l,wheel_r,head_err_deg,touch,msg\n");
    for (const Frame& q : frames)
      fprintf(f, "%.1f,%.3f,%.3f,%.1f,%d,%d,%.2f,%.2f,%.2f,%.2f,%.1f,%d,\"%s\"\n", q.t, q.x, q.y, q.th, q.st, q.leg, q.cl, q.cr, q.ol, q.orr, q.err, q.bump, msgs[q.msg].c_str());
    fclose(f);
    f = fopen((csv + "_walls.csv").c_str(), "w");
    fprintf(f, "x_m,y_m\n");
    for (int64_t k : W.occ) { const int i = (int)(k >> 32) - 1000000, j = (int)(uint32_t)k - 1000000; fprintf(f, "%.2f,%.2f\n", (i + 0.5f) * W.res, (j + 0.5f) * W.res); }
    fclose(f);
    f = fopen((csv + "_path.csv").c_str(), "w");
    fprintf(f, "leg,x_m,y_m\n");
    for (int k = 0; k < LocalPathMap::kLegs; k++) {
      const LocalPathMap::Leg& L = LocalPathMap::kLeg[k];
      for (int i = 0; i < L.nPath; i++) { float gx, gy; legToGlobal(L, L.path[2 * i] * 0.01f, L.path[2 * i + 1] * 0.01f, gx, gy); fprintf(f, "%d,%.2f,%.2f\n", k + 1, gx, gy); }
    }
    fclose(f);
    f = fopen((csv + "_obst.csv").c_str(), "w");
    fprintf(f, "x_m,y_m,r_m,t_from_s,t_to_s\n");
    for (const Obst& o : W.obst) fprintf(f, "%.2f,%.2f,%.2f,%.1f,%.1f\n", o.x, o.y, o.r, o.t0, o.t1 > 1e8f ? -1.0f : o.t1);
    fclose(f);
    f = fopen((csv + "_events.txt").c_str(), "w");
    fprintf(f, "# %s\n", sum);
    for (auto& e : gEvents) fprintf(f, "%7.1f  %s\n", (e.first - 1000) / 1000.0, e.second.c_str());
    fclose(f);
    printf("results: %s_frames.csv (+ _walls, _path, _obst, _events)\n", csv.c_str());
  }
  if (out.empty()) return 0;
  // ---- viewer (optional)
  std::ostringstream js;
  js << "{\"summary\":" << jsonStr(sum) << ",\"args\":";
  { std::string a; for (int i = 1; i < argc; i++) { a += argv[i]; a += ' '; } js << jsonStr(a); }
  js << ",\"walls\":[";
  { bool f = true; for (int64_t k : W.occ) { const int i = (int)(k >> 32) - 1000000, j = (int)(uint32_t)k - 1000000; js << (f ? "" : ",") << i << "," << j; f = false; } }
  js << "],\"path\":[";
  for (int k = 0; k < LocalPathMap::kLegs; k++) {
    const LocalPathMap::Leg& L = LocalPathMap::kLeg[k];
    js << (k ? "," : "") << "[";
    for (int i = 0; i < L.nPath; i++) { float gx, gy; legToGlobal(L, L.path[2 * i] * 0.01f, L.path[2 * i + 1] * 0.01f, gx, gy); js << (i ? "," : "") << lroundf(gx * 100) << "," << lroundf(gy * 100); }
    js << "]";
  }
  js << "],\"obst\":[";
  for (size_t i = 0; i < W.obst.size(); i++) { const Obst& o = W.obst[i]; js << (i ? "," : "") << "[" << o.x << "," << o.y << "," << o.r << "," << o.t0 << "," << (o.t1 > 1e8f ? -1 : o.t1) << "]"; }
  js << "],\"msgs\":[";
  for (size_t i = 0; i < msgs.size(); i++) js << (i ? "," : "") << jsonStr(msgs[i]);
  js << "],\"frames\":[";
  char b[200];
  for (size_t i = 0; i < frames.size(); i++) {
    const Frame& f = frames[i];
    snprintf(b, sizeof(b), "%s[%.1f,%.3f,%.3f,%.1f,%d,%d,%.2f,%.2f,%.2f,%.2f,%d,%d,%.1f]", i ? "," : "", f.t, f.x, f.y, f.th, f.st, f.leg, f.cl, f.cr, f.ol, f.orr, f.msg, f.bump, f.err);
    js << b;
  }
  js << "],\"scans\":[";
  for (size_t i = 0; i < scans.size(); i++) { js << (i ? "," : "") << "["; for (int k = 0; k < 90; k++) js << (k ? "," : "") << scans[i][k]; js << "]"; }
  js << "],\"events\":[";
  for (size_t i = 0; i < gEvents.size(); i++) js << (i ? "," : "") << "[" << (gEvents[i].first - 1000) / 1000.0 << "," << jsonStr(gEvents[i].second) << "]";
  js << "]}";

  std::string tpl;
  {
    std::string exe = argv[0];
    const size_t sl = exe.find_last_of('/');
    const std::string dir = sl == std::string::npos ? "." : exe.substr(0, sl);
    std::ifstream f(dir + "/sim_view.html");
    if (!f) f.open("sim_view.html");
    if (!f) { fprintf(stderr, "sim_view.html not found next to rover_sim\n"); return 1; }
    std::stringstream ss; ss << f.rdbuf(); tpl = ss.str();
  }
  const size_t p = tpl.find("/*SIMDATA*/null");
  if (p == std::string::npos) { fprintf(stderr, "sim_view.html: marker missing\n"); return 1; }
  tpl.replace(p, strlen("/*SIMDATA*/null"), js.str());
  std::ofstream o(out);
  o << tpl;
  printf("viewer: %s\n", out.c_str());
  return 0;
}
