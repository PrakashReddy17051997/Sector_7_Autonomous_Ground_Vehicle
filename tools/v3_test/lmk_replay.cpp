// Open-loop replay of a landmark table on a logged leg: what would LandmarkLeg command per scan?
//   ./lmk_replay log.csv markFrom markTo   (teaching log)   or   ./lmk_replay log.csv leg N  (mission log)
//   optional last argument T (1-based route leg): use the kLmkLegs table of leg T (default: the Open Sea table, leg 2)
#include "LandmarkLeg.h"
#include "LandmarkRoute.h"
#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <string>
#include <vector>
#include <sstream>
#include <fstream>
#include <map>
int main(int argc, char** argv) {
  std::ifstream f(argv[1]); std::string line; std::getline(f, line);
  std::map<std::string, int> H; { std::stringstream ss(line); std::string c; int i = 0; while (std::getline(ss, c, ',')) H[c] = i++; }
  const bool byLeg = std::string(argv[2]) == "leg";
  const int a = atoi(argv[byLeg ? 3 : 2]), b = byLeg ? 0 : atoi(argv[3]);
  static const float REF[] = {1.6f, -84.1f, -182.3f, -94.1f, -93.2f, -82.4f, -93.6f, -99.2f, -112.9f, -183.0f, -184.7f, -183.9f, -183.2f, -183.5f, -273.5f};
  const int tab = argc > 4 ? atoi(argv[4]) : 2;
  const float ref = REF[tab - 1];   // kRoute course (route yaw frame = log_027 gyro frame)
  LandmarkLeg K; K.begin(kOpenSea, LMK_COUNT(kOpenSea));
  K.sideParaExtraDeg = getenv("PARA") ? atof(getenv("PARA")) : 10.0f;   // as RouteFollow::lmkParaExtraDeg
  for (const auto& r : kLmkLegs) if (r.leg == tab) K.begin(r.ph, r.n);
  float roll0 = NAN;
  double s = 0, tp = -1, lastT = -1; float pitch0 = NAN, yawOff = NAN;
  int n = 0, nLine = 0; float maxCmd = 0; std::string lastWhy;
  bool first = true;
  while (std::getline(f, line)) {
    std::vector<std::string> v; { std::stringstream ss(line); std::string c; while (std::getline(ss, c, ',')) v.push_back(c); }
    auto g = [&](const char* k) { return atof(v[H[k]].c_str()); };
    if (first) { first = false; if (byLeg) yawOff = 1.6f - (float)g("yaw_iso_deg"); }   // mission: route start = leg 1 ref 1.6
    const int mark = (int)g("mark"), leg = (int)g("route_leg");
    const bool in = byLeg ? leg == a : (mark >= a && mark < b);
    const double t = g("t_s");
    if (!in) { tp = t; continue; }
    const double vv = 0.5 * (g("out_l") + g("out_r"));
    if (tp >= 0 && vv > 0) s += 1.03 * vv * (t - tp);
    tp = t;
    if (lastT >= 0 && t - lastT < 0.24) continue;   // 4 Hz like the firmware step
    lastT = t;
    uint16_t scan[90]; for (int i = 0; i < 90; i++) scan[i] = (uint16_t)g(("lidar_" + std::string(i * 4 < 10 ? "00" : (i * 4 < 100 ? "0" : "")) + std::to_string(i * 4)).c_str());
    if (getenv("SHIFT_BINS")) {   // test: scan rotated by N bins (4 deg each, + = turned left) - a LiDAR offset error
      const int sh = atoi(getenv("SHIFT_BINS")); uint16_t t2[90];
      for (int i = 0; i < 90; i++) t2[i] = scan[((i - sh) % 90 + 90) % 90];
      for (int i = 0; i < 90; i++) scan[i] = t2[i];
    }
    float yaw = g("yaw_iso_deg");
    static bool legAligned = false;    // mission log with a table leg: the leg starts on its course (route frame)
    if (byLeg && argc > 4 && !legAligned) { legAligned = true; yawOff = ref - yaw; }
    if (std::isnan(yawOff)) yawOff = 0;   // teaching log: logged yaw = route frame
    const float ry = yaw + yawOff;
    if (std::isnan(pitch0)) pitch0 = 2.0f;   // level pitch of this box (log_027 flat parts ~1.5-2.5)
    if (std::isnan(roll0)) roll0 = (float)g("roll_deg");
    const bool tilted = fabsf(g("pitch_deg") - pitch0) > 3.5f;
    const bool rolled = fabsf(g("roll_deg") - roll0) > 5.0f;   // as RouteFollow::lmkRollDeg (12.11.)
    const bool old = getenv("OLD_GATE") != nullptr;            // the 11.11. gate: pitch holds every phase
    const LandmarkLeg::Out o = K.step(scan, ry, ref, (float)s, 0.25f, tilted, old ? (rolled || tilted) : rolled);
    const float cmd = LineFeat::wrap180(o.yawDes - ry);
    n++; if (o.line) nLine++;
    if (fabsf(cmd) > maxCmd && o.line) maxCmd = fabsf(cmd);
    if (std::string(o.why) != lastWhy || n % 12 == 0 || o.done)
      printf("s %5.1f m  yaw %6.1f  phase %u %-9s d %5.2f e %+5.2f turn %+5.1f  -> steer %+6.1f deg  dOff %+5.2f  %s\n",
             s, yaw, K.phase() + 1, K.typeName(), K.lastD(), K.lastE(), K.turnDeg(), cmd, o.dOff, o.why);
    lastWhy = o.why;
    if (o.done) break;
  }
  printf("== %d scans, landmark line used on %d (%.0f %%), leg end at s %.1f m\n", n, nLine, 100.0 * nLine / (n ? n : 1), s);
}
