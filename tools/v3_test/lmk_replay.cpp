// Open-loop replay of a landmark table on a logged leg: what would LandmarkLeg command per scan?
//   ./lmk_replay log.csv markFrom markTo   (teaching log)   or   ./lmk_replay log.csv leg N  (mission log)
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
  const float ref = -84.1f;   // kRoute leg 2 course (route yaw frame = log_027 gyro frame)
  LandmarkLeg K; K.begin(kOpenSea, LMK_COUNT(kOpenSea));
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
    float yaw = g("yaw_iso_deg");
    if (std::isnan(yawOff)) yawOff = 0;   // teaching log: logged yaw = route frame
    const float ry = yaw + yawOff;
    if (std::isnan(pitch0)) pitch0 = 2.0f;   // level pitch of this box (log_027 flat parts ~1.5-2.5)
    const bool tilted = fabsf(g("pitch_deg") - pitch0) > 3.5f;
    const LandmarkLeg::Out o = K.step(scan, ry, ref, (float)s, 0.25f, tilted);
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
