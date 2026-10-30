// replay_fw - feeds a REAL drive log (analyze_log.py CSV) through the REAL firmware route code on
// the laptop (RouteFollow + LocalLocalizer + LocalPath.h, compiled from cas_box/), and writes what the
// code believed and decided at every log record.
//
//   ./replay_fw log_017.csv > log_017_replay.csv
//
// Inputs per record (4 Hz): LiDAR scan, IMU yaw + pitch, the wheel output the box really sent
// (out_l / out_r -> effort, i.e. distance estimate). Between two records the controller runs at
// 50 Hz on the yaw interpolated between them. A route (re)start in the log (state goes from
// idle/pause/done to TURN/DRIVE) restarts the replay at that leg - like pressing the button.
//
// Build:  tools/replay_fw/build.sh   (g++ or clang++, no Arduino needed)
// The firmware [route] messages go to stderr.
// Limits: open loop - the rover in the log really moved as it did; the replay shows what the
// CURRENT code would believe and command at each moment, it cannot show where the rover would
// have gone with those commands (that is tools/sim2). WallFollow is a stub (path mode only).
//
// Output columns: t, log leg/state, replay leg/state, located (LiDAR match ok), x y th(deg) in the
// leg frame, s / sEnd along the leg path, off path, score, carrot heading error, controller error,
// why (1 roll, 2 tap, 3 settle, 4 stand, 5 step), replay command l/r, logged command l/r, msg.
#include "RouteFollow.h"
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>
#include <cmath>
uint32_t g_ms = 0; SerialT Serial; bool gStartWall = true; float gStartD = 1.5f;

static std::vector<std::string> split(const std::string& l) {
  std::vector<std::string> v; std::string c;
  for (char ch : l) { if (ch == ',') { v.push_back(c); c.clear(); } else if (ch != '\r' && ch != '\n') c += ch; }
  v.push_back(c); return v;
}

int main(int argc, char** argv) {
  if (argc < 2) { fprintf(stderr, "usage: replay_fw log.csv [quiet]\n"); return 1; }
  FILE* f = fopen(argv[1], "r"); if (!f) { perror(argv[1]); return 1; }
  static char buf[1 << 16];
  std::vector<std::vector<std::string>> rows; std::vector<std::string> hdr;
  while (fgets(buf, sizeof buf, f)) { auto v = split(buf); if (hdr.empty()) hdr = v; else rows.push_back(v); }
  fclose(f);
  auto col = [&](const char* n) { for (size_t i = 0; i < hdr.size(); i++) if (hdr[i] == n) return (int)i; return -1; };
  const int cT = col("t_s"), cY = col("yaw_iso_deg"), cP = col("pitch_deg"), cOl = col("out_l"), cOr = col("out_r"),
            cCl = col("cmd_l"), cCr = col("cmd_r"), cLeg = col("route_leg"), cSt = col("route_state"), cL0 = col("lidar_000");
  if (cT < 0 || cY < 0 || cL0 < 0) { fprintf(stderr, "missing columns\n"); return 1; }
  auto num = [](const std::string& s) { return s.empty() ? 0.0 : atof(s.c_str()); };

  Sensors s; WallFollow wf; RouteFollow rf;
  bool running = false; int prevSt = 0;
  const double t0 = num(rows[0][cT]);
  printf("t,log_leg,log_st,leg,st,located,x,y,th,s,s_end,off,score,carrot_err,ctrl_err,why,cmd_l,cmd_r,log_cmd_l,log_cmd_r,msg\n");
  for (size_t k = 0; k + 1 < rows.size(); k++) {
    const auto& R = rows[k]; const auto& N = rows[k + 1];
    const double t = num(R[cT]) - t0, tn = num(N[cT]) - t0;
    for (int i = 0; i < 90; i++) s._lidarCm[i] = (uint16_t)num(R[cL0 + i]);
    const float y0 = (float)num(R[cY]), y1 = (float)num(N[cY]);
    float dy = fmodf(y1 - y0 + 540.0f, 360.0f) - 180.0f;
    s._yaw = y0; s._pitch = cP >= 0 ? (float)num(R[cP]) : 0;
    const int lst = cSt >= 0 ? (int)num(R[cSt]) : 0, lleg = cLeg >= 0 ? (int)num(R[cLeg]) : 0;
    const bool act = lst >= 1 && lst <= 4;
    if (act && !(prevSt >= 1 && prevSt <= 4) && lleg >= 1) {     // (re)start in the log
      g_ms = (uint32_t)(t * 1000);
      s._yawNow = y0;
      running = rf.start(s, wf, (uint8_t)(lleg - 1), 0.30f, false);
      if (!running) fprintf(stderr, "t %.1f start leg %d failed: %s\n", t, lleg, rf.msg());
    }
    if (!act) running = false;
    prevSt = lst;
    float l = 0, r = 0;
    const float ol = cOl >= 0 ? (float)num(R[cOl]) : 0, orr = cOr >= 0 ? (float)num(R[cOr]) : 0;
    // 50 Hz between this record and the next: yaw interpolated, the REAL output fed as effort
    const uint32_t ms0 = (uint32_t)(t * 1000), ms1 = (uint32_t)(tn * 1000);
    bool first = true; float l0 = 0, r0 = 0;
    for (uint32_t ms = ms0; ms < ms1; ms += 20) {
      g_ms = ms;
      s._yawNow = y0 + dy * (float)(ms - ms0) / (float)(ms1 - ms0 > 0 ? ms1 - ms0 : 1);
      if (running && rf.active()) { rf.steer(s, wf, l, r); if (first) { l0 = l; r0 = r; first = false; } }
      rf.feedOutput(ol, orr);   // what the rover really got
    }
    if (!running) continue;
    const LocalLocalizer& P = rf.local();
    printf("%.2f,%d,%d,%u,%u,%d,%.2f,%.2f,%.1f,%.2f,%.2f,%.2f,%.2f,%.1f,%.1f,%u,%.2f,%.2f,%.2f,%.2f,\"%s\"\n",
           t, lleg, lst, (unsigned)rf.logLeg(), (unsigned)rf.state(), (int)rf.pathLocked(), P.x(), P.y(), P.th() * 57.29578f,
           P.s(), P.sEnd(), P.off(), P.score(), rf.pathOn() ? P.headingErrorToWaypoint(rf.pathLookM) * 57.29578f : 0.0f,
           rf.ctrlErrDeg(), (unsigned)rf.why(), l0, r0, cCl >= 0 ? num(R[cCl]) : 0, cCr >= 0 ? num(R[cCr]) : 0, rf.msg());
  }
  return 0;
}
