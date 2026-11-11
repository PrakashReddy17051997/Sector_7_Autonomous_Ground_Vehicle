// Build: see tools/replay_fw/build.sh - copy the route sources + tools/replay_fw/stub/*.h into build/, add
//   int usCm(uint8_t i) const { return i < 5 ? _us[i] : 300; } int _us[5];  to stub Sensors.h, then
//   c++ -O2 -std=gnu++17 -Ibuild turn_sim.cpp build/RouteFollow.cpp build/RouteLocalizer.cpp build/LocalLocalizer.cpp build/LandmarkLeg.cpp
// Closed-loop start-turn test: real RouteFollow (box code) + a rover that LOCKS on the spot like log_030.
//   ./turn_sim <startOffDeg> <unlockAfterM> [floorEvery]   unlockAfterM: pivots work only after the wheels moved this far
#include "RouteFollow.h"
#include "WallFollow.h"
#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
uint32_t g_ms = 0; SerialT Serial; bool gStartWall = false; float gStartD = 1.5f;
static float wrap(float a){ a=fmodf(a+180.f,360.f); if(a<0)a+=360.f; return a-180.f; }
int main(int argc, char** argv) {
  const float off0 = atof(argv[1]), unlockM = atof(argv[2]);
  const int floorEvery = argc > 3 ? atoi(argv[3]) : 2;      // 1 of N ultrasonic readings sees the floor
  // log_030 start scan (row 8): the rover stood at M1, 77 deg off. Re-centre it to "on the leg" first:
  std::ifstream f("/tmp/claude-0/l30/log_030.csv"); std::string line; std::getline(f, line);
  std::vector<std::string> hdr; { std::stringstream ss(line); std::string c; while (std::getline(ss,c,',')) hdr.push_back(c); }
  int l0=0; for (size_t i=0;i<hdr.size();i++) if (hdr[i]=="lidar_000") l0=i;
  for (int k=0;k<9;k++) std::getline(f,line);
  std::vector<std::string> v; { std::stringstream ss(line); std::string c; while (std::getline(ss,c,',')) v.push_back(c); }
  uint16_t base[90]; for (int i=0;i<90;i++) base[i]=(uint16_t)atof(v[l0+i].c_str());
  const float logOff = -83.0f;   // what alignSearch finds on this scan (box said -77: rover turned RIGHT of the leg)
  Sensors s; WallFollow wf; RouteFollow rf;
  float yaw = 0;                 // gyro
  // world: heading of the rover relative to the leg = off0 + (yaw - 0)
  auto setScan = [&](){
    const float rel = off0 + yaw;                        // + = rover turned left of the leg
    const int sh = (int)lroundf((rel - logOff) / 4.0f);  // turn left by 4 deg -> bins move +1 (clockwise index)
    for (int i=0;i<90;i++) s._lidarCm[i] = base[((i - sh) % 90 + 90) % 90];
  };
  setScan(); s._yaw = s._yawNow = yaw;
  for (int k=0;k<5;k++) s._us[k]=120; s._us[3]=80;
  if (!rf.start(s, wf, 0, 0.25f, true)) { printf("start refused: %s\n", rf.msg()); return 1; }
  printf("start: %s | align %.0f deg score %.2f\n", rf.msg(), rf.alignDeg(), rf.alignScore());
  float oL=0,oR=0, moved=0, fwd=0, back=0;
  int usTick=0;
  for (g_ms = 10; g_ms < 120000; g_ms += 10) {
    if (g_ms % 250 == 0) { usTick++; s._us[4] = (usTick % floorEvery == 0) ? 16 : 300; }
    float l, r; rf.steer(s, wf, l, r);
    // MotorShaper-like ramp: turn up fast while a tactic runs (2/s), forward 0.5/s, down 1/s
    const float tv=0.5f*(l+r), tw=0.5f*(r-l); float v=0.5f*(oL+oR), w=0.5f*(oR-oL);
    const float wUp = rf.tacticActive()?2.0f:0.35f, dt=0.01f;
    auto step=[&](float cur,float tgt,float up){ float d=tgt-cur; float lim=(fabsf(tgt)>fabsf(cur)?up:1.0f)*dt; return cur+(d>lim?lim:(d<-lim?-lim:d)); };
    v=step(v,tv,0.5f); w=step(w,tw,wUp); oL=v-w; oR=v+w;
    rf.feedOutput(oL, oR);
    // rover: translation 1 m/s per unit; turn only if unlocked (moved far enough) and |w| >= 0.4 (pivot) or a swing/arc
    const float dist = fabsf(v)*dt; moved += dist; if (v>0) fwd += dist; else back += dist;
    float rate = 0;
    if (moved >= unlockM) {
      if (fabsf(v) < 0.05f && fabsf(w) >= 0.40f) rate = 40.0f * (w / 0.5f);        // pivot
      else if (fabsf(v) >= 0.05f) rate = 35.0f * w;                                  // swing / arc / rolling
    }
    yaw = wrap(yaw + rate*dt);
    s._yaw = s._yawNow = yaw;
    if (g_ms % 250 == 0) setScan();
    if (rf.state() == RouteFollow::DRIVE || rf.state() == RouteFollow::PAUSED || rf.state() == RouteFollow::DONE) {
      printf("t %.1f s: %s | state %u, heading off the leg %.1f deg, rolled fwd %.2f m back %.2f m | %s\n", g_ms/1000.0,
             rf.state()==RouteFollow::DRIVE?"DRIVING":"STOPPED", (unsigned)rf.state(), wrap(off0+yaw), fwd, back, rf.msg());
      return 0;
    }
  }
  printf("timeout: state %u off %.1f\n", (unsigned)rf.state(), wrap(off0+yaw));
}
