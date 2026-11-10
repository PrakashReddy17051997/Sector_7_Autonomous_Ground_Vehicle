#define private public
#include "ImuManager.h"
#undef private
#include <random>
// feed |a| as an x-axis signal (0.01 m/s^2 units) at 100 Hz
static int run(const char* name, float noise, std::vector<std::pair<int,float>> spikes /* (start ms, peak) */, int widthMs, int durMs) {
  ImuManager m; std::mt19937 g(1); std::normal_distribution<float> n(0, noise);
  _fakeMs() = 0;
  for (int t = 0; t < durMs; t += 10) {
    _fakeMs() = t;
    float a = n(g);
    for (auto& s : spikes) if (t >= s.first && t < s.first + widthMs) a += s.second;
    m._tap((int16_t)(a * 100), 0, 0, t);
  }
  printf("%-46s -> double taps %u (rms %.2f)\n", name, m.doubleTaps(), m.accRms());
  return m.doubleTaps();
}
int main() {
  int bad = 0;
  bad += run("quiet desk, no tap", 0.1f, {}, 20, 5000) != 0;
  bad += run("single tap", 0.1f, {{2000, 12}}, 20, 5000) != 0;
  bad += run("double tap 250 ms apart", 0.1f, {{2000, 12}, {2250, 12}}, 20, 5000) != 1;
  bad += run("double tap, weak (6 m/s2)", 0.1f, {{2000, 6}, {2300, 6}}, 20, 5000) != 1;
  bad += run("curb bump 150 ms long, twice", 0.1f, {{2000, 12}, {2400, 12}}, 150, 5000) != 0;
  bad += run("taps 1.2 s apart (too slow)", 0.1f, {{2000, 12}, {3200, 12}}, 20, 5000) != 0;
  bad += run("cobblestone driving (rms 1.5), no tap", 1.5f, {}, 20, 20000) != 0;
  bad += run("double tap while driving on cobbles (14)", 1.5f, {{8000, 14}, {8300, 14}}, 20, 12000) != 1;
  printf(bad ? "FAILED %d\n" : "all ok\n", bad);
}
