// Start-align test: real start scans, rotated to every 4-deg heading, matched against the log_027 leg-1 map.
//   c++ -O2 -std=gnu++17 -I../../cas_box_v3 align_test.cpp ../../cas_box_v3/LocalLocalizer.cpp -o align_test
//   ./align_test ../../cas_box_log/log_028.csv [row]
#include "LocalLocalizer.h"
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cmath>
#include <string>
#include <vector>
#include <sstream>
#include <fstream>
#include <chrono>
int main(int argc, char** argv) {
  std::ifstream f(argv[1]); std::string line; std::getline(f, line);
  std::vector<std::string> hdr; { std::stringstream ss(line); std::string c; while (std::getline(ss, c, ',')) hdr.push_back(c); }
  int l0 = -1; for (size_t i = 0; i < hdr.size(); i++) if (hdr[i] == "lidar_000") l0 = (int)i;
  const int want = argc > 2 ? atoi(argv[2]) : 4;
  for (int r = 0; r <= want && std::getline(f, line); r++) {}
  std::vector<std::string> v; { std::stringstream ss(line); std::string c; while (std::getline(ss, c, ',')) v.push_back(c); }
  uint16_t scan[90]; int n = 0; for (int i = 0; i < 90; i++) { scan[i] = (uint16_t)atof(v[l0 + i].c_str()); if (scan[i]) n++; }
  LocalLocalizer L;
  int ok = 0, okRight = 0, worst = 0, amb = 0;
  float th0 = 0; double tmax = 0;
  for (int k = 0; k < 90; k++) {
    uint16_t rot[90]; for (int j = 0; j < 90; j++) rot[j] = scan[((j - k) % 90 + 90) % 90];
    float x, y, th, b, s;
    auto t0 = std::chrono::steady_clock::now();
    const bool good = L.alignSearch(rot, 0, 0.6f, x, y, th, b, s);
    tmax = std::max(tmax, std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count());
    const float deg = th * 57.2958f;
    if (k == 0) th0 = deg;
    float err = fmodf(deg - th0 - 4.0f * k + 540.0f, 360.0f) - 180.0f;
    if (good) { ok++; if (fabsf(err) <= 5) okRight++; else { amb++; } }
    if (fabsf(err) > worst && good) worst = (int)fabsf(err);
    if (k % 15 == 0 || (good && fabsf(err) > 5))
      printf("rot %+4d deg -> %s th %+6.1f (err %+5.1f) pos %.2f/%.2f score %.2f 2nd %.2f\n", (k <= 45 ? 4 * k : 4 * k - 360),
             good ? "OK  " : "UNSURE", deg, err, x, y, b, s);
  }
  printf("%s row %d (%d points): accepted %d/90, correct %d, WRONG %d, worst err of accepted %d deg, host time max %.1f ms\n",
         argv[1], want, n, ok, okRight, amb, worst, tmax);
}
