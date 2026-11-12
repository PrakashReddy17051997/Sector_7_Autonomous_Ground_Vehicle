#pragma once
// Host stub of the Arduino API for the simulator (tools/sim_v3). Firmware Serial messages go to simLog().
#include <stdint.h>
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#define DEG_TO_RAD 0.017453292519943295
#define RAD_TO_DEG 57.29577951308232
extern uint32_t g_ms;
inline uint32_t millis() { return g_ms; }
template <class T, class L, class H> inline T constrain(T x, L lo, H hi) { return x < (T)lo ? (T)lo : (x > (T)hi ? (T)hi : x); }
void simLog(const char* line);
struct SerialT {
  void printf(const char* f, ...) { char b[256]; va_list a; va_start(a, f); vsnprintf(b, sizeof(b), f, a); va_end(a); simLog(b); }
  void println(const char* s) { simLog(s); }
};
extern SerialT Serial;
#ifndef __APPLE__
#if !defined(__GLIBC__) || (__GLIBC__ * 100 + __GLIBC_MINOR__) < 238
inline size_t strlcpy(char* d, const char* s, size_t n) { size_t l = strlen(s); if (n) { size_t c = l < n - 1 ? l : n - 1; memcpy(d, s, c); d[c] = 0; } return l; }
#endif
#endif
