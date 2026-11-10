#pragma once
#include <stdint.h>
#include <stdio.h>
#include <math.h>
#include <string.h>
inline uint32_t& _fakeMs() { static uint32_t m = 0; return m; }
inline uint32_t millis() { return _fakeMs(); }
struct FakeSerial { template <class... A> void printf(const char* f, A... a) { ::printf(f, a...); } void println(const char* s) { puts(s); } };
static FakeSerial Serial;
