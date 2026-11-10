#pragma once
#include <stdint.h>
struct TwoWire { void beginTransmission(uint8_t) {} void write(uint8_t) {} uint8_t endTransmission(bool = true) { return 1; } uint8_t requestFrom(uint8_t, uint8_t) { return 0; } int read() { return 0; } };
static TwoWire Wire;
