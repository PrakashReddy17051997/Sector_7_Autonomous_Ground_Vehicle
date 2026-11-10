#pragma once
#include "Wire.h"
enum { OPERATION_MODE_IMUPLUS = 8 };
struct Adafruit_BNO055 { Adafruit_BNO055(int, uint8_t, TwoWire*) {} bool begin(int) { return true; } void setExtCrystalUse(bool) {} };
