#pragma once
#include "Sensors.h"
extern bool gStartWall; extern float gStartD;
class WallFollow { public: enum Side : int8_t { LEFT=-1, RIGHT=1 };
  struct Wall { bool ok; float distM, angleDeg, rmsM; uint8_t points; };
  Wall measure(const Sensors&, Side, const Wall*) const { return Wall{gStartWall, gStartD, 0.0f, 0.02f, 12}; } };
