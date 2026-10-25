#pragma once
// ImuManager - BNO055 orientation sensor (I2C address 0x28) in NDOF fusion mode.
//
// All angles are given in the vehicle frame of ISO 8855 (as used by Euro NCAP), in degrees:
//   x forward, y left, z up
//   roll  (about x)  positive = right side down
//   pitch (about y)  positive = front down
//   yaw   (about z)  positive = turning left (counter-clockwise seen from above),
//                    0 = magnetic north, range -180..180
// The BNO055 is mounted turned by 90 deg in the box; the conversion from the sensor frame is done
// in update() (measured on kit 10, 2026-10-14: front down -> sensor roll negative, right side
// down -> sensor pitch negative).
//
// Yaw: the BNO055 fusion heading is smooth but, until the magnetometer is calibrated, only
// relative to the start position. begin() therefore measures the tilt-compensated magnetic
// heading for 3 s and stores the offset to the fusion heading ("north anchor"): smooth and
// roughly magnetic north from the start.
// Wire.begin() must have been called before begin().
#include <Arduino.h>

class ImuManager {
public:
  // Starts the sensor and runs the 3 s north alignment (blocking).
  // Returns false if the BNO055 does not answer.
  bool begin();

  // Reads the sensor; call often, it limits itself to one read per 100 ms.
  void update();

  float yawDeg() const;                       // ISO yaw, see above
  float pitchDeg() const { return _pitchDeg; }  // ISO pitch, + = front down
  float rollDeg() const { return _rollDeg; }    // ISO roll, + = right side down

  // Calibration state 0 (none) .. 3 (fully calibrated): system, gyro, accel, magnetometer
  void calibration(uint8_t& sys, uint8_t& gyro, uint8_t& accel, uint8_t& mag) const;

private:
  void _alignNorth(uint16_t durationMs);

  bool _ready = false;
  uint32_t _lastUpdateMs = 0;
  // sensor frame: heading clockwise from north (fusion and magnetic)
  float _fusionYawDeg = 0, _magYawDeg = 0;
  // vehicle frame (ISO 8855)
  float _pitchDeg = 0, _rollDeg = 0;
  bool _northAnchored = false;
  float _northOffsetDeg = 0;   // magnetic heading - fusion heading at start
  uint8_t _cal[4] = {0, 0, 0, 0};
};
