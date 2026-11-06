#pragma once
// ImuManager - BNO055 orientation sensor (I2C address 0x28) in IMUPLUS mode (gyro + accelerometer).
//
// All angles are given in the vehicle frame of ISO 8855 (as used by Euro NCAP), in degrees:
//   x forward, y left, z up
//   roll  (about x)  positive = right side down
//   pitch (about y)  positive = front down
//   yaw   (about z)  positive = turning left (counter-clockwise seen from above), range -180..180
//                    0 = the direction of the FIRST VALID reading (box standing on the ground),
//                    NOT magnetic north (the magnetometer was never calibrated in the field, log_004)
// The BNO055 is mounted turned by 90 deg in the box; the conversion from the sensor frame is done
// in update() (measured on kit 10, 2026-10-14: front down -> sensor roll negative, right side
// down -> sensor pitch negative).
//
// CHANGED 2026-11-04 (adapted from Kethan's field-tested version):
//   - every read is CHECKED: I2C transfer complete, system status 5 (fusion running), error 0,
//     mode IMUPLUS. The Adafruit getVector() returns 0/0/0 on a failed transfer -> the yaw jumped
//     to 0 and the heading controller turned towards a heading that did not exist.
//   - bad samples are dropped (the last good values stay); headingReady() = a good sample in the
//     last 0.5 s. Nothing may steer on the gyro without it.
//   - no blocking 3 s north alignment any more; read up to 100 Hz.
// Wire.begin() must have been called before begin().
#include <Arduino.h>

class ImuManager {
public:
  // Starts relative fusion (IMUPLUS). Returns false if the BNO055 does not answer.
  bool begin();

  // Reads the sensor; call often, it limits itself to one read per 10 ms.
  void update();

  float yawDeg() const;                         // ISO yaw, see above (0 until the first good sample)
  bool headingReady() const { return _ready && _sampleValid && _lastUpdateMs != 0 && millis() - _lastUpdateMs <= 500; }
  float pitchDeg() const { return _pitchDeg; }  // ISO pitch, + = front down
  float rollDeg() const { return _rollDeg; }    // ISO roll, + = right side down
  uint32_t badSamples() const { return _bad; }  // rejected reads since start (Dev view / log check)

  // Calibration state 0 (none) .. 3 (fully calibrated): system, gyro, accel, magnetometer
  void calibration(uint8_t& sys, uint8_t& gyro, uint8_t& accel, uint8_t& mag) const;

private:
  bool _ready = false;
  bool _sampleValid = false;
  uint32_t _lastAttemptMs = 0, _lastUpdateMs = 0, _bad = 0;
  float _fusionYawDeg = 0;                      // sensor frame: heading clockwise
  float _pitchDeg = 0, _rollDeg = 0;            // vehicle frame (ISO 8855)
  bool _zeroSet = false;
  float _zeroDeg = 0;                           // fusion heading of the first good sample
  uint8_t _cal[4] = {0, 0, 0, 0};
};
