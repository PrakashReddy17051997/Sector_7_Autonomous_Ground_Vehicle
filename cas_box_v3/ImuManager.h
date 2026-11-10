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
#include <math.h>

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

  // ---- v3 (2026-11-10): DOUBLE TAP on the box + IMU recording ---------------------------------
  // Linear acceleration (gravity removed by the BNO055 fusion, register 0x28, 1 m/s^2 = 100 LSB) is read
  // with every sample (100 Hz) into a 2.56 s ring buffer. A tap = a short spike of |a|:
  //   threshold  thr = max(tapMinMs2, tapK * rms)   rms = running RMS of |a| over ~1 s (driving on
  //              cobbles raises it, so the vibration does not count as taps)
  //   spike      |a| > thr for at most tapMaxMs (a curb / bump lasts longer -> not a tap)
  //   double     2nd spike 120..700 ms after the 1st, |a| quiet (< thr/2) for 250 ms before the 1st
  // doubleTaps() counts them; the ring buffer holds the IMU signal around the last one (tapRecord()).
  float tapMinMs2 = 4.0f, tapK = 6.0f;
  uint16_t tapMaxMs = 80, tapGapMinMs = 120, tapGapMaxMs = 700, tapQuietMs = 250;
  uint16_t doubleTaps() const { return _dTaps; }
  uint32_t lastTapMs() const { return _tapMs; }
  float accRms() const { return sqrtf(_rms2); }
  float accNow() const { return _aMag; }
  static const uint16_t kRing = 256;          // samples (10 ms) = 2.56 s
  // copies the ring (oldest first): ax/ay/az in 0.01 m/s^2, yaw in 1/16 deg; returns the sample count
  uint16_t tapRecord(int16_t* ax, int16_t* ay, int16_t* az, int16_t* yaw16) const;

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
  // v3 tap detection / ring buffer
  void _tap(int16_t ax, int16_t ay, int16_t az, uint32_t now);
  int16_t _rAx[kRing], _rAy[kRing], _rAz[kRing], _rYaw[kRing];
  uint16_t _rI = 0, _rN = 0;
  float _rms2 = 0.04f, _aMag = 0, _peak = 0;
  uint32_t _spikeStart = 0, _spikeEnd = 0, _quietSince = 0, _noisyBefore = 0, _tap1 = 0, _tapMs = 0;  // _quietSince = last noisy moment
  bool _inSpike = false;
  uint16_t _dTaps = 0;
};
