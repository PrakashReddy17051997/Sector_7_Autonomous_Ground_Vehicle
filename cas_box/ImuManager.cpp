#include "ImuManager.h"
#include <Wire.h>
#include <math.h>
#include <Adafruit_BNO055.h>

static const uint8_t kBnoAddr = 0x28;
static const uint32_t kUpdatePeriodMs = 100;
static const uint16_t kNorthAlignMs = 3000;

// CHANGED 2026-10-22 (log_004): with the magnetometer uncalibrated (cal 0 for the whole run) the
// NDOF heading did not follow the rover's turns (GPS course 91 -> 146 deg, heading stayed 336-343).
// false = IMUPLUS mode: gyro + accelerometer only. Yaw then starts at 0 at power-on (= relative,
// NOT magnetic north) but follows every turn reliably. GpsNav learns true north from GPS anyway.
// true  = original behaviour (NDOF with magnetometer, yaw 0 = magnetic north).
static const bool kUseMagnetometer = false;

static Adafruit_BNO055 bno(55, kBnoAddr, &Wire);

// Wraps an angle to -180..180 degrees.
static float wrap180(float a) {
  a = fmodf(a + 180.0f, 360.0f);
  if (a < 0) a += 360.0f;
  return a - 180.0f;
}

// Tilt-compensated magnetic heading: projects the magnetometer vector into the horizontal plane
// using roll and pitch. Returns false if the horizontal field is ~0.
static bool magneticHeadingDeg(float rollDeg, float pitchDeg, float& headingDeg) {
  imu::Vector<3> m = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
  const float roll = rollDeg * DEG_TO_RAD, pitch = pitchDeg * DEG_TO_RAD;
  const float xh = m.x() * cosf(pitch) + m.z() * sinf(pitch);
  const float yh = m.x() * sinf(roll) * sinf(pitch) + m.y() * cosf(roll) - m.z() * sinf(roll) * cosf(pitch);
  if (sqrtf(xh * xh + yh * yh) < 1e-6f) return false;
  headingDeg = wrap180(atan2f(yh, xh) * RAD_TO_DEG);
  return true;
}

bool ImuManager::begin() {
  if (!bno.begin()) return false;
  bno.setExtCrystalUse(true);
  if (kUseMagnetometer) {
    bno.setMode(OPERATION_MODE_NDOF);
    _ready = true;
    _alignNorth(kNorthAlignMs);
  } else {
    bno.setMode(OPERATION_MODE_IMUPLUS);  // gyro + accel fusion, relative heading
    _ready = true;
    _northAnchored = true;                // yaw = fusion heading, 0 = direction at power-on
    _northOffsetDeg = 0;
    Serial.println("[imu] IMUPLUS mode (no magnetometer): yaw is relative to the start");
  }
  return true;
}

void ImuManager::update() {
  if (!_ready) return;
  const uint32_t now = millis();
  if (now - _lastUpdateMs < kUpdatePeriodMs) return;
  _lastUpdateMs = now;

  // Euler angles from the fusion, sensor frame: x = heading (clockwise), y = roll, z = pitch
  imu::Vector<3> e = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  const float sensorRoll = e.y(), sensorPitch = e.z();
  _fusionYawDeg = wrap180(e.x());
  float h;
  if (kUseMagnetometer && magneticHeadingDeg(sensorRoll, sensorPitch, h)) _magYawDeg = h;  // tilt comp.

  // sensor frame -> vehicle frame (ISO 8855), see header: sensor turned by 90 deg
  _pitchDeg = -sensorRoll;   // front down = sensor roll negative  -> ISO pitch positive
  _rollDeg = -sensorPitch;   // right down = sensor pitch negative -> ISO roll positive
  bno.getCalibration(&_cal[0], &_cal[1], &_cal[2], &_cal[3]);
}

float ImuManager::yawDeg() const {
  // heading is clockwise from north; ISO yaw is counter-clockwise -> sign flip.
  // Without a north anchor (alignment failed) fall back to the raw magnetic heading.
  const float heading = _northAnchored ? wrap180(_fusionYawDeg + _northOffsetDeg) : _magYawDeg;
  return wrap180(-heading);
}

void ImuManager::calibration(uint8_t& sys, uint8_t& gyro, uint8_t& accel, uint8_t& mag) const {
  sys = _cal[0]; gyro = _cal[1]; accel = _cal[2]; mag = _cal[3];
}

// Averages magnetic and fusion heading for durationMs (as unit vectors, so 179/-179 average
// correctly) and stores their difference as north offset.
void ImuManager::_alignNorth(uint16_t durationMs) {
  float sinMag = 0, cosMag = 0, sinFus = 0, cosFus = 0;
  uint16_t n = 0;
  const uint32_t t0 = millis();
  while (millis() - t0 < durationMs) {
    imu::Vector<3> e = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    float mag;
    if (magneticHeadingDeg(e.y(), e.z(), mag)) {
      sinMag += sinf(mag * DEG_TO_RAD);
      cosMag += cosf(mag * DEG_TO_RAD);
      sinFus += sinf(wrap180(e.x()) * DEG_TO_RAD);
      cosFus += cosf(wrap180(e.x()) * DEG_TO_RAD);
      n++;
    }
    delay(20);
  }
  _northAnchored = n >= 5;
  if (!_northAnchored) {
    Serial.println("[imu] north alignment failed, yaw = raw magnetic heading");
    return;
  }
  _northOffsetDeg = wrap180((atan2f(sinMag, cosMag) - atan2f(sinFus, cosFus)) * RAD_TO_DEG);
  Serial.printf("[imu] north offset %.1f deg\n", _northOffsetDeg);
}
