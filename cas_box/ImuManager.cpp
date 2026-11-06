#include "ImuManager.h"
#include <Wire.h>
#include <math.h>
#include <Adafruit_BNO055.h>

static const uint8_t kBnoAddr = 0x28;
static const uint32_t kUpdatePeriodMs = 10;   // up to 100 Hz (RouteFollow steers on the live yaw)

static Adafruit_BNO055 bno(55, kBnoAddr, &Wire);

// Reads `length` registers and CHECKS the transfer (Adafruit's getVector() returns zeros on failure).
static bool readBno(uint8_t reg, uint8_t* data, uint8_t length) {
  Wire.beginTransmission(kBnoAddr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(kBnoAddr, length) != length) return false;
  for (uint8_t i = 0; i < length; ++i) data[i] = Wire.read();
  return true;
}

// Wraps an angle to -180..180 degrees.
static float wrap180(float a) {
  a = fmodf(a + 180.0f, 360.0f);
  if (a < 0) a += 360.0f;
  return a - 180.0f;
}

bool ImuManager::begin() {
  _ready = false; _sampleValid = false; _zeroSet = false;
  _lastAttemptMs = _lastUpdateMs = 0;
  for (uint8_t i = 0; i < 4; ++i) _cal[i] = 0;
  if (!bno.begin(OPERATION_MODE_IMUPLUS)) return false;
  bno.setExtCrystalUse(true);
  _ready = true;
  Serial.println("[imu] IMUPLUS: yaw 0 = first valid reading, checked reads up to 100 Hz");
  return true;
}

void ImuManager::update() {
  if (!_ready) return;
  const uint32_t now = millis();
  if (now - _lastAttemptMs < kUpdatePeriodMs) return;
  _lastAttemptMs = now;

  uint8_t health[9] = {};  // 0x35..0x3D: calibration, self-test, interrupt, clock, status, error, unit, (res), mode
  uint8_t angles[6] = {};  // 0x1A..0x1F: Euler heading, roll, pitch (1/16 deg)
  const bool received = readBno(0x35, health, sizeof(health)) && readBno(0x1A, angles, sizeof(angles));
  _sampleValid = received && health[4] == 5 && health[5] == 0 && health[8] == OPERATION_MODE_IMUPLUS;

  static uint32_t lastReportMs = 0;
  if (now - lastReportMs >= 5000) {
    lastReportMs = now;
    Serial.printf("[imu] read=%s mode=%u status=%u error=%u cal=%u/%u/%u/%u bad=%lu\n",
                  received ? "OK" : "FAIL", health[8], health[4], health[5],
                  (health[0] >> 6) & 3, (health[0] >> 4) & 3, (health[0] >> 2) & 3, health[0] & 3,
                  (unsigned long)_bad);
  }
  if (!_sampleValid) { _bad++; return; }   // keep the last good values

  // Euler angles from the fusion, sensor frame: x = heading (clockwise), y = roll, z = pitch
  const float sensorRoll = (int16_t)(angles[2] | (angles[3] << 8)) / 16.0f;
  const float sensorPitch = (int16_t)(angles[4] | (angles[5] << 8)) / 16.0f;
  _fusionYawDeg = wrap180((int16_t)(angles[0] | (angles[1] << 8)) / 16.0f);
  if (!_zeroSet) { _zeroDeg = _fusionYawDeg; _zeroSet = true; }

  // sensor frame -> vehicle frame (ISO 8855), see header: sensor turned by 90 deg
  _pitchDeg = -sensorRoll;   // front down = sensor roll negative  -> ISO pitch positive
  _rollDeg = -sensorPitch;   // right down = sensor pitch negative -> ISO roll positive
  for (uint8_t i = 0; i < 4; ++i) _cal[i] = (health[0] >> (6 - 2 * i)) & 3;
  _lastUpdateMs = now;
}

float ImuManager::yawDeg() const {
  // sensor heading is clockwise, ISO yaw counter-clockwise -> sign flip; 0 = first good sample
  return _zeroSet ? wrap180(-wrap180(_fusionYawDeg - _zeroDeg)) : 0;
}

void ImuManager::calibration(uint8_t& sys, uint8_t& gyro, uint8_t& accel, uint8_t& mag) const {
  sys = _cal[0]; gyro = _cal[1]; accel = _cal[2]; mag = _cal[3];
}
