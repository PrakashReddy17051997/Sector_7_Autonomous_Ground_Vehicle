#include "Sensors.h"
#include <Wire.h>

static const uint8_t kImuAddr = 0x28;
static const uint32_t kSnapshotMs = 250;
static const uint32_t kImuRetryMs = 5000;

bool i2cPresent(uint8_t addr) {
  Wire.beginTransmission(addr);
  return Wire.endTransmission() == 0;
}

void Sensors::begin(int lidarOffsetDeg) {
  _lidarOffsetDeg = lidarOffsetDeg;
  Wire.begin(21, 22);                 // I2C: IMU + OLED

  _us.begin(kUsMaxCm, 17500);         // 17.5 ms echo timeout ~ 3 m

  LidarScan::Config lc;               // GPIO27/33, 230400 baud, filtering see LidarScan.h
  _lidar.begin(Serial1, lc);

  _gps.begin();                       // Serial2, GPIO16/17, 9600 baud
}

void Sensors::update() {
  _lidar.update();  // drain both UARTs every pass
  _gps.poll();
  if (_imuReady) _imu.update(); else _startImuIfPresent();

  const uint32_t now = millis();
  if (now - _lastSnapMs < kSnapshotMs) return;
  _lastSnapMs = now;

  _usCm[0] = _us.readFrontCm();
  _usCm[1] = _us.readLeftCm();
  _usCm[2] = _us.readRightCm();
  _usCm[3] = _us.readBackCm();
  _usCm[4] = _us.readFrontAngledCm();
  _snapshotLidar();
  _snapshotGps();
  if (_imuReady) {
    _yaw = _imu.yawDeg();
    _pitch = _imu.pitchDeg();
    _roll = _imu.rollDeg();
    _imu.calibration(_cal[0], _cal[1], _cal[2], _cal[3]);
  }
}

// The IMU start blocks ~3.5 s once (north alignment); RoverLink's deadman stops the rover meanwhile.
void Sensors::_startImuIfPresent() {
  const uint32_t now = millis();
  if (now - _lastImuTryMs < kImuRetryMs) return;
  _lastImuTryMs = now;
  if (!i2cPresent(kImuAddr)) return;
  Serial.println("[sensors] BNO055 found, starting IMU (~3.5 s)");
  _imuReady = _imu.begin();
}

// Packet rate, checksum errors and the 90-point view (closest reliable bin per 4 deg).
void Sensors::_snapshotLidar() {
  const uint32_t now = millis();
  if (now - _lastPpsMs >= 1000) {
    const uint32_t packets = _lidar.packets();
    _lidarPps = (packets - _lastPackets) * 1000.0f / (now - _lastPpsMs);
    _lastPackets = packets;
    _lastPpsMs = now;
  }
  _lidarOk = _lidarPps > 1;
  _lidarCrcPct = _lidar.crcErrorPct();

  const int degPerPoint = 360 / kLidarPoints;
  for (uint8_t i = 0; i < kLidarPoints; i++) _lidarCm[i] = 0;
  for (int bin = 0; bin < 360; bin++) {
    const uint16_t mm = _lidar.distanceMm(bin);
    if (!mm) continue;
    const int deg = ((bin + _lidarOffsetDeg) % 360 + 360) % 360;
    const uint8_t i = deg / degPerPoint;
    const uint16_t cm = mm / 10;
    if (_lidarCm[i] == 0 || cm < _lidarCm[i]) _lidarCm[i] = cm;
  }
}

void Sensors::_snapshotGps() {
  _gpsFix = _gps.hasFix();
  if (_gpsFix) {
    _lat = _gps.latDeg();
    _lon = _gps.lonDeg();
  }
  _sats = _gps.satellites();
  _hdop = _gps.hdop();
  _gpsChars = _gps.charsReceived();
}

// {"us":[5 x cm],"usMax":..,"lidar":{ok,pps,crc,d:[90 x cm]},"imu":{ok,yaw,pitch,roll,cal:[4]},
//  "gps":{fix,sats,hdop,lat,lon,chars}}
String Sensors::toJson() const {
  String j;
  j.reserve(720);
  j += "{\"us\":[";
  for (int i = 0; i < 5; i++) { if (i) j += ','; j += _usCm[i]; }
  j += "],\"usMax\":"; j += kUsMaxCm;
  j += ",\"lidar\":{\"ok\":"; j += _lidarOk ? "true" : "false";
  j += ",\"pps\":"; j += String(_lidarPps, 0);
  j += ",\"crc\":"; j += String(_lidarCrcPct, 1);
  j += ",\"d\":[";
  for (int i = 0; i < kLidarPoints; i++) { if (i) j += ','; j += _lidarCm[i]; }
  j += "]},\"imu\":{\"ok\":"; j += _imuReady ? "true" : "false";
  j += ",\"yaw\":"; j += String(_yaw, 1);
  j += ",\"pitch\":"; j += String(_pitch, 1);
  j += ",\"roll\":"; j += String(_roll, 1);
  j += ",\"cal\":["; j += _cal[0]; j += ','; j += _cal[1]; j += ','; j += _cal[2]; j += ','; j += _cal[3];
  j += "]},\"gps\":{\"fix\":"; j += _gpsFix ? "true" : "false";
  j += ",\"sats\":"; j += _sats;
  j += ",\"hdop\":"; j += String(_hdop, 1);
  j += ",\"lat\":"; j += String(_lat, 6);
  j += ",\"lon\":"; j += String(_lon, 6);
  j += ",\"chars\":"; j += _gpsChars;
  j += "}}";
  return j;
}
