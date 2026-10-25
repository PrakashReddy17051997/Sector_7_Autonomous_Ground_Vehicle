#pragma once
// Sensors - all box sensors behind one object.
//
// Load budget: the ultrasonic sensors run in their own task on core 0 (UsManager); LiDAR, GPS and
// IMU are read in update() on core 1. The web page never touches a sensor: it gets a compact
// snapshot (toJson) that is rebuilt 4 times per second; the LiDAR is reduced to 90 points of 4 deg.
// Missing sensors never block: the IMU is started only once it answers on I2C (retry every 5 s).
#include <Arduino.h>
#include "UsManager.h"
#include "LidarScan.h"
#include "GpsManager.h"
#include "ImuManager.h"

class Sensors {
public:
  static const uint8_t kLidarPoints = 90;  // LiDAR points of 4 deg, index 0 = front, clockwise
  static const int kUsMaxCm = 300;         // ultrasonic: "nothing in range" reads as this value

  // Starts I2C and all sensors. lidarOffsetDeg rotates the LiDAR view so that point 0 is the box
  // front (90 for the box as mounted).
  void begin(int lidarOffsetDeg);
  void update();               // call every loop pass (core 1)
  String toJson() const;       // snapshot for the web page (~650 bytes)

  // Ultrasonic distance in cm, updated 4 times per second (kUsMaxCm = nothing in range).
  // i: 0 front, 1 left, 2 right, 3 back, 4 front angled
  int usCm(uint8_t i) const { return i < 5 ? _usCm[i] : kUsMaxCm; }

private:
  void _snapshotLidar();
  void _snapshotGps();
  void _startImuIfPresent();

  UsManager _us;
  LidarScan _lidar;
  GpsManager _gps;
  ImuManager _imu;

  int _lidarOffsetDeg = 0;
  uint32_t _lastSnapMs = 0, _lastImuTryMs = 0, _lastPpsMs = 0, _lastPackets = 0;

  // snapshot values (loop() only)
  int _usCm[5] = {kUsMaxCm, kUsMaxCm, kUsMaxCm, kUsMaxCm, kUsMaxCm};
  bool _lidarOk = false;
  float _lidarPps = 0, _lidarCrcPct = 0;
  uint16_t _lidarCm[kLidarPoints] = {0};  // closest reliable return per 4 deg in cm, 0 = none
  bool _imuReady = false;
  float _yaw = 0, _pitch = 0, _roll = 0;
  uint8_t _cal[4] = {0, 0, 0, 0};
  bool _gpsFix = false;
  int _sats = 0;
  float _hdop = 0;
  double _lat = 0, _lon = 0;
  uint32_t _gpsChars = 0;
};

bool i2cPresent(uint8_t addr);  // true if a device acknowledges on the I2C bus
