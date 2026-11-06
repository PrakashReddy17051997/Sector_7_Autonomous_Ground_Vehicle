#include "GpsManager.h"

static const int kRxPin = 16;  // ESP32 RX2 <- GPS TX
static const int kTxPin = 17;  // ESP32 TX2 -> GPS RX
static const uint32_t kBaud = 9600;
static const uint32_t kMaxFixAgeMs = 3000;

void GpsManager::begin() { Serial2.begin(kBaud, SERIAL_8N1, kRxPin, kTxPin); }

void GpsManager::poll() {
  while (Serial2.available()) _gps.encode(Serial2.read());
}

bool GpsManager::hasFix() { return _gps.location.isValid() && _gps.location.age() < kMaxFixAgeMs; }
double GpsManager::latDeg() { return _gps.location.lat(); }
double GpsManager::lonDeg() { return _gps.location.lng(); }
int GpsManager::satellites() { return _gps.satellites.isValid() ? (int)_gps.satellites.value() : 0; }
float GpsManager::hdop() { return _gps.hdop.isValid() ? (float)_gps.hdop.hdop() : 0.0f; }
uint32_t GpsManager::charsReceived() { return _gps.charsProcessed(); }
