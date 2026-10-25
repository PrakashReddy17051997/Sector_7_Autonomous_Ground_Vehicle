#pragma once
// GpsManager - GY-GPS6MV2 (u-blox NEO-6M) on Serial2, GPIO16 RX / GPIO17 TX, 9600 baud.
//
// poll() feeds all received NMEA characters into TinyGPSPlus; the getters return the latest
// decoded values. charsReceived() > 0 shows that the module talks at all (useful indoors, where
// there is usually no fix).
#include <Arduino.h>
#include <TinyGPSPlus.h>

class GpsManager {
public:
  void begin();
  void poll();  // call every loop pass, reads everything waiting on the UART

  // (not const: the TinyGPSPlus getters are not const)
  bool hasFix();               // valid position not older than 3 s
  double latDeg();             // latitude, valid if hasFix()
  double lonDeg();             // longitude, valid if hasFix()
  int satellites();            // satellites used (0 if unknown)
  float hdop();                // horizontal dilution of precision (0 if unknown)
  uint32_t charsReceived();    // NMEA characters received since start

private:
  TinyGPSPlus _gps;
};
