#pragma once
// RoverLink - box side of the Wi-Fi link to the WAVE ROVER.
//
// The rover joins the box access point as a station and runs its unchanged Waveshare firmware.
// Commands are JSON sent to its HTTP endpoint: GET http://<rover>/js?json={"T":1,"L":..,"R":..}
// All network I/O runs in an own FreeRTOS task on core 0, so loop() never waits for the rover.
//
//   rover.begin();
//   rover.setDrive(l, r);  // call every loop pass; the task sends it every 100 ms ("T":1), and
//                          // AT ONCE when the command changes (2026-10-29: a new command used to
//                          // wait up to 100 ms for the next slot; min. kMinGapMs between commands)
//
// When l = r = 0 the task sends stops until the rover confirmed three of them, then every
// 500 ms alternately repeats the stop and polls the rover status ("T":130, battery voltage).
// Deadman: a set point older than 500 ms counts as 0/0, so the rover stops even if loop() hangs. The rover itself also stops after 3 s without a command.
#include <Arduino.h>
#include <WiFi.h>

class RoverLink {
public:
  void begin(uint32_t periodMs = 100);   // starts the link task

  // Wheel speeds per side, -0.5..0.5 (0.5 = full speed). Thread-safe.
  void setDrive(float left, float right);

  bool connected() const { return _ipRaw != 0; }
  IPAddress ip() const { return IPAddress(_ipRaw); }   // 0.0.0.0 while searching
  float voltage() const { return _voltage; }           // rover battery in V, 0 = unknown
  float rttMs() const { return _rttMs; }               // smoothed round-trip time per command
  uint32_t okCount() const { return _ok; }             // successful commands since start
  uint32_t failCount() const { return _fail; }         // failed commands since start
  String statusText() const;                           // one line for the web page

private:
  static void _taskEntry(void* self);
  void _task();
  bool _find();
  int _cmd(uint32_t ip, const String& json, String* reply);

  uint32_t _periodMs = 100;
  portMUX_TYPE _mux = portMUX_INITIALIZER_UNLOCKED;  // guards the set point
  float _setL = 0, _setR = 0;
  uint32_t _setMs = 0;

  // written by the task, read by loop(); 32-bit values are atomic on the ESP32
  volatile uint32_t _ipRaw = 0;
  volatile float _voltage = 0, _rttMs = 0;
  volatile uint32_t _ok = 0, _fail = 0;
  uint8_t _failRow = 0;  // task only
  TaskHandle_t _taskHandle = nullptr;
  float _lastL = 0, _lastR = 0;          // loop() side: last set point (change detection)
};
