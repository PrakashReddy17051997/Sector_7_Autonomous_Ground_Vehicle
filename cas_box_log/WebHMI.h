#pragma once
// WebHMI - Wi-Fi access point, web server and web page of the box.
//
// Page (phone): state line, drive pad (hold a key = drive, release = stop), speed slider,
// LiDAR top view + ultrasonic, IMU/GPS, and a "Dev" view with named values and the loop time.
// The page polls /values every 250 ms; drive keys call /drive every 150 ms while held.
//
// Routes:  /                 the page
//          /values           JSON: state, dev values, loop time, sensor snapshot
//          /drive?l=..&r=..&id=..&n=..  wheel speeds from the drive pad; id = page instance,
//                            n = rising number (older commands that arrive late are ignored)
//
// Threads: the web server runs in its own task. Everything the page reads is guarded by a mutex,
// the drive set point by a spinlock, so loop() and the server never see half-written data.
#include <Arduino.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>

class WebHMI {
public:
  static const uint8_t kMaxValues = 16;  // named values in the "Dev" view

  // Starts the access point (password min. 8 characters) and the web server.
  void begin(const char* ssid, const char* pass, IPAddress ip, uint8_t channel);

  void setTitle(const String& title);           // page title, set before the first page load
  void enableDrivePad(float maxSpeed);          // shows the drive pad, slider max = maxSpeed
  void setStateText(const String& text);        // state line at the top of the page
  void setSensorJson(const String& json);       // sensor snapshot, see Sensors::toJson()
  void showValue(const char* name, double value);         // named value in the "Dev" view
  void showValue(const char* name, const String& value);

  // Latest drive command from the page; ageMs = time since the page last sent it.
  void getDrive(float& left, float& right, uint32_t& ageMs) const;

  void tick();  // call once per loop pass (measures the loop time for the "Dev" view)

  // Added for DriveLog: the server, so other modules can add their own routes.
  // Valid after begin() (nullptr before).
  AsyncWebServer* server() const { return _server; }

private:
  struct Value {
    char name[24];
    char text[32];
    bool inUse;
  };

  void _lock();  // takes _mtx (release with xSemaphoreGive)
  void _setValue(const char* name, const String& text);
  String _buildPage() const;
  String _buildJson() const;

  AsyncWebServer* _server = nullptr;
  String _title = "CAS Box";
  bool _drivePad = false;
  float _driveMax = 0.5f;

  SemaphoreHandle_t _mtx = nullptr;   // guards _state, _sensorJson, _values, _cycle*
  String _state = "Starting ...";
  String _sensorJson;
  Value _values[kMaxValues] = {};
  float _cycleMinMs = 0, _cycleMaxMs = 0, _cycleAvgMs = 0;

  mutable portMUX_TYPE _driveMux = portMUX_INITIALIZER_UNLOCKED;  // guards the drive set point
  float _driveL = 0, _driveR = 0;
  uint32_t _driveMs = 0;            // 0 = never received
  uint32_t _drivePageId = 0, _driveSeq = 0;  // newest accepted command (page id, number)

  // loop time statistics, loop() only
  uint32_t _cycleLastUs = 0, _cycleWinMs = 0, _cycleMinUs = 0, _cycleMaxUs = 0, _cycleCount = 0;
  uint64_t _cycleSumUs = 0;
};
