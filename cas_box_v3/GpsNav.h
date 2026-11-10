#pragma once
// GpsNav - first autonomous mode: follows a list of GPS waypoints.
//
// SAFETY: the rover moves on its own ONLY while you HOLD the "up" key on the drive pad (dead-man
// switch). Release = stop, as before. Left / right / back keys are manual override. The slider
// sets the maximum speed. SafetyGuard and the acceleration ramp still act after GpsNav.
//
// Heading: log_003 showed the compass is off by 10..105 deg (median 57). So GpsNav does NOT trust
// the absolute compass. It uses the BNO055 yaw only for CHANGES of heading (smooth, short-term
// good) and learns the offset to true north from the GPS course while driving straight:
//     heading_true = -yaw_iso + offset            offset <- GPS course - (-yaw_iso)
//   ALIGN  : after "Auto on" the rover first drives straight until it has moved >= 2 m by GPS
//            (max 10 s) and learns the offset. Give it free space straight ahead!
//   FOLLOW : steer to the next waypoint; "arrived" within arriveM; keeps refining the offset on
//            every straight stretch.
//   DONE   : last waypoint reached -> stop.
// Waypoints: the built-in route (from log_003, every ~8 m) or your own: "Clear" then walk / drive
// to a spot and press "Add WP here" (up to kMaxWp).
//
//   setup(): nav.begin(hmi.server());
//   loop():  if (nav.active()) nav.steer(sensors, padL, padR, padAge, l, r);   (see cas_box.ino)
#include <Arduino.h>
#include <math.h>
#include <ESPAsyncWebServer.h>
#include "Sensors.h"

class GpsNav {
public:
  struct Wp { double lat, lon; };
  enum State : uint8_t { OFF = 0, ALIGN = 1, FOLLOW = 2, DONE = 3, WAIT_GPS = 4 };

  // ---- tuning ----
  float arriveM = 3.0f;      // waypoint reached (GPS noise ~ +-1.5 m in log_003)
  float slowM = 6.0f;        // slow down below this distance
  float kTurn = 0.006f;      // turn command per degree of heading error (30 deg -> 0.18)
  float wMaxFrac = 1.0f;     // max turn as a fraction of the speed: 1.0 = inner wheel may stop
                             // (small differences did not turn the rover, logs 5-8)
  bool arcOnly = true;       // turn in arcs (inner wheel >= 0), never pivot on the spot
  float maxHdop = 2.5f;      // worse GPS -> wait (stop)
  float alignMinM = 2.0f;    // GPS distance needed to learn the heading offset
  uint32_t alignMaxMs = 10000;

  static const uint8_t kMaxWp = 40;

  bool begin(AsyncWebServer* server);
  bool active() const { return _state != OFF; }

  // Called every loop pass while active(). padL/padR/padAge: the drive pad command.
  // Writes the wheel command to l / r (-0.5..0.5).
  void steer(const Sensors& s, float padL, float padR, uint32_t padAge, float& l, float& r);

  String statusText() const;
  uint8_t state() const { return _state; }

  // ---- for Mission (full auto) ----
  void observe(const Sensors& s);           // learn the heading while another mode drives
  bool headingKnown() const { return _offsetOk; }
  uint8_t nearestIndex(const Sensors& s) const;
  float distToWp(const Sensors& s, uint8_t idx) const {  // metres, 1e9 if no such waypoint
    if (idx >= _n) return 1e9f;
    float e, n; _offset(s.lat(), s.lon(), _wp[idx].lat, _wp[idx].lon, e, n); return sqrtf(e * e + n * n);
  }
  uint8_t count() const { return _n; }
  bool startFrom(uint8_t idx);              // FOLLOW from waypoint idx (ALIGN first if needed)
  void forgetHeading() { _resetRun(); }
  void stop() { _state = OFF; }
  // Heading seed: the true heading is known right now (e.g. parallel to a wall of known direction).
  void seedHeading(float trueHeadingDeg, float yawIsoDeg) {
    _offsetDeg = _wrap180(trueHeadingDeg + yawIsoDeg);  // heading = -yaw + offset
    _offsetOk = true;
  }
  float headingDeg(float yawIsoDeg) const { float h = _wrap180(-yawIsoDeg + _offsetDeg); return h < 0 ? h + 360 : h; }
  uint8_t wpIndex() const { return _idx; }

private:
  void _feedGps(const Sensors& s);
  void _resetRun();
  String _statusJson() const;
  static float _wrap180(float a) { a = fmodf(a + 180.0f, 360.0f); if (a < 0) a += 360.0f; return a - 180.0f; }
  static void _offset(double lat0, double lon0, double lat1, double lon1, float& e, float& n);

  // requests from the web task, handled in steer() / loop
  volatile bool _reqOn = false, _reqOff = false, _reqAdd = false, _reqClear = false, _reqDefault = false;

  volatile uint8_t _state = OFF;
  Wp _wp[kMaxWp];
  volatile uint8_t _n = 0, _idx = 0;

  // heading estimation
  bool _offsetOk = false;
  float _offsetDeg = 0;
  float _heading = 0;           // estimated true heading (clockwise from north)
  double _lastLat = 0, _lastLon = 0;
  struct Fix { double lat, lon; float yawIso; uint32_t ms; };
  Fix _hist[6];                 // last GPS fixes (1 Hz)
  uint8_t _histN = 0;
  uint32_t _alignStartMs = 0;    // start of the current straight stretch (heading learning)
  uint32_t _alignTimerMs = 0;    // start of the ALIGN state (timeout)
  Fix _alignStart = {0, 0, 0, 0};  // GPS fix where ALIGN started

  // last values for the page
  float _dist = 0, _bearing = 0, _err = 0;
  char _msg[40] = "";

public:
  void pollRequests(const Sensors& s);  // call every loop pass (also when not active)
};
