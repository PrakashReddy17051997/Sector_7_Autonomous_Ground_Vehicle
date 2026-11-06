#pragma once
// Mission - full-auto run of the whole route: start it once, the rover drives on its own.
//
//   Plan "route" (DEFAULT): teach & repeat with RouteFollow - LiDAR + gyro leg by leg, the legs switch
//     automatically (leg end = landmark + LiDAR end scan, RouteMap.h). Page buttons (2026-10-29):
//       "Run whole route"            leg 1 -> last leg, rover placed at the start like in the teaching run
//       "Test this leg only"         drives the chosen leg and stops at its end; the page shows how
//                                    the end was found (landmark + scan / scan dip / last resort)
//       "Start here, continue"       from the chosen leg to the last one
//       "Resume"                     after a pause, a STOP or a drive key: same leg, heading kept
//     See RouteFollow.h.
//   The older GPS plans below are still in the code (/mission/gps, /mission/wall) but no longer on
//   the page (2026-10-29: GPS ruled out for navigation for now).
//
//   Plan "wall + GPS" (the B -> H route of log_009):
//     leg 1  WALL : WallFollow along the LEFT wall (B facade) at the distance it has at the start.
//                   Meanwhile GpsNav learns the true heading from the GPS course (straight stretch).
//     leg 2  GPS  : when the wall ends (facade end, ~36-40 s in log_003), GpsNav follows the route
//                   from the nearest waypoint to the last one (H).
//   Plan "GPS only": leg 2 only (starts with ALIGN: ~2 m straight to learn the heading).
//
// Heading: seeded at the start from the facade direction + the measured wall angle (no 2 m blind
// drive), then corrected by the GPS course. The switch wall -> GPS happens automatically.
//
// SAFETY (keeps the stop rules of the starter program):
//   - The open web page sends a heartbeat every 250 ms while the mission runs. No heartbeat for
//     heartbeatMs = 0.8 s (page closed / hidden, phone locked, Wi-Fi lost) -> the rover PAUSES (stops) and
//     continues when the heartbeat is back. RoverLink's deadman and the rover's own 3 s stop stay.
//   - STOP button, or ANY drive-pad key -> mission aborted, manual control.
//   - SafetyGuard and the motor shaper still act on every command.
//   - If the heading was not learned during the wall leg, the mission stops at the facade end
//     instead of driving 2 m straight near the edge of the raised path.
//
//   setup(): mission.begin(hmi.server());
//   loop():  mission.pollRequests(sensors, wall, nav);
//            if (mission.active()) mission.steer(sensors, wall, nav, padL, padR, padAge, l, r);
#include <Arduino.h>
#include <ESPAsyncWebServer.h>
#include "Sensors.h"
#include "WallFollow.h"
#include "GpsNav.h"
#include "RouteFollow.h"

class Mission {
public:
  enum State : uint8_t { IDLE = 0, WALL_LEG = 1, GPS_LEG = 2, DONE = 3, ABORTED = 4, ROUTE = 5 };

  uint32_t heartbeatMs = 800;            // no heartbeat this long -> pause (short Wi-Fi gaps do not stop it)
  float minSpeed = 0.10f, maxSpeed = 0.30f;   // route: drive at the teaching speed (log_013: pad 0.30)
  WallFollow::Side wallSide = WallFollow::LEFT;  // B facade is on the left (log_003)
  // Heading seed: true direction of travel along the B facade (clockwise from north).
  // From log_009 (GPS course while following the facade, 5-140 s, 21 m, lateral spread 0.24 m):
  // 125 deg. (log_003 gave 118 deg over only 2.9 m.)
  // Used only when the mission starts within seedMaxDistM of the route start (WP00); GPS course
  // corrects it afterwards on every straight stretch.
  float facadeBearingDeg = 125.0f;
  float seedMaxDistM = 10.0f;

  bool begin(AsyncWebServer* server);
  void pollRequests(const Sensors& s, WallFollow& wall, GpsNav& nav);
  bool active() const { return _state == WALL_LEG || _state == GPS_LEG || _state == ROUTE; }
  bool paused() const { return _paused; }
  void steer(const Sensors& s, WallFollow& wall, GpsNav& nav, float padL, float padR, uint32_t padAge,
             float& l, float& r);
  String statusText() const;
  // every loop pass, after guard + shaper: what went to the rover (route progress = effort)
  void feedOutput(float l, float r) { route.feedOutput(l, r); }
  uint8_t logByte() const { return route.logByte(); }   // route leg / state for DriveLog
  uint8_t logLeg() const { return route.logLeg(); }     // full leg number for DriveLog (v2)
  RouteFollow route;

private:
  void _finish(State st, const char* msg, WallFollow& wall, GpsNav& nav);
  String _statusJson() const;

  volatile bool _reqWall = false, _reqGps = false, _reqStop = false, _reqRoute = false, _reqResume = false;
  volatile bool _reqOnly = false;      // route request: this leg only (segment test)
  volatile uint8_t _reqLeg = 0;
  volatile float _reqSpeed = 0.2f;
  volatile uint32_t _aliveMs = 0;

  volatile uint8_t _state = IDLE;
  volatile bool _paused = false;
  float _speed = 0.2f;
  uint32_t _legStartMs = 0;
  char _msg[48] = "";
  // copies for the page (written in loop, read by the web task)
  volatile uint8_t _wp = 0, _wpN = 0;
  volatile bool _headingOk = false;
  char _legInfo[48] = "";
  volatile uint8_t _rLeg = 0, _rState = 0, _rEffPct = 0;
  volatile bool _rCanResume = false;
  char _endInfo[72] = "";              // how the last leg ended (copy of route.lastEnd())
  mutable portMUX_TYPE _mux = portMUX_INITIALIZER_UNLOCKED;
  // Route map card (/route/live): copies written in pollRequests, read by the web task
  String _liveJson() const;
  uint32_t _liveMs = 0;
  bool _liveRun = false;
  uint8_t _liveLeg = 0, _liveSt = 0;
  float _liveFrac = 0, _liveRy = 0;
  uint8_t _liveLp = 0;            // local path: 0 off, 1 on but not locked, 2 locked
  float _liveWx = 0, _liveWy = 0, _liveGx = 0, _liveGy = 0, _liveGth = 0, _liveLs = 0, _liveLe = 0, _liveOff = 0;
  uint16_t _liveScan[Sensors::kLidarPoints] = {0};
};
