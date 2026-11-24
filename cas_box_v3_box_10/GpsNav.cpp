#include "GpsNav.h"

// Route B -> H from log_009 (2026-10-24, manual teaching run, HDOP 0.6-0.8, 12-19 satellites):
// smoothed GPS track, one point every ~4 m. Regenerate with tools/make_route.py <log.csv|bin>.
// WP00-05: raised path along the B facade (left wall ~1.5 m) | WP10-12: corner, rover struggled
// there in log_009 (150 s, pitch up to 14 deg) | WP21: end (H).
static const GpsNav::Wp kDefaultRoute[] = {
  {48.8301848, 12.9545704},  // WP00 t=0s E+0.0 N+0.0
  {48.8301576, 12.9546080},  // WP01 t=49s E+2.8 N-3.0
  {48.8301380, 12.9546544},  // WP02 t=70s E+6.1 N-5.2
  {48.8301158, 12.9547039},  // WP03 t=97s E+9.8 N-7.7
  {48.8300954, 12.9547504},  // WP04 t=117s E+13.2 N-9.9
  {48.8300772, 12.9548000},  // WP05 t=137s E+16.8 N-12.0
  {48.8300550, 12.9548478},  // WP06 t=151s E+20.3 N-14.4
  {48.8300275, 12.9548875},  // WP07 t=161s E+23.2 N-17.5
  {48.8299992, 12.9549280},  // WP08 t=174s E+26.2 N-20.6
  {48.8299718, 12.9549643},  // WP09 t=190s E+28.8 N-23.7
  {48.8299394, 12.9549886},  // WP10 t=207s E+30.6 N-27.3
  {48.8299328, 12.9550424},  // WP11 t=271s E+34.6 N-28.0
  {48.8298989, 12.9550170},  // WP12 t=373s E+32.7 N-31.8
  {48.8298653, 12.9549966},  // WP13 t=389s E+31.2 N-35.5
  {48.8298376, 12.9549593},  // WP14 t=408s E+28.5 N-38.6
  {48.8298144, 12.9549168},  // WP15 t=427s E+25.4 N-41.2
  {48.8297894, 12.9548761},  // WP16 t=449s E+22.4 N-44.0
  {48.8297641, 12.9548369},  // WP17 t=469s E+19.5 N-46.8
  {48.8297401, 12.9547956},  // WP18 t=495s E+16.5 N-49.4
  {48.8297183, 12.9547507},  // WP19 t=513s E+13.2 N-51.9
  {48.8296977, 12.9547056},  // WP20 t=548s E+9.9 N-54.2
  {48.8296854, 12.9546645},  // WP21 t=644s E+6.9 N-55.5
};
static const uint8_t kDefaultN = sizeof(kDefaultRoute) / sizeof(kDefaultRoute[0]);

static const char* kStateName[] = {"off", "align", "follow", "done", "wait GPS"};

// ---------------------------------------------------------------------------------------------
// (the "Autonomous (GPS)" hold-to-drive card was removed; GpsNav is used by Mission)
// ---------------------------------------------------------------------------------------------
// (web card removed 2026-10-23: the page now has only manual drive + the mission card)

// ---------------------------------------------------------------------------------------------

bool GpsNav::begin(AsyncWebServer* server) {
  for (uint8_t i = 0; i < kDefaultN && i < kMaxWp; i++) _wp[i] = kDefaultRoute[i];
  _n = kDefaultN;
  if (!server) return false;
  server->on("/nav/status", HTTP_GET, [this](AsyncWebServerRequest* req) {
    req->send(200, "application/json", _statusJson());
  });
  // /nav/on|off|add|clear|default (hold-to-drive card) removed 2026-10-24: flash was full.
  return true;
}

// Web requests are handled here, in loop(), so the waypoint list is only changed by one thread.
void GpsNav::pollRequests(const Sensors& s) {
  if (_reqOff) { _reqOff = false; _state = OFF; strlcpy(_msg, "stopped", sizeof(_msg)); }
  if (_reqClear) {
    _reqClear = false;
    _state = OFF;
    _n = 0; _idx = 0;
    strlcpy(_msg, "waypoints cleared", sizeof(_msg));
  }
  if (_reqDefault) {
    _reqDefault = false;
    _state = OFF;
    for (uint8_t i = 0; i < kDefaultN && i < kMaxWp; i++) _wp[i] = kDefaultRoute[i];
    _n = kDefaultN; _idx = 0;
    strlcpy(_msg, "route B->H loaded", sizeof(_msg));
  }
  if (_reqAdd) {
    _reqAdd = false;
    if (!s.gpsFix() || s.hdop() <= 0 || s.hdop() > maxHdop) strlcpy(_msg, "no good GPS - WP not added", sizeof(_msg));
    else if (_n >= kMaxWp) strlcpy(_msg, "waypoint list full", sizeof(_msg));
    else {
      _wp[_n] = {s.lat(), s.lon()};
      _n = _n + 1;
      snprintf(_msg, sizeof(_msg), "WP%02u added", (unsigned)(_n - 1));
    }
  }
  if (_reqOn) {
    _reqOn = false;
    if (_n == 0) { strlcpy(_msg, "no waypoints", sizeof(_msg)); return; }
    _resetRun();
    _state = WAIT_GPS;
    strlcpy(_msg, "hold UP to start", sizeof(_msg));
  }
}

void GpsNav::_resetRun() {
  _idx = 0;
  _offsetOk = false;
  _histN = 0;
  _alignStartMs = 0;
  _alignTimerMs = 0;
}

// Equirectangular offset of point 1 from point 0 in metres (east, north). Fine below ~1 km.
void GpsNav::_offset(double lat0, double lon0, double lat1, double lon1, float& e, float& n) {
  const double k = 111194.93;  // m per degree
  n = (float)((lat1 - lat0) * k);
  e = (float)((lon1 - lon0) * k * cos(lat0 * DEG_TO_RAD));
}

// Keeps the last GPS fixes (new fix = lat/lon changed, 1 Hz) and refines the heading offset
// whenever the rover drove straight (yaw change <= 6 deg) at least 1.2 m within the last ~5 s.
// A 5 s baseline instead of 2 s: in the simulation (sim_gpsnav.py) this halved the worst
// cross-track error (3-5 m -> 2-3 m), because GPS jitter matters less over a longer distance.
void GpsNav::_feedGps(const Sensors& s) {
  if (!s.gpsFix() || s.hdop() <= 0 || s.hdop() > maxHdop) return;
  if (s.lat() == _lastLat && s.lon() == _lastLon) return;
  _lastLat = s.lat();
  _lastLon = s.lon();
  if (_histN == 6) { for (uint8_t i = 0; i < 5; i++) _hist[i] = _hist[i + 1]; _histN = 5; }
  _hist[_histN++] = {s.lat(), s.lon(), s.yawDeg(), millis()};

  if (!_offsetOk) {
    // First offset: learned from any straight stretch of >= alignMinM (also while another mode,
    // e.g. wall following, is driving - see observe()). A turn of > 15 deg restarts the stretch.
    const Fix& b = _hist[_histN - 1];
    if (!_alignStartMs) { _alignStart = b; _alignStartMs = millis(); return; }
    if (fabsf(_wrap180(b.yawIso - _alignStart.yawIso)) > 15.0f) { _alignStart = b; _alignStartMs = millis(); return; }
    float e, n;
    _offset(_alignStart.lat, _alignStart.lon, b.lat, b.lon, e, n);
    if (sqrtf(e * e + n * n) < alignMinM) return;
    const float cog = atan2f(e, n) * RAD_TO_DEG;
    const float yawMid = _alignStart.yawIso + 0.5f * _wrap180(b.yawIso - _alignStart.yawIso);
    _offsetDeg = _wrap180(cog + yawMid);
    _offsetOk = true;
    strlcpy(_msg, "heading learned", sizeof(_msg));
    return;
  }
  if (_histN < 6) return;
  const Fix& a = _hist[0];
  const Fix& b = _hist[_histN - 1];
  if (fabsf(_wrap180(b.yawIso - a.yawIso)) > 6.0f) return;  // was turning
  float e, n;
  _offset(a.lat, a.lon, b.lat, b.lon, e, n);
  if (sqrtf(e * e + n * n) < 1.2f) return;                   // too slow: course is noise
  const float cog = atan2f(e, n) * RAD_TO_DEG;
  const float yawMid = a.yawIso + 0.5f * _wrap180(b.yawIso - a.yawIso);
  const float meas = _wrap180(cog + yawMid);                 // offset = cog - (-yaw)
  _offsetDeg = _wrap180(_offsetDeg + 0.2f * _wrap180(meas - _offsetDeg));
}

void GpsNav::observe(const Sensors& s) { _feedGps(s); }

uint8_t GpsNav::nearestIndex(const Sensors& s) const {
  uint8_t best = 0;
  float bestD = 1e9f;
  for (uint8_t i = 0; i < _n; i++) {
    float e, n;
    _offset(s.lat(), s.lon(), _wp[i].lat, _wp[i].lon, e, n);
    const float dd = e * e + n * n;
    if (dd < bestD) { bestD = dd; best = i; }
  }
  return best;
}

bool GpsNav::startFrom(uint8_t idx) {
  if (_n == 0 || idx >= _n) return false;
  _idx = idx;
  _state = _offsetOk ? FOLLOW : WAIT_GPS;
  strlcpy(_msg, _offsetOk ? "following route" : "need heading: aligning", sizeof(_msg));
  return true;
}

void GpsNav::steer(const Sensors& s, float padL, float padR, uint32_t padAge, float& l, float& r) {
  _feedGps(s);
  const bool fresh = padAge <= 400;
  const bool hold = fresh && padL > 0 && padR > 0 && fabsf(padL - padR) < 0.001f;  // UP key held
  const float spd = hold ? padL : 0;
  if (!hold) {  // released: stop. Other keys: manual override.
    l = fresh ? padL : 0;
    r = fresh ? padR : 0;
    return;
  }
  l = r = 0;
  const bool gpsOk = s.gpsFix() && s.hdop() > 0 && s.hdop() <= maxHdop;

  switch (_state) {
    case WAIT_GPS:
      if (!gpsOk) { strlcpy(_msg, "waiting for good GPS", sizeof(_msg)); return; }
      _state = ALIGN;
      _alignTimerMs = 0;
      return;
    case ALIGN: {
      if (!gpsOk) { _state = WAIT_GPS; return; }
      if (_offsetOk) { _state = FOLLOW; return; }   // learned in _feedGps()
      if (!_alignTimerMs) { _alignTimerMs = millis(); strlcpy(_msg, "aligning: driving straight", sizeof(_msg)); }
      l = r = spd;  // straight ahead
      if (millis() - _alignTimerMs > alignMaxMs) {
        _state = OFF;
        l = r = 0;
        strlcpy(_msg, "align failed: no GPS movement", sizeof(_msg));
      }
      return;
    }
    case FOLLOW: {
      if (!gpsOk) { strlcpy(_msg, "GPS poor - waiting", sizeof(_msg)); return; }
      float hd = _wrap180(-s.yawDeg() + _offsetDeg);
      _heading = hd < 0 ? hd + 360.0f : hd;
      float e, n;
      _offset(s.lat(), s.lon(), _wp[_idx].lat, _wp[_idx].lon, e, n);
      _dist = sqrtf(e * e + n * n);
      if (_dist < arriveM) {
        _idx = _idx + 1;
        if (_idx >= _n) { _state = DONE; strlcpy(_msg, "goal reached", sizeof(_msg)); return; }
        snprintf(_msg, sizeof(_msg), "-> WP%02u", (unsigned)_idx);
        return;  // next pass steers to the new waypoint
      }
      float b = atan2f(e, n) * RAD_TO_DEG;
      _bearing = b < 0 ? b + 360.0f : b;
      _err = _wrap180(_bearing - _heading);  // + = target to the right
      // arcOnly: keep moving forward (at least half speed) and let the inner wheel slow down to 0,
      // but never reverse it -> no pivots: they need full power on this rover (logs 5-8), and
      // log_008 ended during a full-power pivot (box restart suspected).
      const float v = spd * fmaxf(arcOnly ? 0.5f : 0.0f, cosf(_err * DEG_TO_RAD)) * constrain(_dist / slowM, 0.4f, 1.0f);
      const float wMax = arcOnly ? fminf(wMaxFrac * spd, v) : wMaxFrac * spd;
      const float w = constrain(kTurn * _err, -wMax, wMax);
      l = constrain(v + w, -0.5f, 0.5f);  // L > R turns right
      r = constrain(v - w, -0.5f, 0.5f);
      return;
    }
    default:  // OFF, DONE
      return;
  }
}

String GpsNav::statusText() const {
  String t = kStateName[_state];
  t += " WP"; t += (unsigned)_idx; t += "/"; t += (unsigned)_n;
  if (_state == FOLLOW) { t += " d="; t += String(_dist, 1); t += " e="; t += String(_err, 0); }
  return t;
}

String GpsNav::_statusJson() const {
  String j;
  j.reserve(260);
  j += "{\"state\":\""; j += kStateName[_state];
  j += "\",\"idx\":"; j += (unsigned)_idx;
  j += ",\"n\":"; j += (unsigned)_n;
  j += ",\"dist\":"; j += String(_dist, 1);
  j += ",\"bearing\":"; j += String(_bearing, 0);
  j += ",\"heading\":"; j += String(_heading, 0);
  j += ",\"err\":"; j += String(_err, 0);
  j += ",\"offOk\":"; j += _offsetOk ? "true" : "false";
  j += ",\"offset\":"; j += String(_offsetDeg, 0);
  j += ",\"msg\":\""; j += _msg; j += "\"}";
  return j;
}
