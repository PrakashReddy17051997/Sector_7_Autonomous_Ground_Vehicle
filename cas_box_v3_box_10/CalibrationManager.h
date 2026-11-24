#pragma once
// CalibrationManager (19.11.2026, from the teammate's draft): per-box corrections kept in the ESP32 flash
// (Preferences, namespace "rovercal"), so a new box / rover needs no code change and no reflash.
//   lidar  LiDAR mounting angle (deg, "for our maps" value of lidar.cal). Measured ONCE PER BOX: rover standing
//          parallel to a long straight wall, button "Save LiDAR calibration" on the page (cas_box_v3.ino). It cannot be
//          measured at boot: from one scan at a random start pose, "LiDAR turned 7 deg" and "rover turned 7 deg" look
//          the same. Used from the NEXT boot (Sensors::begin takes it once).
//   trim   wheel trim the route STARTS with. Learned while rolling straight (RouteFollow trimLearn) and saved after a
//          mission, but only with safeguards (saveLearnedTrim): >= minLearnS of learning in that mission, at most
//          maxTrimStep away from the stored value per mission, |trim| <= maxTrim. "Reset trim" on the page = default.
// Nothing here changes while the rover drives.
#include <Arduino.h>
#include <Preferences.h>
#include <math.h>

// 19.11.: the "Calibration (this box)" card on the web page (served by cas_box_v3.ino at /cal.js)
static const char kCalJs[] PROGMEM = R"rawjs((function(){
const c=document.createElement('div');c.className='card';
c.innerHTML="<div class='sectionTitle'>Calibration (this box)</div>"
 +"<div class='value' id='clInfo'>-</div><div class='label' id='clMeas'></div>"
 +"<div style='display:flex;gap:8px;flex-wrap:wrap;margin-top:8px'>"
 +"<button class='btnTiny' id='clSave'>Save LiDAR calibration</button>"
 +"<button class='btnTiny' id='clLR'>LiDAR to default</button>"
 +"<button class='btnTiny' id='clTR'>Trim to default</button></div>"
 +"<div class='label' id='clNote'></div>"
 +"<div class='label'>LiDAR: rover STANDING parallel to a long straight wall, then Save (needs 20 scans within 1 deg). Used after the next restart. Trim: saved by itself after a mission with &gt;= 60 s of straight rolling, max 0.10 change per mission.</div>";
const w=document.querySelector('.wrap');w.appendChild(c);
const $=id=>document.getElementById(id);
const go=u=>fetch(u,{cache:'no-store'}).then(st);
$('clSave').onclick=()=>go('/cal/lidar');$('clLR').onclick=()=>go('/cal/lidar_reset');$('clTR').onclick=()=>go('/cal/trim_reset');
async function st(){try{const j=await (await fetch('/cal.json',{cache:'no-store'})).json();
 $('clInfo').textContent=j.text;$('clMeas').textContent=j.meas;$('clNote').textContent=j.note;}catch(e){}}
setInterval(st,2000);st();
})();)rawjs";


class CalibrationManager {
public:
  float minLearnS = 60.0f;     // a mission must have learned the trim this long (s of straight rolling) to save it
  float maxTrimStep = 0.10f;   // the stored trim moves at most this much per mission
  float maxTrim = 0.50f;       // never store more than this

  bool begin(int defLidarDeg, float defTrim) {
    _defLidar = defLidarDeg; _defTrim = defTrim;
    _lidar = defLidarDeg; _trim = defTrim;
    if (!_prefs.begin("rovercal", false)) { _ok = false; return false; }
    _ok = true;
    _lidarSaved = _prefs.isKey("lidar");
    _trimSaved = _prefs.isKey("trim");
    if (_lidarSaved) _lidar = _prefs.getInt("lidar", defLidarDeg);
    if (_trimSaved) _trim = _prefs.getFloat("trim", defTrim);
    if (_lidar < 0 || _lidar >= 360) { _lidar = defLidarDeg; _lidarSaved = false; }
    if (!isfinite(_trim) || fabsf(_trim) > maxTrim) { _trim = defTrim; _trimSaved = false; }
    return true;
  }
  int lidarDeg() const { return _lidar; }
  float trim() const { return _trim; }

  bool saveLidar(int deg) {
    deg %= 360; if (deg < 0) deg += 360;
    if (!_ok || _prefs.putInt("lidar", deg) != sizeof(int32_t)) return false;
    _lidar = deg; _lidarSaved = true; return true;
  }
  void resetLidar() { if (_ok) _prefs.remove("lidar"); _lidar = _defLidar; _lidarSaved = false; }

  // after a mission: learned = the route's trim now, learnS = seconds it learned in this mission.
  // Returns a short note for the page / Serial.
  String saveLearnedTrim(float learned, float learnS) {
    if (!_ok) return "no flash";
    if (!isfinite(learned)) return "not saved (invalid)";
    if (learnS < minLearnS) return "not saved (" + String(learnS, 0) + " s learned, need " + String(minLearnS, 0) + ")";
    float t = _trim + constrain(learned - _trim, -maxTrimStep, maxTrimStep);
    t = constrain(t, -maxTrim, maxTrim);
    if (fabsf(t - _trim) < 0.005f) return "unchanged";
    if (_prefs.putFloat("trim", t) != sizeof(float)) return "save failed";
    const float old = _trim; _trim = t; _trimSaved = true;
    return "saved " + String(old, 3) + " -> " + String(t, 3) + " (learned " + String(learned, 3) + ")";
  }
  void resetTrim() { if (_ok) _prefs.remove("trim"); _trim = _defTrim; _trimSaved = false; }

  String text() const {
    return String("LiDAR ") + _lidar + (_lidarSaved ? " (saved)" : " (default)") + ", start trim " + String(_trim, 3) +
           (_trimSaved ? " (saved)" : " (default)") + (_ok ? "" : ", NO FLASH");
  }
  void print() const { Serial.println("[cal] " + text()); }

private:
  Preferences _prefs;
  bool _ok = false, _lidarSaved = false, _trimSaved = false;
  int _lidar = 90, _defLidar = 90;
  float _trim = 0.25f, _defTrim = 0.25f;
};
