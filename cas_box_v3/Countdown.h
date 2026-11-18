#pragma once
// Countdown - start the whole route at a set time (18.11.2026, course rule of 17.11.):
//   display  "Countdown ACTIVE" + seconds to the start, or "Countdown INACTIVE"
//   web      card "Countdown": start time (local time of the phone) + Activate / Deactivate (/countdown)
//   tap      a double tap on the box (IMU) deactivates it
//   clock    UTC from the u-blox module (Sensors::gpsSod); between NMEA seconds it runs on millis().
//            No GPS time yet -> it stays ACTIVE but cannot count ("waiting for GPS time") and never starts.
//   start    at 0 s: poll() returns true once (the sketch asks the mission for the whole route), then INACTIVE.
#include <Arduino.h>
#include <ESPAsyncWebServer.h>
#include "Sensors.h"

static const char kCountdownJs[] PROGMEM = R"rawjs((function(){
const c=document.createElement('div');c.className='card';
c.innerHTML="<div class='sectionTitle'>Countdown start</div>"
 +"<div style='display:flex;gap:8px;flex-wrap:wrap;align-items:center'>"
 +"<input id='cdT' type='time' step='1' style='padding:6px'>"
 +"<button class='btnTiny' id='cdOn'>Activate</button>"
 +"<button class='btnTiny' id='cdOff'>Deactivate</button></div>"
 +"<div class='value' id='cdInfo' style='margin-top:10px'>-</div>"
 +"<div class='label'>Start time = local time of this phone. Starts the whole route at 0 s (speed = drive slider). Double tap on the box = deactivate.</div>";
const w=document.querySelector('.wrap');w.insertBefore(c,w.children[1]||null);
const $=id=>document.getElementById(id);
const n=new Date(Date.now()+120000);$('cdT').value=n.toTimeString().slice(0,8);
const spd=()=>{const s=document.getElementById('spd');return s?parseFloat(s.value):0.2;};
$('cdOn').onclick=()=>fetch('/countdown?on=1&t='+encodeURIComponent($('cdT').value)+'&tz='+new Date().getTimezoneOffset()+'&spd='+spd().toFixed(2),{cache:'no-store'}).then(st);
$('cdOff').onclick=()=>fetch('/countdown?on=0',{cache:'no-store'}).then(st);
async function st(){try{const j=await (await fetch('/countdown.json',{cache:'no-store'})).json();
 $('cdInfo').textContent=j.line+'  |  GPS time '+(j.utc||'none')+(j.why?('  |  '+j.why):'');
 $('cdInfo').style.color=j.active?'#ffb020':'';}catch(e){}}
setInterval(st,1000);st();
})();)rawjs";

class Countdown {
public:
  void begin(AsyncWebServer* server) {
    if (!server) return;
    server->on("/countdown.js", HTTP_GET, [](AsyncWebServerRequest* req) {
      req->send(200, "application/javascript", kCountdownJs);
    });
    server->on("/countdown", HTTP_GET, [this](AsyncWebServerRequest* req) {
      const bool on = req->hasParam("on") && req->getParam("on")->value() == "1";
      if (!on) { _reqOff = true; req->send(200, "application/json", "{\"ok\":true}"); return; }
      if (!req->hasParam("t")) { req->send(400, "application/json", "{\"ok\":false,\"err\":\"no time\"}"); return; }
      int h = 0, m = 0, sec = 0;
      if (sscanf(req->getParam("t")->value().c_str(), "%d:%d:%d", &h, &m, &sec) < 2) {
        req->send(400, "application/json", "{\"ok\":false,\"err\":\"time hh:mm[:ss]\"}"); return; }
      const long tzMin = req->hasParam("tz") ? req->getParam("tz")->value().toInt() : 0;   // JS: UTC - local
      long sod = (long)h * 3600L + m * 60L + sec + tzMin * 60L;   // local -> UTC
      sod %= 86400L; if (sod < 0) sod += 86400L;
      _reqSpd = req->hasParam("spd") ? req->getParam("spd")->value().toFloat() : 0.2f;
      _reqSod = sod; _reqOn = true;
      req->send(200, "application/json", "{\"ok\":true}");
    });
    server->on("/countdown.json", HTTP_GET, [this](AsyncWebServerRequest* req) {
      char line[32]; this->line(line, sizeof(line));
      String j = "{\"active\":"; j += _active ? "true" : "false";
      j += ",\"remain\":" + String(_remain) + ",\"line\":\"" + line + "\",\"utc\":\"" + _utc + "\",\"why\":\"" + _why + "\"}";
      req->send(200, "application/json", j);
    });
  }

  // every loop pass. true ONCE when the start time is reached.
  bool poll(Sensors& s) {
    const uint32_t now = millis();
    long sod; uint32_t age;
    if (s.gpsSod(sod, age)) { _syncSod = sod; _syncMs = now - age; if (!_syncMs) _syncMs = 1; }
    const bool timeOk = _syncMs != 0 && now - _syncMs < 600000UL;     // run on millis() for up to 10 min
    long nowSod = 0;
    if (timeOk) {
      nowSod = (_syncSod + (long)((now - _syncMs) / 1000UL)) % 86400L;
      snprintf(_utc, sizeof(_utc), "%02ld:%02ld:%02ld UTC", nowSod / 3600, (nowSod / 60) % 60, nowSod % 60);
    }
    // double tap (IMU) -> deactivate
    const uint16_t taps = s.imu().doubleTaps();
    if (!_tapInit) { _taps = taps; _tapInit = true; }
    if (taps != _taps) { _taps = taps; if (_active) { _active = false; strlcpy(_why, "double tap", sizeof(_why)); Serial.println("[countdown] double tap -> INACTIVE"); } }
    if (_reqOff) { _reqOff = false; if (_active) { _active = false; strlcpy(_why, "web page", sizeof(_why)); Serial.println("[countdown] web -> INACTIVE"); } }
    if (_reqOn) {
      _reqOn = false; _active = true; _startSod = _reqSod; speed = _reqSpd; _why[0] = 0;
      Serial.printf("[countdown] ACTIVE, start %02ld:%02ld:%02ld UTC\n", _startSod / 3600, (_startSod / 60) % 60, _startSod % 60);
    }
    if (!_active) { _remain = 0; return false; }
    if (!timeOk) { _remain = -1; strlcpy(_why, "waiting for GPS time", sizeof(_why)); return false; }
    long r = _startSod - nowSod;
    while (r < -43200L) r += 86400L;
    while (r > 43200L) r -= 86400L;
    _remain = r;
    if (_why[0] == 'w') _why[0] = 0;                                    // GPS time is there now
    if (r > 0) return false;
    if (r < -60) { _active = false; strlcpy(_why, "start time passed", sizeof(_why)); return false; }
    _active = false; _remain = 0; strlcpy(_why, "started", sizeof(_why));
    Serial.println("[countdown] 0 s -> mission start");
    return true;
  }

  bool active() const { return _active; }
  long remaining() const { return _remain; }       // s; -1 = no GPS time
  float speed = 0.2f;                               // drive slider when it was activated
  void line(char* buf, size_t n) const {
    if (!_active) snprintf(buf, n, "Countdown INACTIVE");
    else if (_remain < 0) snprintf(buf, n, "Countdown ACTIVE no GPS");
    else snprintf(buf, n, "Countdown ACTIVE %lds", _remain);
  }
  void line2(char* buf, size_t n) const {          // second display line
    if (_active && _remain >= 0) snprintf(buf, n, "%ld s to start", _remain);
    else snprintf(buf, n, "%s", _why[0] ? _why : _utc);
  }

private:
  volatile bool _reqOn = false, _reqOff = false;
  volatile long _reqSod = 0;
  volatile float _reqSpd = 0.2f;
  bool _active = false, _tapInit = false;
  long _startSod = 0, _remain = 0, _syncSod = 0;
  uint32_t _syncMs = 0;
  uint16_t _taps = 0;
  char _why[24] = "";
  char _utc[16] = "";
};
