#include "Mission.h"

static const char* kMissionState[] = {"idle", "leg 1: wall", "leg 2: GPS", "done", "stopped", "route"};
static const char* kRouteState[] = {"off", "turn", "drive", "avoid", "back up", "paused", "done"};

// ---------------------------------------------------------------------------------------------
// /mission.js - "Mission (full auto)" card at the top of the page + heartbeat while running.
// ---------------------------------------------------------------------------------------------
static const char kMissionJs[] PROGMEM = R"rawjs((function(){
const c=document.createElement('div');c.className='card';
c.innerHTML="<div class='sectionTitle'>Mission (full auto) - keep this page open and in front</div>"
 +"<div style='display:flex;gap:8px;flex-wrap:wrap;align-items:center'>"
 +"<button class='btnTiny' id='msR'>Run route B &rarr; H (log_009)</button>"
 +"<button class='btnTiny' id='msC'>Resume</button>"
 +"<select id='msL' style='padding:6px'></select><button class='btnTiny' id='msA'>Start at this leg</button>"
 +"<button class='btnTiny' id='msG'>GPS route only</button></div>"
 +"<button class='stop' id='msS' style='width:100%;padding:18px;margin-top:10px;font-size:20px'>&#9632; STOP</button>"
 +"<div class='value' id='msInfo' style='margin-top:10px'>-</div><div class='label' id='msMore'></div>"
 +"<div class='label'>Speed = drive slider, use 0.20 (as taught). Start: rover parallel to the B facade (left wall). "
 +"Start at leg N: rover placed parallel to that leg. Any drive key pauses - then Resume.</div>";
const d=document.getElementById('driveCard'),w=document.querySelector('.wrap');
w.insertBefore(c,d||w.children[2]);
const $=id=>document.getElementById(id);
let running=false;
fetch('/mission/legs',{cache:'no-store'}).then(r=>r.json()).then(a=>{a.forEach((n,i)=>{const o=document.createElement('option');o.value=i;o.textContent=(i+1)+': '+n;$('msL').appendChild(o);});}).catch(()=>{});
async function st(){try{
  const j=await (await fetch('/mission/status',{cache:'no-store'})).json();
  running=j.running;
  $('msInfo').textContent='Mission: '+j.state+(j.paused?'  (PAUSED - no heartbeat)':'')+(j.msg?('  |  '+j.msg):'');
  $('msInfo').style.color=running?(j.paused?'#ffb020':'#4ea1ff'):(j.canResume?'#ffb020':'');
  $('msMore').textContent=j.rleg?('leg '+j.rleg+'/'+j.rn+' '+j.rstate+'  |  effort '+j.reff+' %  |  speed '+j.spd):(j.leg+'  |  WP '+j.wp+'/'+j.n+'  |  speed '+j.spd);
  $('msC').disabled=!j.canResume||running;
}catch(e){}}
const spd=()=>{const s=document.getElementById('spd');return s?parseFloat(s.value):0.2;};
const go=u=>fetch(u,{cache:'no-store'}).then(()=>{running=true;st();});
$('msR').onclick=()=>go('/mission/route?leg=0&spd='+spd().toFixed(2));
$('msA').onclick=()=>go('/mission/route?leg='+$('msL').value+'&spd='+spd().toFixed(2));
$('msC').onclick=()=>go('/mission/resume?spd='+spd().toFixed(2));
$('msG').onclick=()=>go('/mission/gps?spd='+spd().toFixed(2));
$('msS').onclick=()=>fetch('/mission/stop',{cache:'no-store'}).then(st);
setInterval(()=>{if(running&&!document.hidden)fetch('/mission/alive',{cache:'no-store'}).catch(()=>{});},250);
setInterval(st,1000);st();
})();)rawjs";

// ---------------------------------------------------------------------------------------------

bool Mission::begin(AsyncWebServer* server) {
  if (!server) return false;
  server->on("/mission.js", HTTP_GET, [](AsyncWebServerRequest* req) {
    req->send(200, "application/javascript", kMissionJs);
  });
  server->on("/mission/status", HTTP_GET, [this](AsyncWebServerRequest* req) {
    req->send(200, "application/json", _statusJson());
  });
  server->on("/mission/alive", HTTP_GET, [this](AsyncWebServerRequest* req) {
    _aliveMs = millis();
    req->send(200, "application/json", "{\"ok\":true}");
  });
  auto startReq = [this](AsyncWebServerRequest* req, bool wall) {
    float sp = 0.2f;
    if (req->hasParam("spd")) sp = req->getParam("spd")->value().toFloat();
    _reqSpeed = sp;
    _aliveMs = millis();
    if (wall) _reqWall = true; else _reqGps = true;
    req->send(200, "application/json", "{\"ok\":true}");
  };
  server->on("/mission/wall", HTTP_GET, [startReq](AsyncWebServerRequest* req) { startReq(req, true); });
  server->on("/mission/gps", HTTP_GET, [startReq](AsyncWebServerRequest* req) { startReq(req, false); });
  server->on("/mission/route", HTTP_GET, [this](AsyncWebServerRequest* req) {
    float sp = 0.2f;
    if (req->hasParam("spd")) sp = req->getParam("spd")->value().toFloat();
    _reqLeg = req->hasParam("leg") ? (uint8_t)req->getParam("leg")->value().toInt() : 0;
    _reqSpeed = sp;
    _aliveMs = millis();
    _reqRoute = true;
    req->send(200, "application/json", "{\"ok\":true}");
  });
  server->on("/mission/resume", HTTP_GET, [this](AsyncWebServerRequest* req) {
    float sp = 0.2f;
    if (req->hasParam("spd")) sp = req->getParam("spd")->value().toFloat();
    _reqSpeed = sp;
    _aliveMs = millis();
    _reqResume = true;
    req->send(200, "application/json", "{\"ok\":true}");
  });
  server->on("/mission/legs", HTTP_GET, [](AsyncWebServerRequest* req) {
    String j = "[";
    for (uint8_t i = 0; i < RouteFollow::count(); i++) {
      if (i) j += ',';
      j += '"'; j += RouteFollow::leg(i).name; j += '"';
    }
    j += ']';
    req->send(200, "application/json", j);
  });
  server->on("/mission/stop", HTTP_GET, [this](AsyncWebServerRequest* req) {
    _reqStop = true;
    req->send(200, "application/json", "{\"ok\":true}");
  });
  return true;
}

void Mission::_finish(State st, const char* msg, WallFollow& wall, GpsNav& nav) {
  _state = st;
  _paused = false;
  wall.stop();
  nav.stop();
  route.pause(msg);  // route remembers leg + heading: "Resume" continues
  strlcpy(_msg, msg, sizeof(_msg));
  Serial.printf("[mission] %s: %s\n", kMissionState[st], msg);
}

void Mission::pollRequests(const Sensors& s, WallFollow& wall, GpsNav& nav) {
  if (_reqStop) { _reqStop = false; if (active()) _finish(ABORTED, "STOP pressed - Resume continues", wall, nav); }
  if (_reqRoute || _reqResume) {
    const bool resume = _reqResume && !_reqRoute;
    _reqRoute = _reqResume = false;
    _speed = constrain((float)_reqSpeed, minSpeed, maxSpeed);
    wall.stop();
    nav.stop();
    const bool ok = resume ? route.resume(_speed) : route.start(s, wall, _reqLeg, _speed);
    if (ok) {
      _state = ROUTE;
      _paused = false;
      _legStartMs = millis();
      strlcpy(_msg, route.msg(), sizeof(_msg));
      Serial.printf("[mission] route %s leg %u, speed %.2f\n", resume ? "resume" : "start",
                    (unsigned)(route.legIndex() + 1), _speed);
    } else {
      _state = ABORTED;
      strlcpy(_msg, resume ? "nothing to resume" : route.msg(), sizeof(_msg));
    }
  }
  if (_reqWall || _reqGps) {
    const bool withWall = _reqWall;
    _reqWall = _reqGps = false;
    _speed = constrain((float)_reqSpeed, minSpeed, maxSpeed);
    if (!s.gpsFix() || s.hdop() <= 0 || s.hdop() > nav.maxHdop) { _finish(ABORTED, "no good GPS - wait for a fix", wall, nav); return; }
    if (nav.count() == 0) { _finish(ABORTED, "no waypoints (load route B->H)", wall, nav); return; }
    nav.forgetHeading();  // learn it fresh on this run
    if (withWall) {
      if (!wall.start(s, wallSide)) { _finish(ABORTED, "no wall seen - place parallel to B", wall, nav); return; }
      _state = WALL_LEG;
      strlcpy(_msg, "following the wall", sizeof(_msg));
      // heading seed: rover direction = facade direction, corrected by the measured wall angle
      // (angle < 0 = nose towards the wall: left wall -> rover points left of the facade direction)
      if (nav.distToWp(s, 0) <= seedMaxDistM) {
        const float th = wall.lastWall().angleDeg;
        const float h0 = facadeBearingDeg + (wallSide == WallFollow::LEFT ? th : -th);
        nav.seedHeading(h0, s.yawDeg());
        snprintf(_msg, sizeof(_msg), "following the wall, heading seeded %.0f deg", h0);
      }
    } else {
      uint8_t i = nav.nearestIndex(s);
      nav.startFrom(i);
      _state = GPS_LEG;
      strlcpy(_msg, "aligning, then route", sizeof(_msg));
    }
    _legStartMs = millis();
    _paused = false;
    Serial.printf("[mission] start (%s), speed %.2f\n", withWall ? "wall+GPS" : "GPS", _speed);
  }
  // copies for the page
  _wp = nav.wpIndex();
  _wpN = nav.count();
  _headingOk = nav.headingKnown();
  strlcpy(_legInfo, (_state == WALL_LEG ? ("wall: " + wall.statusText()) : ("gps: " + nav.statusText())).c_str(),
          sizeof(_legInfo));
  _rLeg = route.canResume() || route.state() == RouteFollow::DONE ? route.legIndex() + 1 : 0;
  _rState = route.state();
  _rEffPct = (uint8_t)constrain(lroundf(route.effortFrac() * 100.0f), 0L, 255L);
  _rCanResume = route.canResume() && route.state() != RouteFollow::DONE;
  if (_state == ROUTE) strlcpy(_msg, route.msg(), sizeof(_msg));
}

void Mission::steer(const Sensors& s, WallFollow& wall, GpsNav& nav, float padL, float padR, uint32_t padAge,
                    float& l, float& r) {
  l = r = 0;
  // any drive key = manual override: abort the mission
  if (padAge <= 400 && (padL != 0 || padR != 0)) {
    _finish(ABORTED, "drive key - manual (Resume continues)", wall, nav);
    l = padL;
    r = padR;
    return;
  }
  // no heartbeat from the page -> pause (stop), continue when it is back
  _paused = millis() - _aliveMs > heartbeatMs;
  if (_paused) { nav.observe(s); return; }

  if (_state == ROUTE) {
    route.steer(s, wall, l, r);
    if (route.state() == RouteFollow::DONE) {
      l = r = 0;
      _state = DONE;
      strlcpy(_msg, route.msg(), sizeof(_msg));
      Serial.printf("[mission] %s\n", _msg);
    } else if (route.state() == RouteFollow::PAUSED) {
      l = r = 0;
      _state = ABORTED;  // manual control possible; "Resume" continues the leg
      strlcpy(_msg, route.msg(), sizeof(_msg));
      Serial.printf("[mission] route paused: %s\n", _msg);
    }
    return;
  }

  if (_state == WALL_LEG) {
    nav.observe(s);  // learn the GPS heading while driving along the wall
    wall.steer(s, _speed, _speed, 0, l, r);
    if (wall.state() == WallFollow::LOST) {
      l = r = 0;
      if (!nav.headingKnown()) { _finish(ABORTED, "wall ended, heading not learned - stopped", wall, nav); return; }
      wall.stop();
      uint8_t i = nav.nearestIndex(s);
      if (i == 0 && nav.count() > 1) i = 1;  // WP00 is the start
      nav.startFrom(i);
      _state = GPS_LEG;
      _legStartMs = millis();
      snprintf(_msg, sizeof(_msg), "wall ended -> GPS from WP%02u", (unsigned)i);
      Serial.printf("[mission] %s\n", _msg);
    } else if (wall.state() == WallFollow::OFF) {
      _finish(ABORTED, "wall following stopped", wall, nav);
    }
    return;
  }
  if (_state == GPS_LEG) {
    nav.steer(s, _speed, _speed, 0, l, r);  // speed with "UP held" = mission permission
    if (nav.state() == GpsNav::DONE) { l = r = 0; _finish(DONE, "goal reached", wall, nav); }
    else if (nav.state() == GpsNav::OFF) { l = r = 0; _finish(ABORTED, "GPS leg failed (align?)", wall, nav); }
  }
}

String Mission::statusText() const {
  String t = kMissionState[_state];
  if (_paused) t += " PAUSED";
  return t;
}

String Mission::_statusJson() const {
  String j;
  j.reserve(320);
  j += "{\"running\":"; j += active() ? "true" : "false";
  j += ",\"state\":\""; j += kMissionState[_state];
  j += "\",\"paused\":"; j += _paused ? "true" : "false";
  j += ",\"leg\":\""; j += _legInfo;
  j += "\",\"wp\":"; j += (unsigned)_wp;
  j += ",\"n\":"; j += (unsigned)_wpN;
  j += ",\"hdg\":"; j += _headingOk ? "true" : "false";
  j += ",\"spd\":"; j += String(_speed, 2);
  j += ",\"rleg\":"; j += (unsigned)_rLeg;
  j += ",\"rn\":"; j += (unsigned)RouteFollow::count();
  j += ",\"rstate\":\""; j += kRouteState[_rState < 7 ? _rState : 0];
  j += "\",\"reff\":"; j += (unsigned)_rEffPct;
  j += ",\"canResume\":"; j += _rCanResume ? "true" : "false";
  j += ",\"msg\":\""; j += _msg; j += "\"}";
  return j;
}
