#include "DriveLog.h"
#include <LittleFS.h>
#include <vector>

// ---------------------------------------------------------------------------------------------
// /log.js - adds the "Drive log" card to the main page (below the drive pad).
// The main page loads it with one extra line, see INTEGRATION.md.
// ---------------------------------------------------------------------------------------------
static const char kLogJs[] PROGMEM = R"rawjs((function(){
const c=document.createElement('div');c.className='card';
c.innerHTML="<div class='sectionTitle'>Drive log</div>"
 +"<div style='display:flex;gap:8px;flex-wrap:wrap'>"
 +"<button class='btnTiny' id='lgStart'>&#9679; Start</button>"
 +"<button class='btnTiny' id='lgStop'>&#9632; Stop</button>"
 +"<button class='btnTiny' id='lgMark' style='min-width:90px'>Mark</button>"
 +"<button class='btnTiny' id='lgClear'>Delete all</button></div>"
 +"<div class='value' id='lgInfo' style='margin-top:10px'>-</div>"
 +"<div class='label' id='lgMem'></div><table class='table' id='lgFiles'></table>";
const d=document.getElementById('driveCard'),w=document.querySelector('.wrap');
w.insertBefore(c,d?d.nextSibling:w.children[2]);
const $=id=>document.getElementById(id);
async function st(){try{
  const j=await (await fetch('/log/status',{cache:'no-store'})).json();
  if(!j.fs){$('lgInfo').textContent='No flash file system';return;}
  $('lgInfo').textContent=j.err?('Error: '+j.err):(j.active
    ?('● Recording '+j.file+' – '+j.recs+' records ('+Math.round(j.recs/4)+' s), marks: '+j.marks)
    :'Stopped');
  $('lgInfo').style.color=j.active?'#ff5a5f':'';
  $('lgMark').textContent='Mark ('+j.marks+')';
  $('lgMem').textContent='Flash: '+j.usedKB+' / '+j.totalKB+' kB used';
  $('lgFiles').innerHTML=(j.files||[]).map(f=>"<tr><td class='k'>"+f[0]+"</td><td>"+f[1]
    +" kB</td><td><a style='color:#4ea1ff' href='/log/file?name="+f[0]+"' download='"+f[0]+"'>download</a></td></tr>").join('');
}catch(e){}}
const cmd=p=>fetch('/log/'+p,{cache:'no-store'}).then(st).catch(()=>{});
$('lgStart').onclick=()=>cmd('start');
$('lgStop').onclick=()=>cmd('stop');
$('lgMark').onclick=()=>cmd('mark');
$('lgClear').onclick=()=>{if(confirm('Delete all log files?'))cmd('clear');};
setInterval(st,1000);st();
})();)rawjs";

static int8_t toI8(float v) {  // -0.5..0.5 -> -100..100
  const long x = lroundf(v * 200.0f);
  return (int8_t)(x < -127 ? -127 : (x > 127 ? 127 : x));
}
static uint16_t toU16(long v) { return (uint16_t)(v < 0 ? 0 : (v > 65535 ? 65535 : v)); }

// ---------------------------------------------------------------------------------------------

bool DriveLog::begin(AsyncWebServer* server) {
  _fsOk = LittleFS.begin(true);  // formats the "spiffs" partition on first use
  if (_fsOk) {
    _usedKB = LittleFS.usedBytes() / 1024;
    _totalKB = LittleFS.totalBytes() / 1024;
    Serial.printf("[log] LittleFS %u / %u kB used\n", (unsigned)_usedKB, (unsigned)_totalKB);
  } else {
    Serial.println("[log] LittleFS mount failed - logging disabled (check partition scheme)");
  }
  if (!server) return _fsOk;

  server->on("/log.js", HTTP_GET, [](AsyncWebServerRequest* req) {
    req->send(200, "application/javascript", kLogJs);  // PROGMEM is plain flash on the ESP32
  });
  server->on("/log/status", HTTP_GET, [this](AsyncWebServerRequest* req) {
    req->send(200, "application/json", _statusJson());
  });
  server->on("/log/start", HTTP_GET, [this](AsyncWebServerRequest* req) {
    _reqStart = true;
    req->send(200, "application/json", "{\"ok\":true}");
  });
  server->on("/log/stop", HTTP_GET, [this](AsyncWebServerRequest* req) {
    _reqStop = true;
    req->send(200, "application/json", "{\"ok\":true}");
  });
  server->on("/log/mark", HTTP_GET, [this](AsyncWebServerRequest* req) {
    if (_active) _marks = _marks + 1;  // only the web task writes _marks while recording
    req->send(200, "application/json", "{\"ok\":true}");
  });
  server->on("/log/clear", HTTP_GET, [this](AsyncWebServerRequest* req) {
    _reqClear = true;
    req->send(200, "application/json", "{\"ok\":true}");
  });
  server->on("/log/file", HTTP_GET, [this](AsyncWebServerRequest* req) {
    if (!_fsOk || !req->hasParam("name")) { req->send(400, "text/plain", "name missing"); return; }
    const String name = req->getParam("name")->value();
    if (!_validName(name)) { req->send(400, "text/plain", "bad name"); return; }
    if (_active && name == _fileName) { req->send(409, "text/plain", "stop the log first"); return; }
    if (!LittleFS.exists("/" + name)) { req->send(404, "text/plain", "not found"); return; }
    req->send(LittleFS, "/" + name, "application/octet-stream", true);
  });
  return _fsOk;
}

void DriveLog::update(const Sensors& s, const Sample& d) {
  if (!_fsOk) return;
  if (_reqClear) { _reqClear = false; if (!_active) _clear(); }
  if (_reqStart) { _reqStart = false; if (!_active) _start(); }
  if (_reqStop)  { _reqStop = false;  if (_active) _stop(); }
  if (!_active) return;

  const uint32_t now = millis();
  if (now - _lastRecMs < kPeriodMs) return;
  _lastRecMs += kPeriodMs;
  if (now - _lastRecMs > kPeriodMs) _lastRecMs = now;  // after a long pass: no burst of records

  _fill(_buf[_bufCount++], s, d);
  _records = _records + 1;
  if (_bufCount >= kBufferRecords) _flush();
}

String DriveLog::statusText() const {
  if (!_fsOk) return "no flash FS";
  if (_error[0]) return String("error: ") + _error;
  if (_active) {
    return String("rec ") + _fileName + " " + String((uint32_t)_records) + " rec, " +
           String((uint32_t)_marks) + " marks";
  }
  return String("stopped, ") + String((uint32_t)_usedKB) + "/" + String((uint32_t)_totalKB) + " kB";
}

// ---------------------------------------------------------------------------------------------

void DriveLog::_start() {
  // next free number: one higher than the highest existing log_NNN.bin
  int next = 1;
  fs::File root = LittleFS.open("/");
  for (fs::File f = root.openNextFile(); f; f = root.openNextFile()) {
    const String n = f.name();
    if (_validName(n)) next = max(next, (int)n.substring(4, 7).toInt() + 1);
  }
  root.close();
  if (next > 999) { strlcpy(_error, "999 files - delete some", sizeof(_error)); return; }

  snprintf(_fileName, sizeof(_fileName), "log_%03d.bin", next);
  _file = LittleFS.open(String("/") + _fileName, "w");
  if (!_file) { strlcpy(_error, "cannot create file", sizeof(_error)); return; }

  LogHeader h;
  memcpy(h.magic, "CASLOG", 6);
  h.version = 1;
  h.lidarPoints = Sensors::kLidarPoints;
  h.recordSize = sizeof(LogRecord);
  h.periodMs = kPeriodMs;
  h.startMs = millis();
  _file.write((const uint8_t*)&h, sizeof(h));

  _error[0] = 0;
  _records = 0;
  _marks = 0;
  _bufCount = 0;
  _lastRecMs = millis() - kPeriodMs;  // first record right away
  _active = true;
  Serial.printf("[log] start %s\n", _fileName);
}

void DriveLog::_stop() {
  _flush();
  if (_file) _file.close();
  _active = false;
  _usedKB = LittleFS.usedBytes() / 1024;
  Serial.printf("[log] stop %s, %u records\n", _fileName, (unsigned)_records);
}

// Writes the RAM buffer to flash. Stops the log on a write error or when the flash is nearly full.
void DriveLog::_flush() {
  if (!_bufCount || !_file) { _bufCount = 0; return; }
  const size_t want = _bufCount * sizeof(LogRecord);
  const size_t got = _file.write((const uint8_t*)_buf, want);
  _bufCount = 0;
  _file.flush();  // commit: a power cut loses at most the last 4 s
  _usedKB = LittleFS.usedBytes() / 1024;
  if (got != want) {
    strlcpy(_error, "write failed (flash full?)", sizeof(_error));
  } else if (LittleFS.totalBytes() - LittleFS.usedBytes() < kMinFreeBytes) {
    strlcpy(_error, "flash full - log stopped", sizeof(_error));
  } else {
    return;
  }
  _file.close();
  _active = false;
  Serial.printf("[log] %s\n", _error);
}

void DriveLog::_clear() {
  std::vector<String> names;  // collect first: deleting while iterating confuses the directory walk
  fs::File root = LittleFS.open("/");
  for (fs::File f = root.openNextFile(); f; f = root.openNextFile()) {
    const String n = f.name();
    if (_validName(n)) names.push_back(n);
  }
  root.close();
  for (const String& n : names) LittleFS.remove("/" + n);
  _error[0] = 0;
  _fileName[0] = 0;
  _usedKB = LittleFS.usedBytes() / 1024;
  Serial.printf("[log] deleted %u files\n", (unsigned)names.size());
}

void DriveLog::_fill(LogRecord& r, const Sensors& s, const Sample& d) const {
  r.ms = millis();
  r.latE7 = (int32_t)llround(s.lat() * 1e7);
  r.lonE7 = (int32_t)llround(s.lon() * 1e7);
  r.hdopX100 = toU16(lroundf(s.hdop() * 100.0f));
  r.sats = (uint8_t)constrain(s.sats(), 0, 255);
  r.flags = (s.gpsFix() ? 1 : 0) | (s.imuOk() ? 2 : 0) | (s.lidarOk() ? 4 : 0);
  r.yawX100 = (int16_t)lroundf(s.yawDeg() * 100.0f);
  r.pitchX100 = (int16_t)lroundf(s.pitchDeg() * 100.0f);
  r.rollX100 = (int16_t)lroundf(s.rollDeg() * 100.0f);
  r.magCal = s.magCal();
  r.mark = _marks;
  r.cmdL = toI8(d.cmdL);
  r.cmdR = toI8(d.cmdR);
  r.outL = toI8(d.outL);
  r.outR = toI8(d.outR);
  r.safety = d.safety;
  r.reserved = 0;
  r.batteryX100 = toU16(lroundf(d.batteryV * 100.0f));
  for (uint8_t i = 0; i < 5; i++) r.usCm[i] = toU16(s.usCm(i));
  for (uint8_t i = 0; i < Sensors::kLidarPoints; i++) r.lidarCm[i] = s.lidarCm(i);
}

// Runs in the web task. The file list is read only while no log is being written.
String DriveLog::_statusJson() const {
  String j;
  j.reserve(400);
  j += "{\"fs\":"; j += _fsOk ? "true" : "false";
  j += ",\"active\":"; j += _active ? "true" : "false";
  j += ",\"file\":\""; j += _fileName;
  j += "\",\"recs\":"; j += (uint32_t)_records;
  j += ",\"marks\":"; j += (uint32_t)_marks;
  j += ",\"usedKB\":"; j += (uint32_t)_usedKB;
  j += ",\"totalKB\":"; j += (uint32_t)_totalKB;
  j += ",\"err\":\""; j += _error;
  j += "\",\"files\":[";
  if (_fsOk && !_active) {
    bool first = true;
    fs::File root = LittleFS.open("/");
    for (fs::File f = root.openNextFile(); f; f = root.openNextFile()) {
      const String n = f.name();
      if (!_validName(n)) continue;
      if (!first) j += ',';
      first = false;
      j += "[\""; j += n; j += "\","; j += (uint32_t)((f.size() + 1023) / 1024); j += ']';
    }
    root.close();
  }
  j += "]}";
  return j;
}

bool DriveLog::_validName(const String& n) {  // log_NNN.bin
  if (n.length() != 11 || !n.startsWith("log_") || !n.endsWith(".bin")) return false;
  for (int i = 4; i < 7; i++) if (!isDigit(n[i])) return false;
  return true;
}
