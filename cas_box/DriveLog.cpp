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
 +"<button class='btnTiny' id='lgClear'>Delete all</button></div>"
 +"<div class='label' style='margin-top:10px'>Mark at the END of a segment: how it was driven / what ends it / where</div>"
 +"<div style='display:flex;gap:6px;flex-wrap:wrap;margin-top:4px'>"
 +"<select id='lgDrv'><option value='1'>along LEFT wall</option><option value='2'>along RIGHT wall</option>"
 +"<option value='3'>keep heading</option><option value='0'>-</option></select>"
 +"<select id='lgFt'><option value='14'>start of route</option><option value='1'>wall ends</option>"
 +"<option value='2'>wall starts</option><option value='3'>wall / object ahead</option>"
 +"<option value='4'>ramp starts</option><option value='5'>ramp ends</option>"
 +"<option value='6'>static object beside</option><option value='7'>corner / turn</option>"
 +"<option value='8'>open area</option><option value='0'>note only</option></select>"
 +"<select id='lgSd'><option value='1'>left</option><option value='2'>right</option>"
 +"<option value='3'>ahead</option><option value='0'>-</option></select></div>"
 +"<div style='display:flex;gap:8px;margin-top:6px'>"
 +"<button class='btnTiny' id='lgMark' style='min-width:110px'>Mark</button>"
 +"<button class='btnTiny' id='lgUndo'>Undo last mark</button></div>"
 +"<div class='label' id='lgMarks' style='white-space:pre-line'></div>"
 +"<div class='value' id='lgInfo' style='margin-top:10px'>-</div>"
 +"<div class='label' id='lgMem'></div><table class='table' id='lgFiles'></table>";
const d=document.getElementById('driveCard'),w=document.querySelector('.wrap');
w.insertBefore(c,d?d.nextSibling:w.children[2]);
const $=id=>document.getElementById(id);
const DRV=['','L wall','R wall','heading'],FT=['note','wall ends','wall starts','wall/object','ramp starts',
 'ramp ends','object beside','corner','open area','','','','','','START','UNDO'],SD=['','left','right','ahead'];
const tagText=t=>(t&15)==15?'UNDO previous mark':(t&15)==14?'start of route':
 (DRV[t>>6]?DRV[t>>6]+' | ':'')+FT[t&15]+(SD[(t>>4)&3]?' '+SD[(t>>4)&3]:'');
let lastMn=-1,list=[],lastFile='';
async function st(){try{
  const j=await (await fetch('/log/status',{cache:'no-store'})).json();
  if(!j.fs){$('lgInfo').textContent='No flash file system';return;}
  $('lgInfo').textContent=j.err?('Error: '+j.err):(j.active
    ?('● Recording '+j.file+' – '+j.recs+' records ('+Math.round(j.recs/4)+' s), marks: '+j.marks)
    :'Stopped');
  $('lgInfo').style.color=j.active?'#ff5a5f':'';
  $('lgMark').textContent='Mark ('+j.marks+')';
  if(j.file!==lastFile){lastFile=j.file;list=[];lastMn=-1;$('lgMarks').textContent='';}
  if(j.active&&j.mn!==lastMn&&j.mn>0){lastMn=j.mn;list.push('M'+j.mn+'  '+tagText(j.tag)+'  -  '+j.mi);
   if(list.length>6)list.shift();$('lgMarks').textContent=list.join('\n');}
  $('lgMem').textContent='Flash: '+j.usedKB+' / '+j.totalKB+' kB used';
  $('lgFiles').innerHTML=(j.files||[]).map(f=>"<tr><td class='k'>"+f[0]+"</td><td>"+f[1]
    +" kB</td><td><a style='color:#4ea1ff' href='/log/file?name="+f[0]+"' download='"+f[0]+"'>download</a></td></tr>").join('');
}catch(e){}}
const cmd=p=>fetch('/log/'+p,{cache:'no-store'}).then(st).catch(()=>{});
$('lgStart').onclick=()=>cmd('start');
$('lgStop').onclick=()=>cmd('stop');
$('lgMark').onclick=()=>{const f=+$('lgFt').value;
 const t=f==14?14:(f|(+$('lgSd').value<<4)|(+$('lgDrv').value<<6));cmd('mark?t='+t);
 if(f==14)$('lgFt').value='1';};
$('lgUndo').onclick=()=>cmd('mark?t=15');
$('lgClear').onclick=()=>{if(confirm('Delete all log files?'))cmd('clear');};
setInterval(st,1000);st();
const us=document.createElement('script');us.src='/ui.js';document.body.appendChild(us);  // combined sensor table
const ms=document.createElement('script');ms.src='/mission.js';document.body.appendChild(ms);  // Mission card
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
    _refreshList();
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
    // only the web task writes _markWord while recording; one 16-bit store = count + tag together
    const uint8_t tag = req->hasParam("t") ? (uint8_t)constrain(req->getParam("t")->value().toInt(), 0L, 255L) : 0;
    if (_active && _marks() < 255) _markWord = (uint16_t)(((_marks() + 1) << 8) | tag);
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
  if (_marks() != _seenMarks) _measureMark(s);

  _dbgAcc |= d.dbg;               // a 60 ms tap between two records must still show up
  const uint32_t now = millis();
  if (now - _lastRecMs < kPeriodMs) return;
  _lastRecMs += kPeriodMs;
  if (now - _lastRecMs > kPeriodMs) _lastRecMs = now;  // after a long pass: no burst of records

  Sample dd = d;
  dd.dbg = _dbgAcc;
  _dbgAcc = 0;
  _fill(_buf[_bufCount++], s, dd);
  _records = _records + 1;
  if (_bufCount >= kBufferRecords) _flush();
}

String DriveLog::statusText() const {
  if (!_fsOk) return "no flash FS";
  if (_error[0]) return String("error: ") + _error;
  if (_active) {
    return String("rec ") + _fileName + " " + String((uint32_t)_records) + " rec, " +
           String((uint32_t)_marks()) + " marks";
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
  h.version = 3;
  h.lidarPoints = Sensors::kLidarPoints;
  h.recordSize = sizeof(LogRecord);
  h.periodMs = kPeriodMs;
  h.startMs = millis();
  _file.write((const uint8_t*)&h, sizeof(h));

  _error[0] = 0;
  _records = 0;
  _markWord = 0;
  _seenMarks = 0;
  _markInfo[0] = 0;
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
  _refreshList();
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
  _refreshList();
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
  _refreshList();
  Serial.printf("[log] deleted %u files\n", (unsigned)names.size());
}

// Closest LiDAR return (m) within +-12 deg of direction idx (0 = front, 22 = right, 67 = left), 9 = none.
static float sectorM(const Sensors& s, int idx) {
  uint16_t best = 0;
  for (int d = -3; d <= 3; d++) {
    const uint16_t v = s.lidarCm((uint8_t)((idx + d + Sensors::kLidarPoints) % Sensors::kLidarPoints));
    if (v > 20 && (best == 0 || v < best)) best = v;
  }
  return best ? best / 100.0f : 9.0f;
}

// At every new mark: what the LiDAR / IMU see right now -> shown on the page next to the tag, so a
// wrong tag (e.g. "wall ends left" but the LiDAR still sees a wall at 0.6 m) is noticed on the spot.
void DriveLog::_measureMark(const Sensors& s) {
  const uint16_t mw = _markWord;
  char t[sizeof(_markInfo)];
  const float l = sectorM(s, 67), r = sectorM(s, 22), f = sectorM(s, 0);
  char ls[8], rs[8], fs[8];
  if (l < 9) snprintf(ls, sizeof(ls), "%.2f m", l); else strlcpy(ls, "-", sizeof(ls));
  if (r < 9) snprintf(rs, sizeof(rs), "%.2f m", r); else strlcpy(rs, "-", sizeof(rs));
  if (f < 9) snprintf(fs, sizeof(fs), "%.2f m", f); else strlcpy(fs, "-", sizeof(fs));
  snprintf(t, sizeof(t), "LiDAR left %s, right %s, ahead %s, pitch %+.1f deg", ls, rs, fs, s.pitchDeg());
  portENTER_CRITICAL(&_listMux);
  strlcpy(_markInfo, t, sizeof(_markInfo));
  _seenMarks = (uint8_t)(mw >> 8);
  _seenTag = (uint8_t)mw;
  portEXIT_CRITICAL(&_listMux);
  Serial.printf("[log] mark %u tag 0x%02X: %s\n", (unsigned)(mw >> 8), (unsigned)(mw & 0xFF), t);
}

void DriveLog::_fill(LogRecord& r, const Sensors& s, const Sample& d) const {
  r.ms = millis();
  // v3: route debug in the (unused) GPS fields - see Sample
  auto i16 = [](float x) { return (uint16_t)(int16_t)constrain(lroundf(x), -32767L, 32767L); };
  r.latE7 = (int32_t)(((uint32_t)i16(d.lpX * 100.0f) << 16) | i16(d.lpY * 100.0f));
  r.lonE7 = (int32_t)(((uint32_t)i16(d.lpThDeg * 10.0f) << 16) | i16(d.carrotDeg * 10.0f));
  r.hdopX100 = i16(d.ctrlDeg * 10.0f);
  r.sats = (uint8_t)constrain(lroundf(d.lpScore * 100.0f), 0L, 255L);
  r.flags = (s.imuOk() ? 2 : 0) | (s.lidarOk() ? 4 : 0) | (uint8_t)((d.dbg & 31) << 3);
  r.yawX100 = (int16_t)lroundf(s.yawDeg() * 100.0f);
  r.pitchX100 = (int16_t)lroundf(s.pitchDeg() * 100.0f);
  r.rollX100 = (int16_t)lroundf(s.rollDeg() * 100.0f);
  r.magCal = d.navByte ? d.navByte : s.magCal();
  const uint16_t mw = _markWord;  // one load: count and tag of the same press
  r.mark = (uint8_t)(mw >> 8);
  r.markTag = (uint8_t)mw;
  r.reserved = d.navLeg;
  r.cmdL = toI8(d.cmdL);
  r.cmdR = toI8(d.cmdR);
  r.outL = toI8(d.outL);
  r.outR = toI8(d.outR);
  r.safety = d.safety;
  r.rttX10ms = (uint8_t)constrain(lroundf(d.rttMs / 10.0f), 0L, 255L);
  r.batteryX100 = toU16(lroundf(d.batteryV * 100.0f));
  for (uint8_t i = 0; i < 5; i++) r.usCm[i] = toU16(s.usCm(i));
  for (uint8_t i = 0; i < Sensors::kLidarPoints; i++) r.lidarCm[i] = s.lidarCm(i);
}

// Runs in the web task. The file list is read only while no log is being written.
// Builds the file list once in loop() (after begin / stop / clear). The web task only copies the
// text: reading the flash directory inside the web server task every second could block it long
// enough for the watchdog to restart the box (UI dropping out indoors, 2026-10-23).
void DriveLog::_refreshList() {
  String j;
  bool first = true;
  fs::File root = LittleFS.open("/");
  for (fs::File f = root.openNextFile(); f; f = root.openNextFile()) {
    const String n = f.name();
    if (!_validName(n)) continue;
    if (j.length() > sizeof(_filesJson) - 40) break;  // keep it short (newest files may be cut)
    if (!first) j += ',';
    first = false;
    j += "[\""; j += n; j += "\","; j += (uint32_t)((f.size() + 1023) / 1024); j += ']';
  }
  root.close();
  portENTER_CRITICAL(&_listMux);
  strlcpy(_filesJson, j.c_str(), sizeof(_filesJson));
  portEXIT_CRITICAL(&_listMux);
}

// Runs in the web task: no flash access here.
String DriveLog::_statusJson() const {
  char files[sizeof(_filesJson)];
  portENTER_CRITICAL(&_listMux);
  memcpy(files, _filesJson, sizeof(files));
  portEXIT_CRITICAL(&_listMux);
  String j;
  j.reserve(400);
  j += "{\"fs\":"; j += _fsOk ? "true" : "false";
  j += ",\"active\":"; j += _active ? "true" : "false";
  j += ",\"file\":\""; j += _fileName;
  j += "\",\"recs\":"; j += (uint32_t)_records;
  const uint16_t mw = _markWord;
  char mi[sizeof(_markInfo)];
  portENTER_CRITICAL(&_listMux);
  memcpy(mi, _markInfo, sizeof(mi));
  const uint8_t mn = _seenMarks, mt = _seenTag;
  portEXIT_CRITICAL(&_listMux);
  j += ",\"marks\":"; j += (uint32_t)(mw >> 8);
  j += ",\"tag\":"; j += (uint32_t)mt;   // tag of mark mn (the one measured)
  j += ",\"mn\":"; j += (uint32_t)mn;
  j += ",\"mi\":\""; j += mi; j += "\"";
  j += ",\"usedKB\":"; j += (uint32_t)_usedKB;
  j += ",\"totalKB\":"; j += (uint32_t)_totalKB;
  j += ",\"err\":\""; j += _error;
  j += "\",\"files\":["; j += files; j += "]}";
  return j;
}

bool DriveLog::_validName(const String& n) {  // log_NNN.bin
  if (n.length() != 11 || !n.startsWith("log_") || !n.endsWith(".bin")) return false;
  for (int i = 4; i < 7; i++) if (!isDigit(n[i])) return false;
  return true;
}
