#pragma once
// PageUi - small changes to the starter web page without editing WebHMI.cpp (2026-10-23):
//   - one "Sensors" table instead of the ultrasonic table + the IMU/GPS card:
//       row 1: ultrasonic in cm, columns L | R | F | B | FA   ("-" = nothing within 3 m)
//       row 2: IMU roll / pitch / yaw (+ calibration), row 3: GPS
//   - /boot.txt: why the box (re)started, one line per boot (power on, BROWNOUT, crash, ...)
// It replaces the page's renderSensors() function; loaded via /log.js (DriveLog).
//
//   setup(): registerPageUi(hmi.server());
#include <Arduino.h>
#include <LittleFS.h>
#include <ESPAsyncWebServer.h>

static const char kUiJs[] PROGMEM = R"rawjs((function(){
const $=id=>document.getElementById(id);
const ut=$('usTable'); if(ut) ut.style.display='none';
const nt=$('navTable'); if(nt){const c=nt.closest('.card'); if(c) c.style.display='none';}
const card=document.createElement('div'); card.className='card';
card.innerHTML="<div class='sectionTitle'>Sensors</div><table class='table' id='snTable'></table>";
const sur=$('envCv')?$('envCv').closest('.card'):null;
if(sur) sur.parentNode.insertBefore(card, sur.nextSibling); else document.querySelector('.wrap').appendChild(card);
const cm=(v,m)=>(v===undefined||v>=m)?'-':v;
window.renderSensors=function(s){
  if(!s)return;
  drawEnv(s);
  const L=s.lidar||{};
  $('lidarInfo').textContent=L.ok?('LiDAR '+L.pps+' packets/s, checksum errors '+L.crc+' %'):'LiDAR: no data';
  const u=s.us||[],m=s.usMax||300,I=s.imu||{},G=s.gps||{};
  let t="<tr><td class='k'>Ultrasonic cm</td>"+['L','R','F','B','FA'].map(n=>"<td class='k'>"+n+"</td>").join('')+"</tr>";
  t+="<tr><td></td>"+[1,2,0,3,4].map(i=>"<td><b>"+cm(u[i],m)+"</b></td>").join('')+"</tr>";
  t+=I.ok?("<tr><td class='k'>IMU roll / pitch / yaw</td><td colspan='5'>"+I.roll+"&deg; / "+I.pitch+"&deg; / "+I.yaw
     +"&deg; (yaw relative)<br><span class='k'>cal sys/gyr/acc/mag "+(I.cal||[]).join('/')+"</span></td></tr>")
     :"<tr><td class='k'>IMU</td><td colspan='5'>not found</td></tr>";
  t+=G.chars?("<tr><td class='k'>GPS</td><td colspan='5'>"+(G.fix?'fix':'no fix')+", "+G.sats+" sat, HDOP "+G.hdop
     +(G.fix?("<br>"+G.lat+", "+G.lon):"")+"</td></tr>")
     :"<tr><td class='k'>GPS</td><td colspan='5'>no data from the module</td></tr>";
  $('snTable').innerHTML=t;
};
})();)rawjs";

inline void registerPageUi(AsyncWebServer* server) {
  if (!server) return;
  server->on("/ui.js", HTTP_GET, [](AsyncWebServerRequest* req) {
    req->send(200, "application/javascript", kUiJs);
  });
  server->on("/boot.txt", HTTP_GET, [](AsyncWebServerRequest* req) {
    if (LittleFS.exists("/boot.txt")) req->send(LittleFS, "/boot.txt", "text/plain");
    else req->send(200, "text/plain", "no boot log yet");
  });
}

// Why did the box start? Appends one line to /boot.txt (LittleFS must be mounted) and returns
// a short text. A BROWNOUT means the supply voltage dropped - e.g. motors at full power.
inline String logBootReason() {
  const esp_reset_reason_t rr = esp_reset_reason();
  const char* why;
  switch (rr) {
    case ESP_RST_POWERON:  why = "power on"; break;
    case ESP_RST_BROWNOUT: why = "BROWNOUT (supply voltage dropped)"; break;
    case ESP_RST_PANIC:    why = "CRASH (panic)"; break;
    case ESP_RST_INT_WDT:
    case ESP_RST_TASK_WDT:
    case ESP_RST_WDT:      why = "WATCHDOG (program hung)"; break;
    case ESP_RST_SW:       why = "software restart"; break;
    case ESP_RST_EXT:      why = "reset button / upload"; break;
    default:               why = "other"; break;
  }
  int boots = 0;
  if (LittleFS.exists("/boot.txt")) {
    fs::File f = LittleFS.open("/boot.txt", "r");
    while (f.available()) if (f.read() == '\n') boots++;
    f.close();
  }
  if (boots < 500) {  // keep the file small
    fs::File f = LittleFS.open("/boot.txt", "a");
    if (f) { f.printf("boot %d: %s\n", boots + 1, why); f.close(); }
  }
  Serial.printf("[sys] boot %d, reset reason: %s\n", boots + 1, why);
  return String(why);
}
