#pragma once
// MissionRadio - phone OFF the Wi-Fi while the mission runs, back ON after a double tap (v3, 2026-11-10).
//
// WHY NOT SWITCH THE WI-FI RADIO OFF: the box talks to the rover over the SAME Wi-Fi (the rover is a
// station on the box access point, RoverLink sends every drive command by HTTP). Radio off = rover
// stops after 0.5 s (RoverLink deadman) / 3 s (rover heartbeat). So "Wi-Fi off" here means:
//   QUIET   mission running -> every station on the access point that is NOT the rover is disconnected
//           (esp_wifi_deauth_sta) every 0.3 s. The phone cannot stay connected, the web server gets no
//           requests, the mission does not depend on the page heartbeat (Mission::needHeartbeat=false),
//           and nothing from the phone can pause / jerk the rover. The rover link is not touched.
//           If the rover is not known yet (searching), nobody is kicked (it could be the rover).
//   WAKE    a DOUBLE TAP on the box (ImuManager) ends QUIET until the next mission start: the phone can
//           join again (Wi-Fi list, CAS-Box-NN, page reload). At the same moment the IMU ring buffer
//           (2.56 s: 2 s before + 0.5 s after the tap, linear acceleration + yaw at 100 Hz) is written
//           to the flash as /tap_NNN.csv with the GPS clock (UTC) if the GPS module has a time, else
//           only the box time. Download: http://192.168.<10+N>.1/tap.csv (the newest).
//   The mission keeps running after a double tap (it is only the Wi-Fi). STOP on the page, or a drive
//   key, stops it as before.
//
//   setup(): radio.begin(hmi.server());
//   loop():  radio.update(sensors, mission.active(), rover.ip());   (after sensors.update())
#include <Arduino.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_netif.h>
#include <LittleFS.h>
#include <ESPAsyncWebServer.h>
#include "Sensors.h"

class MissionRadio {
public:
  bool quietOnMission = true;        // false = old behaviour (phone stays connected)
  uint32_t postTapMs = 500;          // record this long after the tap, then write the file

  // 11.11.: switched on the page (/radio?quiet=1|0), default from the sketch; onChange tells the sketch
  void (*onChange)(bool quiet) = nullptr;
  void begin(AsyncWebServer* server) {
    if (!server) return;
    server->on("/radio", HTTP_GET, [this](AsyncWebServerRequest* req) {
      if (req->hasParam("quiet")) {
        quietOnMission = req->getParam("quiet")->value().toInt() != 0;
        if (onChange) onChange(quietOnMission);
        Serial.printf("[radio] phone off during the mission: %s\n", quietOnMission ? "ON (demo)" : "OFF (test)");
      }
      req->send(200, "text/plain", quietOnMission ? "quiet" : "open");
    });
    server->on("/radio.json", HTTP_GET, [this](AsyncWebServerRequest* req) {
      req->send(200, "application/json", String("{\"quiet\":") + (quietOnMission ? "true" : "false") + "}");
    });
    server->on("/tap.csv", HTTP_GET, [this](AsyncWebServerRequest* req) {
      if (_lastFile[0] && LittleFS.exists(_lastFile)) req->send(LittleFS, _lastFile, "text/csv", true);
      else req->send(404, "text/plain", "no tap recording yet");
    });
  }

  bool quiet() const { return _quiet; }
  bool woken() const { return _wake; }
  uint16_t kicked() const { return _kicked; }
  const char* lastFile() const { return _lastFile; }
  String statusText() const {
    String s = _quiet ? "QUIET (phone off, rover only)" : (_wake ? "open (double tap)" : "open");
    s += ", taps " + String(_tapsSeen) + ", kicked " + String(_kicked);
    if (_lastFile[0]) s += ", last " + String(_lastFile);
    return s;
  }

  void update(Sensors& s, bool missionActive, IPAddress roverIp) {
    const uint32_t now = millis();
    if (missionActive && !_wasActive) _wake = false;   // a new mission start: quiet again
    _wasActive = missionActive;

    // ---- double tap: open the Wi-Fi, record
    const uint16_t dt = s.imu().doubleTaps();
    if (dt != _tapsSeen) {
      _tapsSeen = dt;
      _wake = true;
      _pendingMs = now;   // write the file postTapMs later (the ring then holds before + after)
      Serial.println("[radio] double tap -> Wi-Fi open for the phone, recording IMU");
    }
    if (_pendingMs && now - _pendingMs >= postTapMs) { _pendingMs = 0; _write(s); }

    _quiet = quietOnMission && missionActive && !_wake;
    if (!_quiet || now - _lastKickMs < 300) return;
    _lastKickMs = now;
    const uint32_t rv = (uint32_t)roverIp;
    if (rv == 0) return;                    // rover not found yet: kick nobody
    wifi_sta_list_t list;
    if (esp_wifi_ap_get_sta_list(&list) != ESP_OK) return;
    esp_netif_t* ap = esp_netif_get_handle_from_ifkey("WIFI_AP_DEF");
    for (int i = 0; i < list.num; i++) {
      esp_netif_pair_mac_ip_t pair = {};
      memcpy(pair.mac, list.sta[i].mac, 6);
      // no IP yet = just joining; the rover always has its lease -> a station without one is not the rover
      const bool hasIp = esp_netif_dhcps_get_clients_by_mac(ap, 1, &pair) == ESP_OK && pair.ip.addr != 0;
      if (hasIp && pair.ip.addr == rv) { memcpy(_roverMac, list.sta[i].mac, 6); _roverMacOk = true; continue; }   // the rover: never
      if (_roverMacOk && memcmp(_roverMac, list.sta[i].mac, 6) == 0) continue;   // rover re-joining (no lease yet)
      uint16_t aid = 0;
      if (esp_wifi_ap_get_sta_aid(list.sta[i].mac, &aid) == ESP_OK && aid) {
        esp_wifi_deauth_sta(aid);
        _kicked++;
      }
    }
  }

private:
  void _write(Sensors& s) {
    static int16_t ax[ImuManager::kRing], ay[ImuManager::kRing], az[ImuManager::kRing], yw[ImuManager::kRing];
    const uint16_t n = s.imu().tapRecord(ax, ay, az, yw);
    snprintf(_lastFile, sizeof(_lastFile), "/tap_%03u.csv", (unsigned)(_tapsSeen % 1000));
    File f = LittleFS.open(_lastFile, "w");
    if (!f) { Serial.println("[radio] cannot write tap file (flash full / no file system?)"); _lastFile[0] = 0; return; }
    char utc[32];
    const bool hasUtc = s.gpsUtc(utc, sizeof(utc));
    f.printf("# double tap %u, box time %lu ms, GPS clock %s\n", (unsigned)_tapsSeen, (unsigned long)s.imu().lastTapMs(),
             hasUtc ? utc : "unknown (no satellite)");
    f.printf("# 100 Hz, last sample = tap + %lu ms; linear acceleration (gravity removed) and route-free yaw\n",
             (unsigned long)postTapMs);
    f.print("t_ms,ax_ms2,ay_ms2,az_ms2,yaw_deg\n");
    for (uint16_t k = 0; k < n; k++)
      f.printf("%d,%.2f,%.2f,%.2f,%.2f\n", (int)(k - (int)n + 1) * 10, ax[k] * 0.01f, ay[k] * 0.01f, az[k] * 0.01f, yw[k] / 16.0f);
    f.close();
    Serial.printf("[radio] %s written (%u samples, GPS clock %s)\n", _lastFile, (unsigned)n, hasUtc ? utc : "unknown");
  }

  bool _quiet = false, _wake = false, _wasActive = false, _roverMacOk = false;
  uint8_t _roverMac[6] = {0};
  uint16_t _tapsSeen = 0, _kicked = 0;
  uint32_t _lastKickMs = 0, _pendingMs = 0;
  char _lastFile[16] = "";
};
