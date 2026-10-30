#include "RoverLink.h"
#include <HTTPClient.h>
#include <esp_wifi.h>
#include <esp_netif.h>

static const uint16_t kHttpTimeoutMs = 400;  // a command normally takes 20-200 ms
static const uint8_t kStopRepeats = 3;       // confirmed stops after driving
static const uint8_t kLostAfterFails = 10;   // consecutive failures -> search the rover again
static const uint32_t kDeadmanMs = 500;      // set point older than this -> stop
static const uint32_t kSearchEveryMs = 2000;
static const uint32_t kIdleEveryMs = 500;    // idle: stop or status poll, alternately
static const uint32_t kMinGapMs = 30;        // changed command: send at once, but not more often

// Percent-encodes everything except unreserved URL characters (the JSON goes into a query string).
static String urlEncode(const String& s) {
  const char* hex = "0123456789ABCDEF";
  String out;
  for (size_t i = 0; i < s.length(); i++) {
    const char c = s[i];
    if (isalnum((unsigned char)c) || c == '-' || c == '_' || c == '.' || c == '~') {
      out += c;
    } else {
      out += '%';
      out += hex[(c >> 4) & 0xF];
      out += hex[c & 0xF];
    }
  }
  return out;
}

void RoverLink::begin(uint32_t periodMs) {
  _periodMs = periodMs;
  xTaskCreatePinnedToCore(_taskEntry, "roverLink", 6144, this, 1, &_taskHandle, 0);
}

void RoverLink::setDrive(float left, float right) {
  portENTER_CRITICAL(&_mux);
  _setL = left;
  _setR = right;
  _setMs = millis();
  portEXIT_CRITICAL(&_mux);
  // changed (a tap starts / ends, a stop): wake the link task now instead of in the next 100 ms slot
  if (_taskHandle && (fabsf(left - _lastL) > 0.005f || fabsf(right - _lastR) > 0.005f)) {
    _lastL = left;
    _lastR = right;
    xTaskNotifyGive(_taskHandle);
  }
}

String RoverLink::statusText() const {
  if (!connected()) return "Rover: searching ...";
  String s = "Rover " + ip().toString();
  const float v = _voltage;
  if (v > 0) s += ", " + String(v, 2) + " V";
  s += ", " + String((float)_rttMs, 0) + " ms";
  return s;
}

void RoverLink::_taskEntry(void* self) { static_cast<RoverLink*>(self)->_task(); }

// Sends one JSON command to ip. Returns the HTTP code (200 = ok, < 0 = connection error).
// Counts successes/failures; after kLostAfterFails failures in a row the rover counts as lost.
int RoverLink::_cmd(uint32_t ip, const String& json, String* reply) {
  HTTPClient http;
  http.setTimeout(kHttpTimeoutMs);
  http.setConnectTimeout(kHttpTimeoutMs);
  const uint32_t t0 = millis();
  if (!http.begin("http://" + IPAddress(ip).toString() + "/js?json=" + urlEncode(json))) return -100;
  const int code = http.GET();
  if (code == 200) {
    const String body = http.getString();
    if (reply) *reply = body;
    const float rtt = millis() - t0;
    _rttMs = (_rttMs == 0) ? rtt : 0.8f * _rttMs + 0.2f * rtt;  // smoothed
    _ok++;
    _failRow = 0;
  } else {
    _fail++;
    if (++_failRow >= kLostAfterFails && _ipRaw != 0) {
      Serial.println("[rover] lost, searching again");
      _ipRaw = 0;
    }
  }
  http.end();
  return code;
}

// The rover is the station on our access point that answers {"T":405} with its Wi-Fi config.
// The IP is only published (connected() = true) after that check.
bool RoverLink::_find() {
  wifi_sta_list_t list;
  if (esp_wifi_ap_get_sta_list(&list) != ESP_OK) return false;
  static uint32_t lastLog = 0;
  if (millis() - lastLog > 10000) {
    lastLog = millis();
    Serial.printf("[rover] searching, %d station(s) on the AP\n", list.num);
  }
  esp_netif_t* ap = esp_netif_get_handle_from_ifkey("WIFI_AP_DEF");
  for (int i = 0; i < list.num; i++) {
    esp_netif_pair_mac_ip_t pair = {};
    memcpy(pair.mac, list.sta[i].mac, 6);
    if (esp_netif_dhcps_get_clients_by_mac(ap, 1, &pair) != ESP_OK || pair.ip.addr == 0) continue;
    String reply;
    if (_cmd(pair.ip.addr, "{\"T\":405}", &reply) == 200 && reply.indexOf("wifi_mode_on_boot") >= 0) {
      _failRow = 0;
      _ipRaw = pair.ip.addr;
      Serial.printf("[rover] found at %s\n", IPAddress(_ipRaw).toString().c_str());
      return true;
    }
  }
  return false;
}

void RoverLink::_task() {
  uint32_t lastFind = 0, lastIdle = 0;
  uint8_t stopsLeft = 0;
  bool idleStatus = false;
  for (;;) {
    const uint32_t start = millis();
    const uint32_t ip = _ipRaw;
    if (ip == 0) {
      if (start - lastFind >= kSearchEveryMs) {
        lastFind = start;
        _find();
      }
      vTaskDelay(pdMS_TO_TICKS(_periodMs));
      continue;
    }

    float l, r;
    uint32_t setMs;
    portENTER_CRITICAL(&_mux);
    l = _setL; r = _setR; setMs = _setMs;
    portEXIT_CRITICAL(&_mux);
    if (millis() - setMs > kDeadmanMs) l = r = 0;  // millis() read after setMs: never negative

    if (l != 0 || r != 0) {
      _cmd(ip, String("{\"T\":1,\"L\":") + String(l, 2) + ",\"R\":" + String(r, 2) + "}", nullptr);
      stopsLeft = kStopRepeats;
    } else if (stopsLeft) {
      // a stop only counts when the rover confirmed it (HTTP 200); lost stops are repeated
      if (_cmd(ip, "{\"T\":1,\"L\":0,\"R\":0}", nullptr) == 200) stopsLeft--;
    } else if (start - lastIdle >= kIdleEveryMs) {
      // idle: alternately repeat the stop (clears anything stale) and read the battery voltage
      lastIdle = start;
      idleStatus = !idleStatus;
      String reply;
      if (!idleStatus) {
        _cmd(ip, "{\"T\":1,\"L\":0,\"R\":0}", nullptr);
      } else if (_cmd(ip, "{\"T\":130}", &reply) == 200) {
        const int i = reply.indexOf("\"v\":");
        if (i >= 0) _voltage = reply.substring(i + 4).toFloat();
      }
    }
    // keep the period; one command can take up to ~200 ms (rover Wi-Fi power save).
    // A changed set point (setDrive) ends the wait early - after at least kMinGapMs.
    const uint32_t used = millis() - start;
    if (used < kMinGapMs) vTaskDelay(pdMS_TO_TICKS(kMinGapMs - used));
    const uint32_t used2 = millis() - start;
    ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(used2 < _periodMs ? _periodMs - used2 : 1));
  }
}
