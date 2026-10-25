// =============================================================================================
//  CAS Box - starter program for the sensor box on the WAVE ROVER
//  Case Study Cooperative and Autonomous Systems, version 1.0 (2026-10-15)
// =============================================================================================
//
//  WHAT IT DOES
//    The box ESP32 is the "brain" of your robot. It opens its own Wi-Fi, serves a web page for
//    your phone, reads all box sensors and drives the WAVE ROVER underneath it over Wi-Fi.
//
//      phone ──Wi-Fi──► box (this program): web page, sensors ──Wi-Fi──► rover (motors)
//
//    Web page: drive pad (hold a key = drive, release = stop), speed slider, LiDAR top view,
//    ultrasonic distances, IMU (roll/pitch/yaw), GPS. "Dev" (top right) shows raw values.
//
//  BEFORE YOU FLASH: set BOX_ID below to the number of your kit (sticker on box and rover).
//    Everything else follows from it - do not change the naming scheme:
//      Wi-Fi name  CAS-Box-<NN>      e.g. kit 7 -> CAS-Box-07
//      password    CASbox<NN>        e.g. kit 7 -> CASbox07
//      web page    http://192.168.<10+N>.1   e.g. kit 7 -> http://192.168.17.1
//      channel     odd kits 1, even kits 11
//    Your rover is already set up to join exactly this Wi-Fi. A wrong BOX_ID means: your rover
//    does not find your box, or your box takes over another team's network.
//
//  START ORDER: switch on the box first, then the rover (the rover looks for its box only
//    while it boots). Phone: join CAS-Box-<NN>, open the web page.
//
//  ARDUINO IDE: board "ESP32 Dev Module", ESP32 core 3.3.x, default partition scheme.
//    Required libraries and versions: README.md. Serial monitor: 115200 baud.
//
//  FILES
//    cas_box.ino        this file: settings, setup(), loop()
//    RoverLink.*        box -> rover link (sends drive commands, reads battery voltage)
//    Sensors.*          all sensors behind one object (ultrasonic, LiDAR, IMU, GPS)
//    LidarScan.h        LiDAR (LD20) reader with noise filtering
//    WebHMI.*           web server and web page
//    UsManager.*        5 ultrasonic sensors (HC-SR04), measured in a background task
//    ImuManager.*       orientation sensor BNO055 (roll / pitch / yaw in the ISO 8855 vehicle frame)
//    GpsManager.*       GPS module NEO-6M
//    DisplayManager.*   small OLED on the box
//
//  SAFETY / BEHAVIOUR
//    The rover stops when you release the key, when the page loses focus, when the box gets no
//    update from the phone for 0.4 s, when RoverLink gets no new set point for 0.5 s, and - in
//    the rover itself - after 3 s without a command. Do not remove these stops.
//    The rover firmware must not be changed.
// =============================================================================================

#include "WebHMI.h"
#include "RoverLink.h"
#include "Sensors.h"
#include "DisplayManager.h"

// ---- YOUR KIT ------------------------------------------------------------------------------
const int BOX_ID = 10;                     // 1..10 = kit number on the box

// ---- settings --------------------------------------------------------------------------------
const float MAX_SPEED = 0.5f;             // drive pad maximum; 0.5 = full rover speed (100 % PWM)
const uint32_t PAD_TIMEOUT_MS = 400;      // no update from the phone for this long -> stop
const int LIDAR_OFFSET_DEG = 90;          // rotates the LiDAR view so that "front" is up

// ---- objects ---------------------------------------------------------------------------------
WebHMI hmi;           // web server + page
RoverLink rover;      // connection to the rover
Sensors sensors;      // all box sensors
DisplayManager disp;  // small OLED on the box
bool oledReady = false;
char boxSsid[16], boxPassword[16];

// OLED: Wi-Fi name, web address, rover state. Started only when the display answers on I2C.
void updateOled() {
  static uint32_t lastTry = 0;
  if (!oledReady) {
    if (millis() - lastTry < 5000) return;
    lastTry = millis();
    if (!i2cPresent(0x3C)) return;
    oledReady = disp.begin();
  }
  String ip = WiFi.softAPIP().toString();
  String r = rover.connected() ? "Rover " + String(rover.voltage(), 1) + "V" : "Rover: search";
  disp.show(boxSsid, ip.c_str(), r.c_str());
}

void setup() {
  Serial.begin(115200);
  delay(200);

  // naming scheme, derived from BOX_ID only (see header)
  snprintf(boxSsid, sizeof(boxSsid), "CAS-Box-%02d", BOX_ID);
  snprintf(boxPassword, sizeof(boxPassword), "CASbox%02d", BOX_ID);
  uint8_t channel = (BOX_ID % 2) ? 1 : 11;

  hmi.setTitle(String("CAS Box ") + BOX_ID);
  hmi.enableDrivePad(MAX_SPEED);
  hmi.setStateText("Starting ...");
  hmi.begin(boxSsid, boxPassword, IPAddress(192, 168, 10 + BOX_ID, 1), channel);

  sensors.begin(LIDAR_OFFSET_DEG);
  rover.begin(100);  // sends the current drive command every 100 ms
}

void loop() {
  uint32_t t0 = micros();
  hmi.tick();
  sensors.update();

  // ---- driving -------------------------------------------------------------------------------
  // Manual driving from the phone. For your own (autonomous) behaviour, compute l and r
  // yourself from the sensors, e.g. sensors.usCm(0) = front ultrasonic in cm, and call
  // rover.setDrive(l, r) every loop pass (range -0.5..0.5 per side).
  float l, r;
  uint32_t age;
  hmi.getDrive(l, r, age);
  if (age > PAD_TIMEOUT_MS) l = r = 0;  // key released, page closed or phone gone
  rover.setDrive(l, r);

  // ---- web page and OLED -----------------------------------------------------------------------
  uint32_t now = millis();
  static uint32_t lastSnap = 0, lastUi = 0, lastOled = 0;
  if (now - lastSnap >= 250) {  // sensor snapshot for the page, 4 times per second
    lastSnap = now;
    hmi.setSensorJson(sensors.toJson());
  }
  if (now - lastUi >= 500) {  // status line + values in the "Dev" view
    lastUi = now;
    hmi.setStateText(rover.statusText());
    hmi.showValue("rover.ip", rover.ip().toString());
    hmi.showValue("rover.voltage_V", rover.voltage());
    hmi.showValue("rover.rtt_ms", rover.rttMs());
    hmi.showValue("rover.ok", (double)rover.okCount());
    hmi.showValue("rover.fail", (double)rover.failCount());
    hmi.showValue("sys.heap_kB", ESP.getFreeHeap() / 1024.0);
  }
  if (now - lastOled >= 1000) {
    lastOled = now;
    updateOled();
  }

  // ---- load report on Serial every 10 s (loop time, free memory, sensor snapshot) -------------
  static uint32_t lastRep = 0, n = 0, maxUs = 0;
  static uint64_t sumUs = 0;
  uint32_t dt = micros() - t0;
  n++; sumUs += dt; if (dt > maxUs) maxUs = dt;
  if (now - lastRep >= 10000) {
    Serial.printf("[sys] loop avg %.2f ms max %.1f ms, heap %u kB, rover %s\n",
                  sumUs / 1000.0 / n, maxUs / 1000.0, ESP.getFreeHeap() / 1024,
                  rover.connected() ? "ok" : "searching");
    Serial.printf("[sys] %s\n", sensors.toJson().c_str());
    lastRep = now; n = 0; sumUs = 0; maxUs = 0;
  }
  delay(2);
}
