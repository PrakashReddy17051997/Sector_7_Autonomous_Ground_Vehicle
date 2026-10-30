// =============================================================================================
//  CAS Box - starter program for the sensor box on the WAVE ROVER
//  Case Study Cooperative and Autonomous Systems, version 1.0 (2026-10-15)
//  + DriveLog (drive recording to flash) and SafetyGuard (emergency stop layer), 2026-10-22
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
//    NEW: "Drive log" card below the drive pad (Start / Stop / Mark / download), and the drive
//    command passes through SafetyGuard before it goes to the rover (see SafetyGuard.h).
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
//  ARDUINO IDE: board "ESP32 Dev Module", ESP32 core 3.3.x, default partition scheme
//    (the default scheme is needed: it holds the ~1.3 MB log storage).
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
//    DriveLog.*         NEW: records drives to flash, web card, download (decode: analyze_log.py)
//    SafetyGuard.h      NEW: tilt / ultrasonic / LiDAR stop on every drive command
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
#include "DriveLog.h"
#include "SafetyGuard.h"
#include "GpsNav.h"
#include "WallFollow.h"
#include "Mission.h"
#include "MotorShaper.h"
#include "PageUi.h"
#include <esp_wifi.h>

// ---- YOUR KIT ------------------------------------------------------------------------------
const int BOX_ID = 4;                      // 1..10 = kit number on the box

// ---- settings --------------------------------------------------------------------------------
const float MAX_SPEED = 0.5f;             // drive pad maximum; 0.5 = full rover speed (100 % PWM)
const uint32_t PAD_TIMEOUT_MS = 400;      // no update from the phone for this long -> stop
const int LIDAR_OFFSET_DEG = 90;          // rotates the LiDAR view so that "front" is up
const bool USE_SAFETY_GUARD = true;       // false = drive commands go to the rover unchanged

// ---- objects ---------------------------------------------------------------------------------
WebHMI hmi;           // web server + page
RoverLink rover;      // connection to the rover
Sensors sensors;      // all box sensors
DisplayManager disp;  // small OLED on the box
DriveLog logger;      // drive recording to flash (web card "Drive log")
SafetyGuard guard;    // emergency stop layer, thresholds in guard.cfg
GpsNav nav;           // autonomous GPS waypoint mode (web card "Autonomous (GPS)")
WallFollow wall;      // autonomous LiDAR wall following (web card "Wall following")
Mission mission;      // full-auto run: route B -> H leg by leg (RouteFollow, teach & repeat of log_009)
MotorShaper shaper;   // dead band / pivot / breakaway compensation (the rover needs ~0.5 to start turning)
bool oledReady = false;
char boxSsid[16], boxPassword[16];
uint8_t safetyReason = SafetyGuard::OK;  // last guard decision, for the page
String resetReason;   // why the box started (see /boot.txt)

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

  // Less delay box -> rover (log_004): the rover's Wi-Fi sleeps between beacons and only wakes
  // every DTIM beacons to fetch data. DTIM 1 = it wakes at every beacon (100 ms instead of 200 ms).
  wifi_config_t apCfg;
  if (esp_wifi_get_config(WIFI_IF_AP, &apCfg) == ESP_OK) {
    apCfg.ap.dtim_period = 1;
    esp_wifi_set_config(WIFI_IF_AP, &apCfg);
  }
  WiFi.setTxPower(WIFI_POWER_19_5dBm);  // full transmit power (box is inside a closed case)

  // Drive log: mounts the flash file system and adds its web routes (/log.js, /log/...).
  // The very first boot formats the storage once (a few seconds).
  logger.begin(hmi.server());
  resetReason = logBootReason();   // needs the file system from logger.begin()
  registerPageUi(hmi.server());    // combined sensor table, /boot.txt
  nav.begin(hmi.server());     // loads the B->H route from log_003
  wall.begin(hmi.server());
  mission.begin(hmi.server());

  // Safety thresholds - change here if needed (defaults: see SafetyGuard.h)
  // log_002 (2026-10-22): the front-angled ultrasonic looks at the floor (reads ~17 cm on flat
  // ground) and blocked 64 % of all forward commands -> stop-go jerking. Off for obstacle stops;
  // it stays in the log as a candidate cliff sensor (reading jumps up at an edge).
  guard.cfg.useFrontAngled = false;
  // guard.cfg.frontStopCm = 25;

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
  const float padL = l, padR = r;       // raw pad command (GpsNav needs it as dead-man switch)
  if (age > PAD_TIMEOUT_MS) l = r = 0;  // key released, page closed or phone gone

  // Full-auto mission (heartbeat from the page) - otherwise manual driving from the pad.
  mission.pollRequests(sensors, wall, nav);
  if (mission.active()) mission.steer(sensors, wall, nav, padL, padR, age, l, r);

  const float cmdL = l, cmdR = r;       // what was asked for (logged)
  // ramp legs of the route: the LiDAR sees the ramp surface as a wall -> LiDAR stop off there
  // (front ultrasonic 25 cm and tilt stay on), see RouteFollow::rampLegs
  guard.cfg.lidarStopCm = mission.route.onRamp() ? 0 : 30;
  // log_018: at the ramp foot the front ultrasonic read 9-23 cm (the ramp surface) -> stop / kick /
  // stop. On ramp legs it stops only for something really close.
  guard.cfg.frontStopCm = mission.route.onRamp() ? 10 : 25;
  safetyReason = USE_SAFETY_GUARD ? guard.apply(sensors, l, r) : (uint8_t)SafetyGuard::OK;

  // make small commands move the rover (dead band, pivot minimum, breakaway kick); no kick while
  // the guard limits for tilt (slow careful reversing)
  shaper.setBattery(rover.voltage());   // battery compensation (MotorShaper: full pack -> smaller cmds)
  shaper.setPitch(sensors.pitchDeg());  // slope: less forward output downhill, a bit more uphill
  shaper.boost = mission.active() ? mission.route.boost() : 0.0f;   // no progress on rough ground: more
  shaper.apply(l, r, safetyReason != SafetyGuard::TILT);

  rover.setDrive(l, r);                 // every loop pass, after guard and shaper
  // route progress = what really went to the rover, as the command it equals at 11.6 V (so the
  // effort per metre does not change with the battery)
  mission.feedOutput(l * shaper.equivalent(), r * shaper.equivalent());

  // ---- drive log (records 4x per second while "Start" is active) -----------------------------
  DriveLog::Sample smp;
  smp.cmdL = cmdL;
  smp.cmdR = cmdR;
  smp.outL = l;
  smp.outR = r;
  smp.safety = safetyReason;
  smp.batteryV = rover.voltage();
  smp.rttMs = rover.rttMs();             // box -> rover command delay (log_004: "lag")
  smp.navByte = mission.logByte();       // route leg + state (in the mag_cal byte, see DriveLog.h)
  smp.navLeg = mission.logLeg();         // full leg number (the byte above holds only legs 1..15)
  if (mission.active()) {                // v3 debug: who drove, what the route code believed
    const RouteFollow& rt = mission.route;
    const uint8_t w = rt.why();
    smp.dbg = (w == RouteFollow::WHY_TAP ? 1 : 0) | (w == RouteFollow::WHY_SETTLE ? 2 : 0) |
              (shaper.kicking() ? 4 : 0) | (w == RouteFollow::WHY_STEP ? 8 : 0) | (rt.pathLocked() ? 16 : 0);
    smp.ctrlDeg = rt.ctrlErrDeg();
    if (rt.pathOn()) {
      const LocalLocalizer& P = rt.local();
      smp.lpX = P.x(); smp.lpY = P.y(); smp.lpThDeg = P.th() * RAD_TO_DEG; smp.lpScore = P.score();
      smp.carrotDeg = P.headingErrorToWaypoint(rt.pathLookM) * RAD_TO_DEG;
    }
  }
  logger.update(sensors, smp);

  // ---- web page and OLED -----------------------------------------------------------------------
  uint32_t now = millis();
  static uint32_t lastSnap = 0, lastUi = 0, lastOled = 0;
  if (now - lastSnap >= 250) {  // sensor snapshot for the page, 4 times per second
    lastSnap = now;
    hmi.setSensorJson(sensors.toJson());
  }
  if (now - lastUi >= 500) {  // status line + values in the "Dev" view
    lastUi = now;
    String state = rover.statusText();
    if (safetyReason != SafetyGuard::OK) state = String(SafetyGuard::reasonText(safetyReason)) + " | " + state;
    if (mission.active()) state = "MISSION " + mission.statusText() + " | " + state;
    if (millis() < 120000 && resetReason != "power on" && !resetReason.startsWith("reset button"))
      state = "BOX RESTARTED: " + resetReason + " | " + state;   // first 2 minutes after a reset
    if (logger.active()) state = "● REC | " + state;
    hmi.setStateText(state);
    hmi.showValue("rover.ip", rover.ip().toString());
    hmi.showValue("rover.voltage_V", rover.voltage());
    hmi.showValue("motor.vscale", String(shaper.vScale(), 3) + (shaper.vComp ? "" : " (off)"));
    hmi.showValue("motor.slope", String(shaper.slopeFactor(), 2) + " / boost " + String(shaper.boost, 2));
    hmi.showValue("route.tap", String(mission.route.tapGain(), 0) + " deg/s, drift " +
                  String(mission.route.driftDegS(), 1) + " deg/s, progress " + String(mission.route.progressCm(), 1) + " cm");
    hmi.showValue("rover.rtt_ms", rover.rttMs());
    hmi.showValue("rover.ok", (double)rover.okCount());
    hmi.showValue("rover.fail", (double)rover.failCount());
    hmi.showValue("sys.heap_kB", ESP.getFreeHeap() / 1024.0);
    hmi.showValue("safety", String(SafetyGuard::reasonText(safetyReason)));
    hmi.showValue("log", logger.statusText());
    hmi.showValue("sys.reset", resetReason);
    hmi.showValue("mission", mission.statusText());
    hmi.showValue("route", String(mission.route.legIndex() + 1) + " " + mission.route.msg() + " eff " +
                           String(mission.route.effortFrac() * 100.0f, 0) + "%");
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
    Serial.printf("[sys] loop avg %.2f ms max %.1f ms, heap %u kB (min %u kB), rover %s, %s\n",
                  sumUs / 1000.0 / n, maxUs / 1000.0, (unsigned)(ESP.getFreeHeap() / 1024),
                  (unsigned)(ESP.getMinFreeHeap() / 1024),
                  rover.connected() ? "ok" : "searching", logger.statusText().c_str());
    Serial.printf("[sys] %s\n", sensors.toJson().c_str());
    lastRep = now; n = 0; sumUs = 0; maxUs = 0;
  }
  delay(2);
}
