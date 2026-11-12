// =============================================================================================
//  CAS Box - starter program for the sensor box on the WAVE ROVER
//  Case Study Cooperative and Autonomous Systems, version 1.0 (2026-10-15)
//  + DriveLog (drive recording to flash) and SafetyGuard (emergency stop layer), 2026-10-22
//  v3 (2026-11-10, branch prakash_v3):
//    - START ALIGN: put the rover down facing ANY direction at the start; it finds its heading from
//      the LiDAR (full-circle match to the leg map), turns on the spot to the path, then drives.
//    - LANDMARK LEGS: the Open Sea (leg 2) is driven on landmark lines - wall behind = heading anchor,
//      store front = side PID at 6.75 m, wall ahead = stop (LandmarkLeg.h, LandmarkRoute.h).
//    - MISSION RADIO: phone off the Wi-Fi during the mission (rover link untouched), double tap on
//      the box -> Wi-Fi open again + IMU recording /tap_NNN.csv with GPS clock (MissionRadio.h).
//  v4 (2026-11-11, from v3 + the IMU/wall/obstacle work of 05.11.; STEP6_2026-11-11.md):
//    - SMART OBSTACLES: new things on the locked path are seen from 1.5 m (v3: 0.45 m - with the 2 s S-curve
//      stop the rover rolled into them); stop for an obstacle = FIRM (0.6 s); a moving one (person) is waited
//      for, a static one is driven around at once (width from the LiDAR, free side, back to the line).
//      Switch SMART_OBSTACLES below. Closed-loop test on the real route code: tools/v4_test/obstacle_sim.
//    - v4b SMOOTH REROUTE: a static obstacle seen 1.2-3.5 m ahead is passed on a planned smooth curve (out, beside
//      it, back onto the taught path) without stopping; re-checked every scan, falls back to stop + 45 deg detour.
//      Switch: mission.route.smoothReroute (RouteFollow.h).
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
//  ARDUINO IDE: board "ESP32 Dev Module", ESP32 core 3.3.x, partition scheme "No OTA (2MB APP/2MB SPIFFS)"
//    + Erase All Flash the first time (STEP5: v3/v4 are ~1.6 MB, more than the default 1.25 MB app partition).
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
#include "MissionRadio.h"
#include "LandmarkRoute.h"
#include <esp_wifi.h>

// ---- YOUR KIT ------------------------------------------------------------------------------
const int BOX_ID = 7;                      // 1..10 = kit number on the box (10.11.: box 7 + rover 7)
// Wi-Fi transmit power: at start (smallest current spike) and while running. Raise WIFI_TX_RUN to
// WIFI_POWER_19_5dBm only if the phone or the rover lose the connection - and only with a good supply.
#define WIFI_TX_START WIFI_POWER_8_5dBm
#define WIFI_TX_RUN WIFI_POWER_15dBm

// ---- settings --------------------------------------------------------------------------------
const float MAX_SPEED = 0.30f;            // drive pad + mission maximum (05.11.: 0.25 + 0.05; slider steps 0.05!)
const uint32_t PAD_TIMEOUT_MS = 400;      // no update from the phone for this long -> stop
// LiDAR mounting angle. Our maps (LocalPath.h, RouteMap.h), crabDeg -7.6 and the landmark lineOff were all taught
// with BOX 4 at 90, and box 4's LiDAR sits 7 deg turned (Arjun: its true offset is 83). So for our maps a box needs
// LIDAR_OFFSET_DEG = (its true offset) + 7. BOX 7 IS NOT MEASURED YET: stand the rover parallel to a long straight
// wall, read "lidar.cal" on the Dev page, put its "for our maps" value here, reflash (STEP5, section 6).
const int LIDAR_OFFSET_DEG = 90;
const int MAP_LIDAR_ERR_DEG = 7;          // box 4's mounting error baked into our taught maps
const bool USE_SAFETY_GUARD = true;       // false = drive commands go to the rover unchanged
// ---- v3 switches (each can be turned off on its own for the step-by-step field test) -------------
const bool START_ALIGN = true;            // false = old start (rover parallel to the wall)
const uint8_t OPEN_SEA_LEG = 2;           // route leg driven on landmarks (kOpenSea), 0 = off
const uint8_t BLOCK_I_LEG = 0;            // last leg to Block I (kBlockI), 0 = off (not taught yet)
const bool LMK_LEGS_6_15 = true;          // legs 6-15 on landmark lines (LandmarkRoute.h kLmkLegs), false = taught path
const bool MISSION_QUIET = false;         // START value of the page switch "Phone OFF during the mission" (11.11.:
                                          // off = tests with feedback; switch it on on the page for the demo)
// 11.11.: rover 7 needs 0.5 to pivot - the pad turn keys and the Left/Right 90 buttons pivot at this,
// ramped 0 -> 0.5 in 1 s (MotorShaper wUp 0.5/s), whatever the speed slider says
const float PIVOT_CMD = 0.50f;
const float TURN90_STOP_EARLY_MAX = 20.0f; // released this much early at most (it coasts on; Kethan: 14 at 0.35)
// Wheel trim start value (learned on from here while rolling straight). Rover 7: +0.25 (Arjun, campus_10.3; his
// learning then added up to +0.30 more). Rover 4: -0.046. + = right wheel faster.
const float ROVER_TRIM_START = 0.25f;
// ---- v4 switch ---------------------------------------------------------------------------------------------------
// SMART_OBSTACLES: early detection (1.5 m) of NEW things on the locked path + detour of static ones (RouteFollow
// "SMART OBSTACLES"). false = the v3 handling (wait 3 s, sidestep). The FIRM stop for obstacles is on either way
// (it is what keeps the rover from rolling into an obstacle after the 2 s S-curve).
const bool SMART_OBSTACLES = true;

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
MissionRadio radio;   // v3: phone off during the mission, double tap -> back on + IMU recording

// ---- v3 (11.11.) Left / Right 90 (Kethan's turn90, cas_box_kethan_log, at 0.5 for rover 7) -------------------
// gyro-controlled pivot of 90 deg: command PIVOT_CMD (ramped by MotorShaper), progress = gyro change, released when
// the rest is smaller than what it still coasts (|rate| * 0.45 s, at most TURN90_STOP_EARLY_MAX); stops on: the
// cancel button / any drive key, no IMU, 15 s, or less than 1 deg in 2 s at full command ("no turn response").
volatile int8_t t90Req = 0; volatile bool t90New = false;
char t90Msg[64] = "";
// 12.11.: ROLLING START (logs 030-033, rover 7: a pivot from standstill turned in 7 % of the tries, while rolling in
// 51 %): the button first rolls straight at TURN90_ROLL_CMD (back if the rear is free, else forward) for
// TURN90_ROLL_MS and then pivots at 0.5 WITHOUT stopping. No response for 2 s -> rolls again the other way (3 tries).
const float TURN90_ROLL_CMD = 0.30f;
const uint32_t TURN90_ROLL_MS = 1200;
struct Turn90 {
  bool on = false; int8_t dir = 1; float prog = 0, prevYaw = 0, rate = 0, respProg = 0;
  uint32_t t0 = 0, respMs = 0, lastMs = 0;
  bool rolling = false; int8_t rollDir = -1; uint8_t tries = 0; uint32_t rollMs = 0, rollEndMs = 0; bool carry = false;
} t90;
static bool t90Free(bool back) {   // LiDAR (4 deg bins, 0 = ahead) + ultrasonic 3 (back): ~0.55 m free
  if (back) {
    for (uint8_t i = 37; i <= 52; i++) { const uint16_t cm = sensors.lidarCm(i); if (cm > 25 && cm < 55) return false; }
    const int b = sensors.usCm(3); return !(b > 2 && b < 40);
  }
  for (int8_t k = -7; k <= 7; k++) { const uint16_t cm = sensors.lidarCm((uint8_t)((k + 90) % 90)); if (cm > 25 && cm < 60) return false; }
  return true;
}
static void t90Roll(uint32_t now) {   // next roll: back first, then alternating; none possible -> pivot only
  int8_t want = t90.tries == 0 ? -1 : -t90.rollDir;
  if (!t90Free(want < 0)) want = -want;
  if (!t90Free(want < 0)) { t90.rolling = false; return; }
  t90.rolling = true; t90.rollDir = want; t90.rollMs = now;
}
static float wrap180f(float a) { a = fmodf(a + 180.0f, 360.0f); if (a < 0) a += 360.0f; return a - 180.0f; }
void turn90Stop(const char* why) { t90.on = false; snprintf(t90Msg, sizeof(t90Msg), "%s (%.0f deg)", why, t90.prog); Serial.printf("[turn90] %s\n", t90Msg); }
// returns true while it drives the wheels
bool turn90Step(float padL, float padR, uint32_t padAge, float& l, float& r) {
  const uint32_t now = millis();
  if (t90New) {
    t90New = false;
    if (t90Req == 0) { if (t90.on) turn90Stop("cancelled"); }
    else if (!sensors.imuOk()) { snprintf(t90Msg, sizeof(t90Msg), "no IMU - no turn"); }
    else {
      t90 = Turn90(); t90.on = true; t90.dir = t90Req; t90.t0 = t90.respMs = t90.lastMs = now; t90.prevYaw = sensors.yawNowDeg();
      t90Roll(now);
      snprintf(t90Msg, sizeof(t90Msg), "%s 90: %s", t90.dir > 0 ? "Left" : "Right", t90.rolling ? "rolling start" : "starting");
    }
  }
  if (!t90.on) return false;
  if (padAge <= PAD_TIMEOUT_MS && (padL != 0 || padR != 0)) { turn90Stop("drive key - cancelled"); return false; }
  if (!sensors.imuOk()) { turn90Stop("IMU invalid - stopped"); return false; }
  if (now - t90.t0 > 20000) { turn90Stop("20 s - stopped"); return false; }
  const float y = sensors.yawNowDeg();
  const float dy = wrap180f(y - t90.prevYaw);
  const float dt = (now - t90.lastMs) / 1000.0f;
  t90.prevYaw = y; t90.lastMs = now;
  if (fabsf(dy) > 30) { turn90Stop("IMU jump - stopped"); return false; }
  if (dt > 0 && dt < 0.2f) t90.rate = 0.8f * t90.rate + 0.2f * (dy / dt);
  t90.prog += t90.dir * dy;
  if (t90.prog < -5) { turn90Stop("turned the wrong way - stopped"); return false; }
  const float coast = fminf(TURN90_STOP_EARLY_MAX, fabsf(t90.rate) * 0.45f);
  if (t90.prog >= 90.0f - coast) { turn90Stop("done"); return false; }
  if (t90.prog - t90.respProg >= 1.0f) { t90.respProg = t90.prog; t90.respMs = now; }
  if (t90.rolling) {                                      // straight roll, then into the pivot without a stop
    if (now - t90.rollMs >= TURN90_ROLL_MS || !t90Free(t90.rollDir < 0)) {
      t90.carry = t90Free(t90.rollDir < 0);                 // still free: keep rolling INTO the pivot (fading)
      t90.rolling = false; t90.tries++; t90.respMs = t90.rollEndMs = now; t90.respProg = t90.prog;
    } else {
      l = r = t90.rollDir * TURN90_ROLL_CMD;
      snprintf(t90Msg, sizeof(t90Msg), "%s 90: rolling %s (try %u)", t90.dir > 0 ? "Left" : "Right", t90.rollDir < 0 ? "back" : "fwd", t90.tries + 1);
      return true;
    }
  }
  if (now - t90.respMs > 2000) {
    if (t90.tries < 3) { t90Roll(now); if (t90.rolling) { t90.respMs = now; l = r = t90.rollDir * TURN90_ROLL_CMD; return true; } }
    turn90Stop("no turn response - stopped"); return false;
  }
  float cv = 0;                                           // rolling speed carried into the pivot, fading over 1.2 s
  if (t90.carry && now - t90.rollEndMs < 1200 && t90Free(t90.rollDir < 0))
    cv = t90.rollDir * TURN90_ROLL_CMD * (1.0f - (now - t90.rollEndMs) / 1200.0f);
  else t90.carry = false;
  l = cv - t90.dir * PIVOT_CMD; r = cv + t90.dir * PIVOT_CMD;
  snprintf(t90Msg, sizeof(t90Msg), "%s 90: %.0f / 90 deg", t90.dir > 0 ? "Left" : "Right", t90.prog);
  return true;
}
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
  if (radio.quiet()) { disp.show(boxSsid, "MISSION: phone off", "2x tap = Wi-Fi on"); return; }
  disp.show(boxSsid, ip.c_str(), r.c_str());
}

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("[boot] 1 setup start");

  // naming scheme, derived from BOX_ID only (see header)
  snprintf(boxSsid, sizeof(boxSsid), "CAS-Box-%02d", BOX_ID);
  snprintf(boxPassword, sizeof(boxPassword), "CASbox%02d", BOX_ID);
  uint8_t channel = (BOX_ID % 2) ? 1 : 11;

  hmi.setTitle(String("CAS Box ") + BOX_ID);
  hmi.enableDrivePad(MAX_SPEED);
  hmi.setStateText("Starting ...");
  // Gentle start (2026-11-01, brownout resets at power-on): the Wi-Fi radio starts at reduced transmit
  // power, so its current spike is smaller; full power is not needed - the rover is next to the box.
  WiFi.mode(WIFI_AP);
  WiFi.setTxPower(WIFI_TX_START);
  Serial.println("[boot] 2 wifi starting");
  hmi.begin(boxSsid, boxPassword, IPAddress(192, 168, 10 + BOX_ID, 1), channel);
  Serial.println("[boot] 3 wifi up");

  // Less delay box -> rover (log_004): the rover's Wi-Fi sleeps between beacons and only wakes
  // every DTIM beacons to fetch data. DTIM 1 = it wakes at every beacon (100 ms instead of 200 ms).
  wifi_config_t apCfg;
  if (esp_wifi_get_config(WIFI_IF_AP, &apCfg) == ESP_OK) {
    apCfg.ap.dtim_period = 1;
    esp_wifi_set_config(WIFI_IF_AP, &apCfg);
  }
  WiFi.setTxPower(WIFI_TX_RUN);         // was 19.5 dBm; lower = smaller current peaks (brownout)

  // Drive log: mounts the flash file system and adds its web routes (/log.js, /log/...).
  // The very first boot formats the storage once (a few seconds).
  logger.begin(hmi.server());
  resetReason = logBootReason();   // needs the file system from logger.begin()
  registerPageUi(hmi.server());    // combined sensor table, /boot.txt
  nav.begin(hmi.server());     // loads the B->H route from log_003
  wall.begin(hmi.server());
  mission.begin(hmi.server());
  // v3 settings
  mission.route.alignStart = START_ALIGN;
  mission.route.trim = ROVER_TRIM_START;
  mission.route.smartAvoid = SMART_OBSTACLES;   // v4
  if (OPEN_SEA_LEG) mission.route.setLandmarkLeg(OPEN_SEA_LEG - 1, kOpenSea, LMK_COUNT(kOpenSea));
  if (BLOCK_I_LEG) mission.route.setLandmarkLeg(BLOCK_I_LEG - 1, kBlockI, LMK_COUNT(kBlockI));
  // 11.11. (log_032): legs 6-15 (ramp M51-M57 and the open area M57-M72) on landmark lines, tables from log_027
  if (LMK_LEGS_6_15)
    for (const LmkLegReg& g : kLmkLegs) mission.route.setLandmarkLeg(g.leg - 1, g.ph, g.n, g.lenM);
  mission.needHeartbeat = !MISSION_QUIET;   // phone off -> the mission must run without the page heartbeat
  radio.quietOnMission = MISSION_QUIET;
  radio.onChange = [](bool q) { mission.needHeartbeat = !q; };
  mission.route.kethanCmd = PIVOT_CMD;      // route pivots at 0.5 too (rover 7)
  // Left / Right 90 buttons (page): d = 1 left, -1 right, 0 cancel
  hmi.server()->on("/turn90", HTTP_GET, [](AsyncWebServerRequest* req) {
    if (req->hasParam("d")) t90Req = (int8_t)constrain(req->getParam("d")->value().toInt(), -1, 1), t90New = true;
    req->send(200, "text/plain", "ok");
  });
  hmi.server()->on("/turn90.json", HTTP_GET, [](AsyncWebServerRequest* req) {
    req->send(200, "application/json", String("{\"msg\":\"") + t90Msg + "\"}");
  });
  radio.begin(hmi.server());

  // Safety thresholds - change here if needed (defaults: see SafetyGuard.h)
  // log_002 (2026-10-22): the front-angled ultrasonic looks at the floor (reads ~17 cm on flat
  // ground) and blocked 64 % of all forward commands -> stop-go jerking. Off for obstacle stops;
  // it stays in the log as a candidate cliff sensor (reading jumps up at an edge).
  guard.cfg.useFrontAngled = false;
  // guard.cfg.frontStopCm = 25;

  Serial.println("[boot] 4 log + web ready, starting sensors (LiDAR motor) in 0.5 s");
  delay(500);                        // let the supply recover from the Wi-Fi start first
  sensors.begin(LIDAR_OFFSET_DEG);
  Serial.println("[boot] 5 sensors started");
  rover.begin(100);  // sends the current drive command every 100 ms
  Serial.println("[boot] 6 setup done");
}

void loop() {
  uint32_t t0 = micros();
  hmi.tick();
  sensors.update();
  radio.update(sensors, mission.active(), rover.ip());   // v3: phone off during the mission / double tap

  // ---- driving -------------------------------------------------------------------------------
  // Manual driving from the phone. For your own (autonomous) behaviour, compute l and r
  // yourself from the sensors, e.g. sensors.usCm(0) = front ultrasonic in cm, and call
  // rover.setDrive(l, r) every loop pass (range -0.5..0.5 per side).
  float l, r;
  uint32_t age;
  hmi.getDrive(l, r, age);
  const float padL = l, padR = r;       // raw pad command (GpsNav needs it as dead-man switch)
  if (age > PAD_TIMEOUT_MS) l = r = 0;  // key released, page closed or phone gone

  // 11.11.: pad turn keys (left/right on the spot) pivot at PIVOT_CMD - the slider's 0.30 does not turn rover 7
  if (l * r < 0) { const float d = r > 0 ? 1.0f : -1.0f; l = -d * PIVOT_CMD; r = d * PIVOT_CMD; }

  // Full-auto mission (heartbeat from the page) - otherwise manual driving from the pad.
  mission.pollRequests(sensors, wall, nav);
  if (mission.active()) { if (t90.on) turn90Stop("mission started"); mission.steer(sensors, wall, nav, padL, padR, age, l, r); }
  else turn90Step(padL, padR, age, l, r);   // Left / Right 90 buttons (manual only)

  shaper.firm = mission.active() && mission.route.firmStop();   // v4: obstacle stop / back-up in 0.6 s, not 2 s
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
  // v3 (log_030): turn tactics need full turn power quickly (Arjun: pivots worked 36 % with full power at once,
  // 13 % with a slow ramp) - 0 -> 0.50 in 0.25 s while a tactic turn runs; the stop ramp stays gentle (wDown)
  shaper.wUp = 0.50f;   // 11.11.: every pivot 0 -> 0.5 in 1 s (Kethan's 1 s ramp, rover 7 needs 0.5)
  shaper.wheelRamp = mission.active() && mission.route.wheelRamp();   // 12.11.: K-turn pieces keep their curve
  shaper.apply(l, r, safetyReason != SafetyGuard::OK);   // gradual; a guard stop (obstacle / tilt) drops the speed at once

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
    hmi.showValue("route.avoid", mission.route.avoidPhase() ? String("phase ") + mission.route.avoidPhase() + " (1 out, 2 pass, 3 back), " +
                  mission.route.msg() : String("-"));
    if (safetyReason != SafetyGuard::OK) state = String(SafetyGuard::reasonText(safetyReason)) + " | " + state;
    if (mission.active()) state = "MISSION " + mission.statusText() + " | " + state;
    if (millis() < 120000 && resetReason != "power on" && !resetReason.startsWith("reset button"))
      state = "BOX RESTARTED: " + resetReason + " | " + state;   // first 2 minutes after a reset
    if (logger.active()) state = "● REC | " + state;
    hmi.setStateText(state);
    hmi.showValue("rover.ip", rover.ip().toString());
    hmi.showValue("rover.voltage_V", rover.voltage());
    hmi.showValue("drive.v_w", String(shaper.v(), 3) + " / " + String(shaper.w(), 3));
    hmi.showValue("motor.slope", String(shaper.slopeFactor(), 2) + " / boost " + String(shaper.boost, 2));
    hmi.showValue("route.ctrl", "err " + String(mission.route.ctrlErrDeg(), 1) + " deg, progress " +
                  String(mission.route.progressCm(), 1) + " cm" + (mission.route.onYawLeg() ? ", YAW-ONLY leg" : "") +
                  (sensors.imuOk() ? "" : ", IMU INVALID"));
    hmi.showValue("route.track", "look " + String(mission.route.lookAhead(), 2) + " m, off path " +
                  String(mission.route.crossTrack() * 100.0f, 0) + " cm (+ = left), I " + String(mission.route.xteI(), 1) +
                  " deg" + (mission.route.onRamp() ? ", ON RAMP" : ""));
    // v3
    hmi.showValue("turn90", t90Msg);
    hmi.showValue("route.turn", String(mission.route.tacticActive() ? mission.route.tacticMsg() : "-") +
                  ", floor US " + String(sensors.usCm(4)) + " cm, back US " + String(sensors.usCm(3)) + " cm");
    hmi.showValue("route.trim", String(mission.route.trim, 3) + " (start " + String(ROVER_TRIM_START, 2) + ", " +
                  String(mission.route.driveSeconds(), 0) + " s rolled)");
    hmi.showValue("radio", radio.statusText());
    // v3 LiDAR calibration: a long straight wall left / right of a rover standing PARALLEL to it. Foot of the wall
    // should be +90 (left) / -90 (right); true offset = LIDAR_OFFSET_DEG + (foot - expected). Averaged over scans.
    {
      static float calEst = NAN; static uint16_t calN = 0;
      uint16_t sc[Sensors::kLidarPoints];
      for (uint8_t k = 0; k < Sensors::kLidarPoints; k++) sc[k] = sensors.lidarCm(k);
      LineFeat::Cfg cc; cc.rMax = 4.0f; cc.minLen = 1.5f; cc.footTol = 25; cc.halfDeg = 50;
      cc.centerDeg = cc.footExp = 90;  const LineFeat::Line wl = LineFeat::extract(sc, cc);
      cc.centerDeg = cc.footExp = -90; const LineFeat::Line wr = LineFeat::extract(sc, cc);
      const LineFeat::Line& w = (wl.ok && (!wr.ok || wl.len >= wr.len)) ? wl : wr;
      String t = "no straight wall within 4 m";
      if (w.ok) {
        const float est = LIDAR_OFFSET_DEG + LineFeat::wrap180(w.foot - (&w == &wl ? 90.0f : -90.0f));
        if (isnan(calEst) || fabsf(est - calEst) > 5.0f) { calEst = est; calN = 1; }
        else { calN = calN < 40 ? calN + 1 : 40; calEst += (est - calEst) / calN; }
        t = String(&w == &wl ? "left" : "right") + " wall " + String(w.d, 2) + " m, " + String(w.len, 1) + " m long: true offset " +
            String(calEst, 1) + " -> for our maps " + String(calEst + MAP_LIDAR_ERR_DEG, 0) + " (" + String(calN) + " scans)";
      }
      hmi.showValue("lidar.cal", t);
    }
    hmi.showValue("imu.tap", "|a| " + String(sensors.imu().accNow(), 2) + " m/s2, rms " + String(sensors.imu().accRms(), 2) +
                  ", thr " + String(fmaxf(sensors.imu().tapMinMs2, sensors.imu().tapK * sensors.imu().accRms()), 1) +
                  ", double taps " + String(sensors.imu().doubleTaps()));
    hmi.showValue("start.align", String(mission.route.alignDeg(), 0) + " deg off, score " +
                  String(mission.route.alignScore(), 2) + " (2nd " + String(mission.route.alignSecond(), 2) + ")");
    if (mission.route.onLmkLeg()) {
      const LandmarkLeg& K = mission.route.lmk();
      hmi.showValue("route.lmk", String(mission.route.legMetres(), 1) + " m, phase " + String(K.phase() + 1) + "/" +
                    String(K.phases()) + " " + K.typeName() + ", d " + String(K.lastD(), 2) + " e " + String(K.lastE(), 2) +
                    " turn " + String(K.turnDeg(), 1) + " | " + mission.route.lmkWhy());
    }
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
