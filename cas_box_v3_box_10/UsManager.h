#pragma once
// UsManager - the five HC-SR04 ultrasonic sensors of the box.
//
// A background task (core 0) measures the sensors one after another (no cross-talk) and keeps
// the last distance of each; the read functions only return these cached values and never block.
// "No echo" or "farther than maxDistanceCm" reads as maxDistanceCm.
// Echo lines go through a 1k/2k voltage divider (5 V sensor -> 3.3 V ESP32).
#include <Arduino.h>

class UsManager {
public:
  // GPIO pins (trigger / echo) as wired on the box PCB
  struct Pins {
    int trigFront = 32, echoFront = 25;
    int trigLeft = 4, echoLeft = 18;
    int trigRight = 13, echoRight = 19;
    int trigBack = 5, echoBack = 14;
    int trigFrontAngled = 23, echoFrontAngled = 26;
  };

  // Configures the pins and starts the measuring task.
  // timeoutUs: longest wait for an echo (17.5 ms ~ 3 m). Call once.
  void begin(int maxDistanceCm, uint32_t timeoutUs);

  // Last measured distances in cm (maxDistanceCm = nothing in range)
  int readFrontCm() const { return _frontCm; }
  int readLeftCm() const { return _leftCm; }
  int readRightCm() const { return _rightCm; }
  int readBackCm() const { return _backCm; }
  int readFrontAngledCm() const { return _frontAngledCm; }

private:
  int _measureCm(int trigPin, int echoPin);  // one blocking measurement (task only)
  static void _taskEntry(void* self);
  void _taskLoop();

  Pins _pins;
  int _maxDistanceCm = 300;
  uint32_t _timeoutUs = 17500;
  TaskHandle_t _task = nullptr;

  // written by the task, read by loop(); int reads/writes are atomic on the ESP32
  volatile int _frontCm = 0, _leftCm = 0, _rightCm = 0, _backCm = 0, _frontAngledCm = 0;
};
