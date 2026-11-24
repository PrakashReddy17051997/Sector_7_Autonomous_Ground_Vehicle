#pragma once
// DisplayManager - the 0.91" SSD1306 OLED (128x32, I2C address 0x3C): up to three text lines.
// Wire.begin() must have been called before begin().
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

class DisplayManager {
public:
  DisplayManager();
  bool begin();  // false if the display could not be started
  // Shows up to three lines (nullptr or "" = skipped), each cut to 20 characters.
  void show(const char* line1, const char* line2 = nullptr, const char* line3 = nullptr);

private:
  void _printLine(uint8_t index, const char* text);

  Adafruit_SSD1306 _disp;
  bool _ready = false;
};
