#include "DisplayManager.h"

static const uint8_t kWidth = 128, kHeight = 32;
static const uint8_t kAddr = 0x3C;
static const uint8_t kMaxChars = 20;   // 6 px per character at text size 1
static const uint8_t kLineStep = 12;   // 8 px text + 4 px gap -> 3 lines on 32 px

DisplayManager::DisplayManager() : _disp(kWidth, kHeight, &Wire, -1) {}

bool DisplayManager::begin() {
  _ready = _disp.begin(SSD1306_SWITCHCAPVCC, kAddr);
  if (!_ready) return false;
  _disp.setTextColor(SSD1306_WHITE);
  _disp.setTextSize(1);
  _disp.clearDisplay();
  _disp.display();
  return true;
}

void DisplayManager::show(const char* line1, const char* line2, const char* line3) {
  if (!_ready) return;
  _disp.clearDisplay();
  uint8_t row = 0;
  for (const char* line : {line1, line2, line3}) {
    if (line && line[0]) _printLine(row++, line);  // empty lines do not leave a gap
  }
  _disp.display();
}

void DisplayManager::_printLine(uint8_t index, const char* text) {
  char buf[kMaxChars + 1];
  strncpy(buf, text, kMaxChars);
  buf[kMaxChars] = '\0';
  _disp.setCursor(0, index * kLineStep);
  _disp.print(buf);
}
