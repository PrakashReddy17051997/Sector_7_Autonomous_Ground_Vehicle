#pragma once
// LidarScan - LD20 LiDAR reader with a stable distance map, one value per degree.
//
// Why a filter: the raw LD20 points are steady (2-60 mm), but about a third of all points are
// "no return" (distance 0). An older reader shrank the stored value on every such point, so the
// view jumped by up to 1 m although nothing moved. Here a "no return" point changes nothing;
// instead:
//   - a bin keeps its last 3 valid hits; its value is their median (single outliers drop out)
//   - a bin counts only with >= 2 hits younger than expireMs (removes ghost points in open
//     directions), otherwise it reads 0 = no return
//   - hits closer than minMm (the box itself) are ignored
// Angle convention: raw angle + 90, rounded to 1 degree -> front = bin 270, clockwise.
// LD20 protocol: 47-byte frames, header 0x54 0x2C, 12 points, CRC8 at the end; ~340 frames/s.
// Header-only (the whole class is in this file).
#include <Arduino.h>

class LidarScan {
public:
  struct Config {
    int rxPin = 27, txPin = 33;   // ESP32 RX <- LD20 TX, ESP32 TX -> LD20 RX (unused by the LD20)
    uint32_t baud = 230400;       // LD20 default
    int angleShiftDeg = 90;   // raw angle + 90 -> front = bin 270
    uint16_t minMm = 200;     // ignore the box itself
    uint16_t maxMm = 12000;   // LD20 range
    uint16_t expireMs = 500;  // LD20 turns at ~10 Hz -> ~5 revolutions
  };

  // Builds the CRC table, so feed() works even without begin().
  LidarScan() {
    for (int i = 0; i < 256; i++) {  // CRC8, poly 0x4D (LD20 protocol)
      uint8_t c = i;
      for (int b = 0; b < 8; b++) c = (c & 0x80) ? (c << 1) ^ 0x4D : c << 1;
      _crcTab[i] = c;
    }
  }

  // Opens the UART. Use Serial1 (Serial2 is the GPS).
  void begin(HardwareSerial& port, const Config& cfg) {
    _cfg = cfg;
    _port = &port;
    _port->setRxBufferSize(4096);  // ~23 kB/s: survive slow loop passes
    _port->begin(_cfg.baud, SERIAL_8N1, _cfg.rxPin, _cfg.txPin);
  }

  void update() {  // drain the UART, call every loop pass
    if (!_port) return;
    while (_port->available()) feed(_port->read());
  }

  // Frame parser, one byte at a time. Valid frames go into the distance map.
  void feed(uint8_t c) {
    if (_idx == 0 && c != 0x54) return;  // wait for header
    if (_idx == 1 && c != 0x2C) {        // second byte must be ver_len
      _idx = (c == 0x54) ? 1 : 0;        // ...unless this byte is the next header
      return;
    }
    _buf[_idx++] = c;
    if (_idx < sizeof(Frame)) return;
    _idx = 0;
    uint8_t crc = 0;
    for (size_t i = 0; i < sizeof(Frame) - 1; i++) crc = _crcTab[crc ^ _buf[i]];
    if (crc != _buf[sizeof(Frame) - 1]) { _crcErrors++; return; }
    _packets++;
    _handle(reinterpret_cast<const Frame*>(_buf));
  }

  // Distance of one bin in mm, 0 = no (reliable) return.
  uint16_t distanceMm(int bin) const {
    const Bin& b = _bins[((bin % 360) + 360) % 360];
    uint32_t now = millis();
    uint16_t v[3];
    int n = 0;
    for (int k = 0; k < 3; k++)
      if (b.mm[k] && now - b.ms[k] <= _cfg.expireMs) v[n++] = b.mm[k];
    if (n < 2) return 0;
    if (n == 2) return (v[0] + v[1]) / 2;
    if (v[0] > v[1]) { uint16_t t = v[0]; v[0] = v[1]; v[1] = t; }  // median of 3
    if (v[1] > v[2]) { v[1] = v[2]; }
    return v[0] > v[1] ? v[0] : v[1];
  }

  uint32_t packets() const { return _packets; }  // valid frames since start
  float crcErrorPct() const {                     // share of frames with a wrong checksum
    uint32_t t = _packets + _crcErrors;
    return t ? 100.0f * _crcErrors / t : 0;
  }

private:
  struct __attribute__((packed)) Pt { uint16_t dist; uint8_t inten; };
  struct __attribute__((packed)) Frame {
    uint8_t header, verLen; uint16_t speed, startAngle; Pt p[12]; uint16_t endAngle, ts; uint8_t crc;
  };
  struct Bin { uint16_t mm[3] = {0, 0, 0}; uint32_t ms[3] = {0, 0, 0}; uint8_t head = 0; };

  // Spreads the 12 points between start and end angle and stores the valid ones.
  void _handle(const Frame* f) {
    float a0 = f->startAngle / 100.0f, a1 = f->endAngle / 100.0f;
    if (a1 < a0) a1 += 360;
    float step = (a1 - a0) / 11;
    uint32_t now = millis();
    for (int k = 0; k < 12; k++) {
      uint16_t d = f->p[k].dist;
      if (d < _cfg.minMm || d >= _cfg.maxMm) continue;  // no return / too close: no change
      int bin = ((int)lroundf(a0 + step * k) + _cfg.angleShiftDeg) % 360;
      Bin& b = _bins[bin];
      b.mm[b.head] = d;
      b.ms[b.head] = now;
      b.head = (b.head + 1) % 3;
    }
  }

  Config _cfg;
  HardwareSerial* _port = nullptr;
  uint8_t _crcTab[256];
  uint8_t _buf[sizeof(Frame)];
  uint8_t _idx = 0;
  uint32_t _packets = 0, _crcErrors = 0;
  Bin _bins[360];
};
