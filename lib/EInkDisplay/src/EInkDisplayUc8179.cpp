// UltraChip UC8179 path for the Xteink X4 Pro. Most X4 Pro batches ship a UC8179 in
// place of the SSD1677. Minimal B/W port of freeink-sdk's Uc8179Driver (MIT, FreeInk
// SDK Contributors; recovered from the X4 Pro OEM firmware): factory OTP waveforms,
// KW mode, DTM1 = OLD plane, DTM2 = NEW plane, panel addressed as 800x600.

#ifdef XTEINK_X4_PRO

#include <EInkDisplay.h>
#include <driver/gpio.h>

namespace {
constexpr uint8_t CMD_PSR = 0x00;
constexpr uint8_t CMD_POF = 0x02;
constexpr uint8_t CMD_PFS = 0x03;
constexpr uint8_t CMD_PON = 0x04;
constexpr uint8_t CMD_BTST = 0x06;
constexpr uint8_t CMD_DTM1 = 0x10;
constexpr uint8_t CMD_DRF = 0x12;
constexpr uint8_t CMD_DTM2 = 0x13;
constexpr uint8_t CMD_CDI = 0x50;
constexpr uint8_t CMD_TRES = 0x61;
constexpr uint8_t CMD_GSST = 0x65;
constexpr uint8_t CMD_FLG = 0x71;
constexpr uint8_t CMD_VER = 0x70;
constexpr uint8_t CMD_PTIN = 0x91;
constexpr uint8_t CMD_PTOUT = 0x92;
constexpr uint8_t CMD_CCSET = 0xE0;
constexpr uint8_t CMD_GATE_SCAN = 0xE1;
constexpr uint8_t CMD_PWS = 0xE3;
constexpr uint8_t CMD_TSSET = 0xE5;

// OEM values (freeink-sdk uc8179DefaultConfig).
constexpr uint8_t PSR0 = 0x3F;  // REG=1 + SHL; refreshes send PSR0 & 0xDF (OTP LUTs)
constexpr uint8_t PSR1 = 0x0A;
constexpr uint8_t PFS = 0x20;
constexpr uint8_t BTST[4] = {0x25, 0x25, 0x3C, 0x25};
constexpr uint8_t GATE_SCAN = 0x02;
constexpr uint8_t CCSET = 0x02;
constexpr uint8_t TSSET_FULL = 0x1E;
constexpr uint8_t TSSET_FAST = 0x5A;  // frame-rate lever that makes the partial short
constexpr uint8_t CDI_ACTIVE = 0x29;
constexpr uint8_t CDI_IDLE = 0xA9;
constexpr uint8_t CDI_INTERVAL = 0x07;
constexpr uint8_t PWS = 0x22;
constexpr uint16_t TRES_HEIGHT = 600;  // OTP waveform is tuned for the 600-gate scan

constexpr uint16_t ROW_BYTES = EInkDisplay::DISPLAY_WIDTH / 8;
uint8_t rowChunk[ROW_BYTES * 20];

// --- Bit-banged controller probe (freeink-sdk XteinkDetect) ---------------------
// SSD1677 has no VER (0x70) / FLG (0x71) reads; a UC81xx drives SDA back with a
// non-floating VER and FLG bit0 (BUSY_N) set while idle.
struct ProbePins {
  int8_t sclk, sda, cs, dc, rst;
};

void probeWrite(const ProbePins& p, uint8_t b) {
  for (uint8_t i = 0; i < 8; i++) {
    digitalWrite(p.sda, (b & 0x80) ? HIGH : LOW);
    delayMicroseconds(1);
    digitalWrite(p.sclk, HIGH);
    delayMicroseconds(1);
    digitalWrite(p.sclk, LOW);
    b <<= 1;
  }
}

void probeRead(const ProbePins& p, uint8_t cmd, uint8_t* out, uint8_t len) {
  pinMode(p.sda, OUTPUT);
  digitalWrite(p.dc, LOW);
  digitalWrite(p.cs, LOW);
  delayMicroseconds(1);
  probeWrite(p, cmd);
  digitalWrite(p.dc, HIGH);
  pinMode(p.sda, INPUT_PULLUP);
  for (uint8_t n = 0; n < len; n++) {
    uint8_t b = 0;
    for (uint8_t i = 0; i < 8; i++) {
      delayMicroseconds(1);
      b = static_cast<uint8_t>((b << 1) | (digitalRead(p.sda) == HIGH ? 1 : 0));
      digitalWrite(p.sclk, HIGH);
      delayMicroseconds(1);
      digitalWrite(p.sclk, LOW);
    }
    out[n] = b;
  }
  digitalWrite(p.cs, HIGH);
  pinMode(p.sda, OUTPUT);
}

bool probePass(const ProbePins& p, uint8_t rstLowMs, uint8_t ver[5]) {
  pinMode(p.cs, OUTPUT);
  digitalWrite(p.cs, HIGH);
  pinMode(p.sclk, OUTPUT);
  digitalWrite(p.sclk, LOW);
  pinMode(p.dc, OUTPUT);
  pinMode(p.sda, OUTPUT);
  gpio_hold_dis(static_cast<gpio_num_t>(p.rst));
  pinMode(p.rst, OUTPUT);
  digitalWrite(p.rst, HIGH);
  delay(2);
  digitalWrite(p.rst, LOW);
  delay(rstLowMs);
  digitalWrite(p.rst, HIGH);
  delay(30);

  uint8_t flg = 0;
  probeRead(p, CMD_FLG, &flg, 1);
  probeRead(p, CMD_VER, ver, 5);
  bool floating = true;
  for (int i = 1; i < 5; i++) floating &= ver[i] == ver[0];
  return flg != 0x00 && flg != 0xFF && (flg & 0x01) && !floating;
}
}  // namespace

bool EInkDisplay::ucProbe() {
  const ProbePins p{_sclk, _mosi, _cs, _dc, _rst};
  uint8_t ver1[5] = {0}, ver2[5] = {0};
  const bool pass1 = probePass(p, 1, ver1);
  delay(2);
  const bool pass2 = probePass(p, pass1 ? 50 : 1, ver2);
  if (Serial) {
    Serial.printf("[%lu]   Panel probe VER=%02X %02X %02X %02X %02X\n", millis(), ver1[0], ver1[1], ver1[2], ver1[3],
                  ver1[4]);
  }
  // shortcut: UC8279 batches (LUT_VER 0x02/0x03/0x67-0x69) also take this UC8179 path;
  // port freeink-sdk's Uc8279Driver if one shows up.
  return pass1 && pass2 && memcmp(ver1, ver2, sizeof(ver1)) == 0;
}

void EInkDisplay::ucWaitIdle(const char* comment) {
  const unsigned long start = millis();
  while (digitalRead(_busy) == LOW) {  // BUSY_N: LOW while busy
    delay(1);
    if (millis() - start > 10000) {
      if (Serial) Serial.printf("[%lu]   Timeout waiting for busy%s\n", millis(), comment ? comment : "");
      return;
    }
  }
}

void EInkDisplay::ucInit() {
  digitalWrite(_rst, HIGH);
  delay(10);
  digitalWrite(_rst, LOW);
  delay(10);
  digitalWrite(_rst, HIGH);
  delay(60);

  sendCommand(CMD_PSR);
  sendData(PSR0);
  sendData(PSR1);
  sendCommand(CMD_TRES);  // HRES then VRES, 16-bit big-endian
  sendData(DISPLAY_WIDTH >> 8);
  sendData(DISPLAY_WIDTH & 0xFF);
  sendData(TRES_HEIGHT >> 8);
  sendData(TRES_HEIGHT & 0xFF);
  sendCommand(CMD_GSST);
  for (int i = 0; i < 4; i++) sendData(0x00);
  sendCommand(CMD_PFS);
  sendData(PFS);
  sendCommand(CMD_BTST);
  sendData(BTST, sizeof(BTST));
  sendCommand(CMD_GATE_SCAN);
  sendData(GATE_SCAN);
  sendCommand(CMD_PWS);
  sendData(PWS);

  isScreenOn = false;
  ucOldPlaneValid = false;
}

// Rows go out bottom-up (PSR SHL handles the horizontal direction), padded with white
// to the 600 addressed gates.
void EInkDisplay::ucStreamPlane(const uint8_t cmd, const uint8_t* fb) {
  constexpr uint16_t rowsPerChunk = sizeof(rowChunk) / ROW_BYTES;
  sendCommand(cmd);
  uint16_t pending = 0;
  for (uint16_t i = 0; i < TRES_HEIGHT; i++) {
    uint8_t* dst = rowChunk + pending * ROW_BYTES;
    if (fb && i < DISPLAY_HEIGHT) {
      memcpy(dst, fb + static_cast<uint32_t>(DISPLAY_HEIGHT - 1 - i) * ROW_BYTES, ROW_BYTES);
    } else {
      memset(dst, 0xFF, ROW_BYTES);
    }
    if (++pending == rowsPerChunk) {
      sendData(rowChunk, pending * ROW_BYTES);
      pending = 0;
    }
  }
  if (pending) sendData(rowChunk, pending * ROW_BYTES);
}

// Fast = OTP differential partial against the OLD plane (needs a valid one); Full and
// Half = the clearing OTP waveform from white.
void EInkDisplay::ucStartRefresh(const RefreshMode mode, const bool turnOffScreen) {
  const bool fast = mode == FAST_REFRESH && ucOldPlaneValid;

  ucStreamPlane(CMD_DTM2, frameBuffer);
  if (!fast) ucStreamPlane(CMD_DTM1, nullptr);

  sendCommand(CMD_CDI);
  sendData(CDI_ACTIVE);
  sendData(CDI_INTERVAL);
  sendCommand(CMD_CCSET);
  sendData(CCSET);
  sendCommand(CMD_TSSET);
  sendData(fast ? TSSET_FAST : TSSET_FULL);
  sendCommand(CMD_PSR);
  sendData(PSR0 & 0xDF);  // REG cleared: OTP waveforms
  sendData(PSR1);
  if (fast) {
    // Without these the OTP waveform runs its full frame count.
    sendCommand(CMD_PFS);
    sendData(PFS);
    sendCommand(CMD_GATE_SCAN);
    sendData(GATE_SCAN);
  }
  if (!isScreenOn) {
    sendCommand(CMD_PON);
    ucWaitIdle(" UC8179 PON");
    isScreenOn = true;
  }
  if (fast) sendCommand(CMD_PTIN);  // whole-panel partial
  sendCommand(CMD_DRF);
  // Let BUSY_N drop so a poll can't see the pre-refresh idle level.
  const unsigned long t0 = millis();
  while (digitalRead(_busy) == HIGH && millis() - t0 < 50) delay(1);

  ucPartial = fast;
  ucTurnOff = turnOffScreen;
}

void EInkDisplay::ucFinishRefresh() {
  if (ucPartial) sendCommand(CMD_PTOUT);
  sendCommand(CMD_CDI);
  sendData(CDI_IDLE);
  sendData(CDI_INTERVAL);
  // OLD plane = what is on screen now, so the next fast refresh clears erased pixels.
  ucStreamPlane(CMD_DTM1, frameBuffer);
  ucOldPlaneValid = true;
  if (ucTurnOff) {
    sendCommand(CMD_POF);
    ucWaitIdle(" UC8179 POF");
    isScreenOn = false;
  }
}

#endif  // XTEINK_X4_PRO
