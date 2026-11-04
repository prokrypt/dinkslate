#include <HalGPIO.h>
#include <Preferences.h>
#include <SPI.h>
#include <esp_sleep.h>
#include <driver/gpio.h>
#ifdef XTEINK_X4_PRO
#include <Wire.h>
#include <driver/rtc_io.h>
#endif

#ifdef XTEINK_X4_PRO
// CW2017 fuel gauge, ported from freeink-sdk (register map + OEM init recovered from the
// X4 Pro stock firmware). The gauge reports 0% until a matching battery profile is resident.
namespace {
constexpr uint8_t CW2017_REG_VERSION = 0x00;    // 0xA0 while starting; running = 0x0D/0x0F
constexpr uint8_t CW2017_REG_SOC = 0x04;        // integer percent
constexpr uint8_t CW2017_REG_MODE = 0x08;       // soft-reset / sleep control
constexpr uint8_t CW2017_REG_SOC_ALERT = 0x0B;  // bit7 = profile loaded
constexpr uint8_t CW2017_REG_BATINFO = 0x10;    // 80-byte profile, 0x10..0x5F
constexpr uint8_t CW2017_MODE_NORMAL = 0x00;
constexpr uint8_t CW2017_UPDATE_FLAG = 0x80;

// OEM profile for the X4 Pro cell (stock app1 table).
constexpr uint8_t CW2017_BATINFO[80] = {
    0x50, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xaa, 0xbf, 0xb5, 0xb4, 0xa4, 0x9c, 0xeb, 0xe2,
    0xdf, 0xe5, 0xca, 0xa0, 0x8a, 0x62, 0x53, 0x48, 0x40, 0x3a, 0x32, 0xb1, 0xae, 0xda, 0xb5, 0xff,
    0xff, 0xff, 0xe8, 0xdb, 0xd9, 0xd6, 0xd4, 0xd2, 0xd0, 0xcb, 0xc3, 0xbc, 0x9e, 0x87, 0x7b, 0x71,
    0x72, 0x7c, 0x8c, 0xa3, 0xb7, 0xc8, 0xa5, 0x4f, 0x00, 0x00, 0xab, 0x02, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x64, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x23};

bool cw2017Read(uint8_t reg, uint8_t& out) {
  Wire.beginTransmission(CW2017_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0 ||
      Wire.requestFrom(static_cast<uint8_t>(CW2017_ADDR), static_cast<uint8_t>(1)) != 1) {
    return false;
  }
  out = Wire.read();
  return true;
}

bool cw2017Write(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(CW2017_ADDR);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission() == 0;
}

bool cw2017Running(uint8_t version) { return (version & 0xFD) == 0x0D; }

// Soft reset (MODE 0xF0 -> 0x30 -> 0x00), then wait for the engine and a valid SoC (~1 s + ~3 s max).
bool cw2017Restart() {
  for (const uint8_t m : {0xF0, 0x30, 0x00}) {
    if (!cw2017Write(CW2017_REG_MODE, m)) return false;
    delay(20);
  }
  uint8_t v = 0;
  int i = 0;
  while (!(cw2017Read(CW2017_REG_VERSION, v) && cw2017Running(v))) {
    if (++i >= 50) return false;
    delay(20);
  }
  for (i = 0; i < 30; ++i) {
    if (cw2017Read(CW2017_REG_SOC, v) && v <= 100) return true;
    delay(100);
  }
  return false;
}

// Upload the profile if missing/different and restart the gauge if it isn't running.
// Any I2C error returns false (caller retries) so a glitch never causes a partial rewrite.
bool cw2017EnsureProfile() {
  uint8_t mode = 0, version = 0, cfg = 0, b = 0;
  if (!cw2017Read(CW2017_REG_MODE, mode) || !cw2017Read(CW2017_REG_VERSION, version) ||
      !cw2017Read(CW2017_REG_SOC_ALERT, cfg)) {
    return false;
  }
  bool matches = (cfg & CW2017_UPDATE_FLAG) != 0;
  for (uint8_t i = 0; matches && i < sizeof(CW2017_BATINFO); ++i) {
    if (!cw2017Read(CW2017_REG_BATINFO + i, b)) return false;
    matches = b == CW2017_BATINFO[i];
  }
  if (!matches) {
    for (uint8_t i = 0; i < sizeof(CW2017_BATINFO); ++i) {
      if (!cw2017Write(CW2017_REG_BATINFO + i, CW2017_BATINFO[i])) return false;
    }
    if (!cw2017Write(CW2017_REG_SOC_ALERT, CW2017_UPDATE_FLAG)) return false;
    delay(20);
    return cw2017Restart();
  }
  if (mode != CW2017_MODE_NORMAL || !cw2017Running(version)) return cw2017Restart();
  return cw2017Read(CW2017_REG_SOC, b) && (b <= 100 || cw2017Restart());
}
}  // namespace
#endif

void HalGPIO::begin() {
#ifdef XTEINK_X4_PRO
  // Release pad holds a previous deep sleep (ours or the other OTA app's) left armed,
  // then raise the master peripheral rail before touching the panel or SD.
  for (const int pin : {PERIPH_RAIL_EN, TOUCH_RAIL_EN, SD_RAIL_EN, EPD_RST}) {
    gpio_hold_dis(static_cast<gpio_num_t>(pin));
  }
  pinMode(PERIPH_RAIL_EN, OUTPUT);
  digitalWrite(PERIPH_RAIL_EN, HIGH);
  // An ext0 wake leaves the power key under RTC IO control; hand it back to digital GPIO.
  rtc_gpio_deinit(static_cast<gpio_num_t>(InputManager::POWER_BUTTON_PIN));
  inputMgr.begin();
  SPI.begin(EPD_SCLK, SPI_MISO, EPD_MOSI, EPD_CS);
#else
  inputMgr.begin();
  SPI.begin(EPD_SCLK, SPI_MISO, EPD_MOSI, EPD_CS);
  // BAT_GPIO0 is configured for ADC via adc1_config_channel_atten in InputManager::begin()
  // — do NOT call pinMode() here as it reconfigures the pin as digital input in dual framework
  pinMode(UART0_RXD, INPUT);
#endif
}

void HalGPIO::update() { inputMgr.update(); }

bool HalGPIO::isPressed(uint8_t buttonIndex) const { return inputMgr.isPressed(buttonIndex); }

bool HalGPIO::wasPressed(uint8_t buttonIndex) const { return inputMgr.wasPressed(buttonIndex); }

bool HalGPIO::wasAnyPressed() const { return inputMgr.wasAnyPressed(); }

bool HalGPIO::wasReleased(uint8_t buttonIndex) const { return inputMgr.wasReleased(buttonIndex); }

bool HalGPIO::wasAnyReleased() const { return inputMgr.wasAnyReleased(); }

unsigned long HalGPIO::getHeldTime() const { return inputMgr.getHeldTime(); }

void HalGPIO::startDeepSleep() {
  // Ensure that the power button has been released to avoid immediately turning back on if you're holding it
  while (inputMgr.isPressed(BTN_POWER)) {
    delay(50);
    inputMgr.update();
  }
#ifdef XTEINK_X4_PRO
  // Keep the master rail HIGH (else the next press cold-boots), cut touch and SD.
  const auto hold = [](int pin, int level) {
    gpio_hold_dis(static_cast<gpio_num_t>(pin));
    pinMode(pin, OUTPUT);
    digitalWrite(pin, level);
    gpio_hold_en(static_cast<gpio_num_t>(pin));
  };
  hold(PERIPH_RAIL_EN, HIGH);
  hold(TOUCH_RAIL_EN, HIGH);
  hold(SD_RAIL_EN, HIGH);
  hold(EPD_RST, HIGH);  // panel rail stays up; keep the controller out of reset
  gpio_deep_sleep_hold_en();
  // S3 has no deep-sleep GPIO wake; GPIO3 is an RTC pad, so use ext0.
  rtc_gpio_pullup_en(static_cast<gpio_num_t>(InputManager::POWER_BUTTON_PIN));
  rtc_gpio_pulldown_dis(static_cast<gpio_num_t>(InputManager::POWER_BUTTON_PIN));
  esp_sleep_enable_ext0_wakeup(static_cast<gpio_num_t>(InputManager::POWER_BUTTON_PIN), 0);
#else
  // Arm the wakeup trigger *after* the button is released
  esp_deep_sleep_enable_gpio_wakeup(1ULL << InputManager::POWER_BUTTON_PIN, ESP_GPIO_WAKEUP_GPIO_LOW);
#endif
  // Enter Deep Sleep
  esp_deep_sleep_start();
}

int HalGPIO::getBatteryPercentage() const {
#ifdef XTEINK_X4_PRO
  static bool profileReady = false;
  static int cachedPct = -1;
  static unsigned long nextReadMs = 0;
  if (static_cast<long>(millis() - nextReadMs) >= 0) {
    nextReadMs = millis() + 30000;
    // A gauge that lost power reads 0% until its profile is reloaded; re-check on any bad read.
    if (!profileReady) profileReady = cw2017EnsureProfile();
    uint8_t mode = 0, version = 0, soc = 0;
    if (profileReady && cw2017Read(CW2017_REG_MODE, mode) && mode == CW2017_MODE_NORMAL &&
        cw2017Read(CW2017_REG_VERSION, version) && cw2017Running(version) && cw2017Read(CW2017_REG_SOC, soc) &&
        soc <= 100) {
      cachedPct = soc;
    } else {
      profileReady = false;
      nextReadMs = millis() + 1000;  // retry sooner, but never every frame
    }
  }
  return cachedPct < 0 ? 0 : cachedPct;
#else
  static const BatteryMonitor battery = BatteryMonitor(BAT_GPIO0);
  static int cachedPct = -1;
  static float smoothedMv = -1.0f;
  static unsigned long lastReadMs = 0;
  static bool adcSettled = false;

  // On first call, load the last persisted reading so we don't show a stale default
  if (cachedPct < 0) {
    Preferences prefs;
    prefs.begin("battery", true);  // read-only
    cachedPct = prefs.getInt("pct", -1);
    prefs.end();
  }

  unsigned long now = millis();

  const bool usbCharging = isUsbConnected();

  // ADC reads high for ~2 minutes after boot/wake — trust NVS cache during settling.
  // Skip this window when charging: voltage is actively changing and we want real readings.
  if (!adcSettled) {
    if (!usbCharging && now < 120000) {
      // Still settling — return NVS cached value if we have one
      if (cachedPct >= 0) return cachedPct;
      // No NVS value at all — fall through to read ADC (better than showing nothing)
    }
    adcSettled = true;
  }

  // Battery voltage changes on a timescale of minutes — no need to read every frame
  if (cachedPct < 0 || (now - lastReadMs) >= 30000) {
    float rawMv = static_cast<float>(battery.readMillivolts());

    // EMA smoothing on millivolts (before the nonlinear polynomial).
    // Alpha=0.3 means ~70% weight on history — takes ~5 reads (~2.5 min) to converge,
    // which rejects brief voltage spikes from charging cycles / SPI / BLE noise.
    // When charging, use alpha=1.0 (no smoothing) so voltage tracks in real time.
    if (smoothedMv < 0) {
      smoothedMv = rawMv;  // seed with first real reading
    } else if (usbCharging) {
      smoothedMv = rawMv;  // no smoothing while charging
    } else {
      smoothedMv = 0.3f * rawMv + 0.7f * smoothedMv;
    }

    int newPct = BatteryMonitor::percentageFromMillivolts(static_cast<uint16_t>(smoothedMv));

    // Rate-limit drops only: max 2% decrease per read cycle (every 30s) on battery.
    // Rising is uncapped so the display catches up quickly after charging.
    // When charging via USB, drops are also uncapped (voltage actively rising anyway).
    if (cachedPct >= 0) {
      const int maxDrop = usbCharging ? 20 : 2;
      if (newPct < cachedPct - maxDrop) newPct = cachedPct - maxDrop;
    }

    if (newPct != cachedPct) {
      Preferences prefs;
      prefs.begin("battery", false);
      prefs.putInt("pct", newPct);
      prefs.end();
    }
    cachedPct = newPct;
    lastReadMs = now;
  }
  return cachedPct;
#endif
}

bool HalGPIO::isUsbConnected() const {
#ifdef XTEINK_X4_PRO
  return false;  // shortcut: X4 Pro VBUS-detect pin is unconfirmed (GPIO10 candidate is the touch INT)
#else
  // U0RXD/GPIO20 reads HIGH when USB is connected
  return digitalRead(UART0_RXD) == HIGH;
#endif
}

HalGPIO::WakeupReason HalGPIO::getWakeupReason() const {
  const bool usbConnected = isUsbConnected();
  const auto wakeupCause = esp_sleep_get_wakeup_cause();
  const auto resetReason = esp_reset_reason();

  if ((wakeupCause == ESP_SLEEP_WAKEUP_UNDEFINED && resetReason == ESP_RST_POWERON && !usbConnected) ||
      ((wakeupCause == ESP_SLEEP_WAKEUP_GPIO || wakeupCause == ESP_SLEEP_WAKEUP_EXT0) &&
       resetReason == ESP_RST_DEEPSLEEP && usbConnected)) {
    return WakeupReason::PowerButton;
  }
  if (wakeupCause == ESP_SLEEP_WAKEUP_UNDEFINED && resetReason == ESP_RST_UNKNOWN && usbConnected) {
    return WakeupReason::AfterFlash;
  }
  if (wakeupCause == ESP_SLEEP_WAKEUP_UNDEFINED && resetReason == ESP_RST_POWERON && usbConnected) {
    return WakeupReason::AfterUSBPower;
  }
  return WakeupReason::Other;
}
