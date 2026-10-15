#include <HalGPIO.h>
#include <Preferences.h>
#include <SPI.h>
#include <esp_sleep.h>
#include <driver/gpio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>
#ifdef XTEINK_X4_PRO
#include <Wire.h>
#include <driver/rtc_io.h>
#endif

namespace {
// Debounced input states sampled by the input task, oldest first.
QueueHandle_t inputQueue = nullptr;
constexpr int INPUT_QUEUE_LEN = 32;
}  // namespace

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

void HalGPIO::update() {
  if (!inputQueue) {
    inputMgr.update();
    return;
  }
  // One queued state per loop pass, so a press and release made while the loop was
  // busy still show up as two separate edges.
  uint8_t state;
  if (xQueueReceive(inputQueue, &state, 0) == pdTRUE) {
    inputMgr.applyState(state);
  } else {
    inputMgr.applyState(inputMgr.getCurrentState());
  }
}

static void inputTaskMain(void* arg) {
  auto* mgr = static_cast<InputManager*>(arg);
  uint8_t sent = mgr->getCurrentState();
  uint8_t last = sent;
  unsigned long changedAt = millis();
  for (;;) {
    const uint8_t state = mgr->getState();
    if (state != last) {
      last = state;
      changedAt = millis();
    }
    if (state != sent && millis() - changedAt >= 5) {  // same debounce as InputManager
      if (xQueueSend(inputQueue, &state, 0) == pdTRUE) sent = state;
    }
    vTaskDelay(pdMS_TO_TICKS(state || state != sent ? 5 : 15));
  }
}

void HalGPIO::startInputTask() {
  if (inputQueue) return;
  inputQueue = xQueueCreate(INPUT_QUEUE_LEN, sizeof(uint8_t));
  if (!inputQueue) return;
  // Above the Arduino loop task (1), so sampling is never starved by loop work.
  if (xTaskCreate(inputTaskMain, "input", 3072, &inputMgr, 2, nullptr) != pdPASS) {
    vQueueDelete(inputQueue);
    inputQueue = nullptr;  // fall back to sampling on the loop
  }
}

void HalGPIO::waitForInput(const uint32_t timeoutMs) {
  uint8_t state;
  if (!inputQueue) {
    delay(timeoutMs);
    return;
  }
  xQueuePeek(inputQueue, &state, pdMS_TO_TICKS(timeoutMs));
}

bool HalGPIO::isPressed(uint8_t buttonIndex) const { return inputMgr.isPressed(buttonIndex); }

bool HalGPIO::wasPressed(uint8_t buttonIndex) const { return inputMgr.wasPressed(buttonIndex); }

bool HalGPIO::wasAnyPressed() const { return inputMgr.wasAnyPressed(); }

bool HalGPIO::wasReleased(uint8_t buttonIndex) const { return inputMgr.wasReleased(buttonIndex); }

bool HalGPIO::wasAnyReleased() const { return inputMgr.wasAnyReleased(); }

unsigned long HalGPIO::getHeldTime() const { return inputMgr.getHeldTime(); }

void HalGPIO::startDeepSleep() {
  // Ensure that the power button has been released to avoid immediately turning back on if you're holding it.
  // Read the pin directly: the input task may own InputManager's sampling.
  while (digitalRead(InputManager::POWER_BUTTON_PIN) == LOW) {
    delay(50);
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
  // shortcut: reads the CW2017 SoC register only and relies on the battery profile the
  // stock/CrossPoint firmware already loaded into the gauge; port freeink-sdk's BATINFO
  // upload if readings are wrong after the battery was fully disconnected.
  static int cachedPct = -1;
  static unsigned long lastReadMs = 0;
  if (cachedPct < 0 || millis() - lastReadMs >= 30000) {
    lastReadMs = millis();
    Wire.beginTransmission(CW2017_ADDR);
    Wire.write(0x04);
    if (Wire.endTransmission(false) == 0 && Wire.requestFrom(static_cast<uint8_t>(CW2017_ADDR), static_cast<uint8_t>(1)) == 1) {
      const int soc = Wire.read();
      if (soc <= 100) cachedPct = soc;
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
