#pragma once

#include <Arduino.h>
#include <BatteryMonitor.h>
#include <InputManager.h>

#ifdef XTEINK_X4_PRO
// Xteink X4 Pro (ESP32-S3). Pins from freeink-sdk's hardware-confirmed XTEINK_X4_PRO profile.
#define EPD_SCLK 12
#define EPD_MOSI 11
#define EPD_CS 13
#define EPD_DC 18
#define EPD_RST 14
#define EPD_BUSY 6
#define SPI_MISO -1  // panel is write-only; SD is native SDMMC, not SPI

#define PERIPH_RAIL_EN 1  // master rail for panel + SD: HIGH, held through deep sleep
#define TOUCH_RAIL_EN 2   // GT911 rail, active LOW
#define SD_RAIL_EN 5      // SD enable, active LOW (driven by SDCardManager)

#define X4PRO_I2C_SDA 39  // shared by GT911 touch, CW2017 gauge, RTC
#define X4PRO_I2C_SCL 38
#define CW2017_ADDR 0x63
#else
// Display SPI pins (custom pins for XteinkX4, not hardware SPI defaults)
#define EPD_SCLK 8   // SPI Clock
#define EPD_MOSI 10  // SPI MOSI (Master Out Slave In)
#define EPD_CS 21    // Chip Select
#define EPD_DC 4     // Data/Command
#define EPD_RST 5    // Reset
#define EPD_BUSY 6   // Busy

#define SPI_MISO 7  // SPI MISO, shared between SD card and display (Master In Slave Out)

#define BAT_GPIO0 0  // Battery voltage

#define UART0_RXD 20  // Used for USB connection detection

#endif

class HalGPIO {
#if CROSSPOINT_EMULATED == 0
  InputManager inputMgr;
#endif

 public:
  HalGPIO() = default;

  // Start button GPIO and setup SPI for screen and SD card
  void begin();

  // Button input methods
  void update();
  bool isPressed(uint8_t buttonIndex) const;
  bool wasPressed(uint8_t buttonIndex) const;
  bool wasAnyPressed() const;
  bool wasReleased(uint8_t buttonIndex) const;
  bool wasAnyReleased() const;
  unsigned long getHeldTime() const;

  // Setup wake up GPIO and enter deep sleep
  void startDeepSleep();

  // Get battery percentage (range 0-100)
  int getBatteryPercentage() const;

  // Check if USB is connected
  bool isUsbConnected() const;

  enum class WakeupReason { PowerButton, AfterFlash, AfterUSBPower, Other };

  WakeupReason getWakeupReason() const;

  // Sample input on its own task and queue every change, so presses made while the
  // loop is busy (render, SD save) are delivered in order instead of lost.
  void startInputTask();
  // Idle wait for the loop: returns early as soon as an input change is queued.
  void waitForInput(uint32_t timeoutMs);

  // Button indices
  static constexpr uint8_t BTN_BACK = 0;
  static constexpr uint8_t BTN_CONFIRM = 1;
  static constexpr uint8_t BTN_LEFT = 2;
  static constexpr uint8_t BTN_RIGHT = 3;
  static constexpr uint8_t BTN_UP = 4;
  static constexpr uint8_t BTN_DOWN = 5;
  static constexpr uint8_t BTN_POWER = 6;
};
