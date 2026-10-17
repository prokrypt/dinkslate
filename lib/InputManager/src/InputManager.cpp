#include "InputManager.h"
#include <driver/adc.h>
#ifdef XTEINK_X4_PRO
#include <Wire.h>
#include <driver/gpio.h>
#endif

// Recorded ADC values from real devices
// BACK CONF LEFT RGHT   UP DOWN
// 3597 2760 1530    6 2300    6
// 3470 2666 1480    6 2222    5
// 3470 2655 1470    3 2205    3

// Averages
// BACK CONF LEFT RGHT   UP DOWN
// 3512 2694 1493    5 2242    5

// Setup ranges, if ADC value is between value `i` and `i + 1`, button `i` is being pressed
// These ranges are based on real world values above, and are much more tolerant of different
// devices than a fixed threshold check
// These values are calculated by taking the midpoint of the pairs of averaged values above
const int InputManager::ADC_RANGES_1[] = {ADC_NO_BUTTON, 3100, 2090, 750, INT32_MIN};
const int InputManager::ADC_RANGES_2[] = {ADC_NO_BUTTON, 1120, INT32_MIN};
const char* InputManager::BUTTON_NAMES[] = {"Back", "Confirm", "Left", "Right", "Up", "Down", "Power"};

InputManager::InputManager()
    : currentState(0),
      lastState(0),
      pressedEvents(0),
      releasedEvents(0),
      lastDebounceTime(0),
      buttonPressStart(0),
      buttonPressFinish(0) {}

void InputManager::begin() {
#ifdef XTEINK_X4_PRO
  pinMode(BUTTON_UP_PIN, INPUT_PULLUP);
  pinMode(BUTTON_DOWN_PIN, INPUT_PULLUP);
  pinMode(POWER_BUTTON_PIN, INPUT_PULLUP);
  beginTouch();
#else
  // Configure ADC using ESP-IDF API directly to avoid GPIO reconfiguration
  // that Arduino's analogRead() triggers on every call in the dual framework
  adc1_config_width(ADC_WIDTH_BIT_12);
  adc1_config_channel_atten(ADC1_CHANNEL_0, ADC_ATTEN_DB_12);  // GPIO 0 (battery)
  adc1_config_channel_atten(ADC1_CHANNEL_1, ADC_ATTEN_DB_12);  // GPIO 1
  adc1_config_channel_atten(ADC1_CHANNEL_2, ADC_ATTEN_DB_12);  // GPIO 2

  // Power button is digital, keep using Arduino API
  pinMode(POWER_BUTTON_PIN, INPUT_PULLUP);
#endif
}

int InputManager::getButtonFromADC(const int adcValue, const int ranges[], const int numButtons) {
  for (int i = 0; i < numButtons; i++) {
    if (ranges[i + 1] < adcValue && adcValue <= ranges[i]) {
      return i;
    }
  }

  return -1;
}

uint8_t InputManager::getState() {
  uint8_t state = 0;

#ifdef XTEINK_X4_PRO
  if (digitalRead(BUTTON_UP_PIN) == LOW) state |= (1 << BTN_UP);
  if (digitalRead(BUTTON_DOWN_PIN) == LOW) state |= (1 << BTN_DOWN);
  if (digitalRead(POWER_BUTTON_PIN) == LOW) state |= (1 << BTN_POWER);
  return state | pollTouch();
#else

  // Read GPIO1 buttons using ESP-IDF API (no GPIO reconfiguration overhead)
  const int adcValue1 = adc1_get_raw(ADC1_CHANNEL_1);
  const int button1 = getButtonFromADC(adcValue1, ADC_RANGES_1, NUM_BUTTONS_1);
  if (button1 >= 0) {
    state |= (1 << button1);
  }

  // Read GPIO2 buttons using ESP-IDF API
  const int adcValue2 = adc1_get_raw(ADC1_CHANNEL_2);
  const int button2 = getButtonFromADC(adcValue2, ADC_RANGES_2, NUM_BUTTONS_2);
  if (button2 >= 0) {
    state |= (1 << (button2 + 4));
  }

  // Read power button (digital, active LOW)
  if (digitalRead(POWER_BUTTON_PIN) == LOW) {
    state |= (1 << BTN_POWER);
  }

  return state;
#endif
}

void InputManager::update() {
  const unsigned long currentTime = millis();
  uint8_t state = getState();

  // Always clear events first
  pressedEvents = 0;
  releasedEvents = 0;

#ifdef XTEINK_X4_PRO
  // GT911 frames are already filtered. Debouncing them would drop any tap seen in only
  // one loop pass (50 ms idle), so touch bits skip the debounce and only keys use it.
  constexpr uint8_t TOUCH_BITS = (1 << BTN_BACK) | (1 << BTN_CONFIRM);
  const uint8_t touch = state & TOUCH_BITS;
  state &= ~TOUCH_BITS;
#endif

  // Debounce
  if (state != lastState) {
    lastDebounceTime = currentTime;
    lastState = state;
  }
  if ((currentTime - lastDebounceTime) <= DEBOUNCE_DELAY) {
    state = currentState;  // keys not settled yet: keep their last debounced state
  }
#ifdef XTEINK_X4_PRO
  state = (state & ~TOUCH_BITS) | touch;
#endif

  if (state != currentState) {
    // Calculate pressed and released events
    pressedEvents = state & ~currentState;
    releasedEvents = currentState & ~state;

    // If pressing buttons and wasn't before, start recording time
    if (pressedEvents > 0 && currentState == 0) {
      buttonPressStart = currentTime;
    }

    // If releasing a button and no other buttons being pressed, record finish time
    if (releasedEvents > 0 && state == 0) {
      buttonPressFinish = currentTime;
    }

    currentState = state;
  }
}

bool InputManager::isPressed(const uint8_t buttonIndex) const {
  return currentState & (1 << buttonIndex);
}

bool InputManager::wasPressed(const uint8_t buttonIndex) const {
  return pressedEvents & (1 << buttonIndex);
}

bool InputManager::wasAnyPressed() const {
  return pressedEvents > 0;
}

bool InputManager::wasReleased(const uint8_t buttonIndex) const {
  return releasedEvents & (1 << buttonIndex);
}

bool InputManager::wasAnyReleased() const {
  return releasedEvents > 0;
}

unsigned long InputManager::getHeldTime() const {
  // Still hold a button
  if (currentState > 0) {
    return millis() - buttonPressStart;
  }

  return buttonPressFinish - buttonPressStart;
}

const char* InputManager::getButtonName(const uint8_t buttonIndex) {
  if (buttonIndex <= BTN_POWER) {
    return BUTTON_NAMES[buttonIndex];
  }
  return "Unknown";
}

bool InputManager::isPowerButtonPressed() const {
  return isPressed(BTN_POWER);
}

#ifdef XTEINK_X4_PRO
// --- GT911 touch (X4 Pro) ----------------------------------------------------
// Minimal port of freeink-sdk's GT911 path: the capacitive Home key is BACK and a
// tap anywhere on the panel is CONFIRM. Pins: SDA39/SCL38, INT10, RST4, rail GPIO2
// (active LOW). The controller self-loads its config on reset.
namespace {
constexpr int TOUCH_SDA = 39;
constexpr int TOUCH_SCL = 38;
constexpr int TOUCH_INT = 10;
constexpr int TOUCH_RST = 4;
constexpr int TOUCH_RAIL = 2;
constexpr unsigned long TOUCH_POLL_MS = 15;
}  // namespace

void InputManager::beginTouch() {
  gpio_hold_dis(static_cast<gpio_num_t>(TOUCH_RAIL));  // sleep path holds it off
  pinMode(TOUCH_RAIL, OUTPUT);
  digitalWrite(TOUCH_RAIL, LOW);
  delay(50);

  Wire.begin(TOUCH_SDA, TOUCH_SCL, 400000);
  Wire.setTimeOut(10);

  // Reset dance: INT level while RST rises selects the I2C address (LOW = 0x5D).
  for (const uint8_t intLevel : {LOW, HIGH}) {
    pinMode(TOUCH_INT, OUTPUT);
    pinMode(TOUCH_RST, OUTPUT);
    digitalWrite(TOUCH_RST, LOW);
    digitalWrite(TOUCH_INT, intLevel);
    delay(10);
    digitalWrite(TOUCH_RST, HIGH);
    delay(60);
    pinMode(TOUCH_INT, INPUT);
    delay(50);
    for (const uint8_t addr : {0x5D, 0x14}) {
      Wire.beginTransmission(addr);
      if (Wire.endTransmission() == 0) {
        touchAddr = addr;
        return;
      }
    }
  }
}

uint8_t InputManager::pollTouch() {
  if (!touchAddr || millis() - lastTouchPoll < TOUCH_POLL_MS) return touchState;
  lastTouchPoll = millis();

  Wire.beginTransmission(touchAddr);
  Wire.write(0x81);
  Wire.write(0x4E);
  if (Wire.endTransmission(false) != 0 || Wire.requestFrom(touchAddr, static_cast<uint8_t>(1)) != 1) {
    return touchState;  // transient bus error: keep the last state
  }
  const uint8_t status = Wire.read();
  if (!(status & 0x80)) return touchState;  // no fresh frame; a motionless hold keeps its state

  touchState = 0;
  if (status & 0x10) touchState |= (1 << BTN_BACK);     // capacitive Home key
  if (status & 0x0F) touchState |= (1 << BTN_CONFIRM);  // any contact on the panel

  Wire.beginTransmission(touchAddr);  // ack the frame
  Wire.write(0x81);
  Wire.write(0x4E);
  Wire.write(0x00);
  Wire.endTransmission();
  return touchState;
}
#endif
