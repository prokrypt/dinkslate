#pragma once

#include "config.h"

void inputSetup();
void enqueueKeyEvent(uint8_t keyCode, uint8_t modifiers, bool pressed);
int processAllInput();
bool discardKeyEvents();
char hidToAscii(uint8_t hid, uint8_t modifiers);
bool inputRepeatArmed();  // a held BLE key is waiting to repeat: keep the loop ticking
