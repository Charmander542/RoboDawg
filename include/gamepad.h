#pragma once

#include <Arduino.h>

// Bluetooth gamepad support via Bluepad32 (see README / platformio.ini).
// When the firmware is built with the stock Arduino-ESP32 core, these
// functions compile to no-ops. Use the `esp32dev_gamepad` environment
// for a core that ships <Bluepad32.h>.

void gamepadBegin();
void gamepadPoll();

bool gamepadIsConnected();
bool gamepadIsDumping();
void gamepadSetDump(bool on);
