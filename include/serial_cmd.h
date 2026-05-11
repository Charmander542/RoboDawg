#pragma once

#include <Arduino.h>

namespace serialCmd {

// Initialise the serial command parser.
void begin();

// Pull any available bytes off Serial and process complete lines.
// Must be called every loop iteration. Non-blocking.
void poll();

}  // namespace serialCmd
