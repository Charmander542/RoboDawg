#pragma once

#include <Arduino.h>

namespace gait {

// Reset the gait state machine (call when switching into walk mode).
void reset();

// Tick the walk gait. Reads g_state.walkX / walkYaw and drives all four
// legs directly in servo-angle space (no IK) using a crawl gait pattern.
// Non-blocking; call from the 100 Hz control loop.
void tick();

// Hold a static pose. The height field of POSE is reinterpreted as a 0..1
// interpolation between the LOW and HIGH body poses (any value >1 is
// treated as legacy mm and clamped sensibly).
void poseTick();

// Park the dog in named, hardware-safe servo poses. Used by the LOW /
// HIGH / STAND serial commands. These bypass the gait/IK entirely and
// drive every leg to a fixed femur/tibia combination.
void driveLow();
void driveHigh();
void driveStand();

}  // namespace gait
