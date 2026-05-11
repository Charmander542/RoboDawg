#pragma once

#include <Arduino.h>

namespace gait {

// Reset the gait state machine (call when switching into walk mode).
void reset();

// Tick the gait state machine. Reads g_state.walkX / walkY / walkYaw and
// drives the four legs via the kinematics() function. Non-blocking.
void tick();

// Hold a static pose. Reads g_state.poseRoll/Pitch/Yaw/Height and drives
// the four legs via kinematics() with no stepping. Non-blocking.
void poseTick();

}  // namespace gait
