#pragma once

#include <Arduino.h>
#include "config.h"

// Per-channel calibration. Stored in NVS so each servo can be zeroed
// individually without recompiling.
struct ServoCal {
    uint16_t minPulse;    // µs at minAngle
    uint16_t maxPulse;    // µs at maxAngle
    int16_t  trimOffset;  // µs added to every commanded pulse
    float    minAngle;    // ° corresponding to minPulse
    float    maxAngle;    // ° corresponding to maxPulse
};

namespace servoDriver {

// Initialise the PCA9685 and load calibration from NVS.
// Returns false if the PCA9685 cannot be reached on the I2C bus.
bool begin();

// Drive a single PCA9685 channel to a commanded angle in degrees.
// Range is clamped to the per-channel min/max angle.
void driveServo(uint8_t channel, float angleDegrees);

// Drive a wheel ESC on the given PCA9685 channel.
// speed is -100..+100 (negative = reverse, 0 = neutral, +100 = full fwd).
void driveWheel(uint8_t channel, float speed);

// Zero every channel (servos go to mid-range, ESCs to neutral 1500 µs).
void stopAll();

// Read / update the calibration table.
const ServoCal& getCalibration(uint8_t channel);
void setTrimOffset(uint8_t channel, int16_t trimUs, bool persist = true);
void setPulseRange(uint8_t channel, uint16_t minUs, uint16_t maxUs, bool persist = true);

// Persist the entire calibration table to NVS.
void saveCalibration();

// Reset calibration to defaults (does NOT persist unless persist=true).
void resetCalibration(bool persist = false);

}  // namespace servoDriver
