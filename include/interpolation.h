#pragma once

#include <Arduino.h>

// Linear interpolator that mirrors the openDog `Interpolation` helper
// (originally based on the Ramp library). Each call to `go()` advances
// the ramp non-blockingly. If `input` changes, a new ramp is started
// from the current interpolated value to the new target.

class Interpolation {
public:
    float go(float input, uint32_t durationMs) {
        if (input != savedValue_) {
            startValue_ = currentValue_;
            targetValue_ = input;
            startMs_ = millis();
            durationMs_ = durationMs;
            active_ = (durationMs > 0);
            savedValue_ = input;
            if (!active_) {
                currentValue_ = input;
            }
        }
        update();
        return currentValue_;
    }

    float value() const { return currentValue_; }

private:
    void update() {
        if (!active_) return;
        uint32_t elapsed = millis() - startMs_;
        if (elapsed >= durationMs_) {
            currentValue_ = targetValue_;
            active_ = false;
            return;
        }
        float t = (float)elapsed / (float)durationMs_;
        currentValue_ = startValue_ + (targetValue_ - startValue_) * t;
    }

    float    startValue_   = 0.0f;
    float    targetValue_  = 0.0f;
    float    currentValue_ = 0.0f;
    float    savedValue_   = NAN;          // forces first input to start a ramp
    uint32_t startMs_      = 0;
    uint32_t durationMs_   = 0;
    bool     active_       = false;
};

// First-order low-pass filter, same shape as the original openDog `filter()`.
inline float lowPassFilter(float current, float prev, int weight) {
    return (prev + current * (float)weight) / (float)(weight + 1);
}

// Twelve global ramps (4 legs x { X, Y, Z, Yaw }) used by the IK.
extern Interpolation interpFRX, interpFRY, interpFRZ, interpFRT;
extern Interpolation interpFLX, interpFLY, interpFLZ, interpFLT;
extern Interpolation interpBRX, interpBRY, interpBRZ, interpBRT;
extern Interpolation interpBLX, interpBLY, interpBLZ, interpBLT;
