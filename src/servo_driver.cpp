#include "servo_driver.h"

#include <Adafruit_PWMServoDriver.h>
#include <Preferences.h>
#include <Wire.h>

namespace {

Adafruit_PWMServoDriver pca(PCA9685_ADDR);
Preferences              prefs;

ServoCal cal[NUM_SERVO_CHANNELS];

constexpr const char* NVS_NAMESPACE = "robodawg";
constexpr const char* NVS_KEY_BLOB  = "servocal";

void defaultCal(ServoCal& c, bool isWheel) {
    if (isWheel) {
        c.minPulse   = ESC_MIN_US;
        c.maxPulse   = ESC_MAX_US;
        c.trimOffset = 0;
        c.minAngle   = -100.0f;
        c.maxAngle   =  100.0f;
    } else {
        c.minPulse   = SERVO_DEFAULT_MIN_US;
        c.maxPulse   = SERVO_DEFAULT_MAX_US;
        c.trimOffset = 0;
        c.minAngle   = SERVO_DEFAULT_MIN_DEG;
        c.maxAngle   = SERVO_DEFAULT_MAX_DEG;
    }
}


void loadFromNvs() {
    prefs.begin(NVS_NAMESPACE, /*readOnly=*/true);
    size_t expected = sizeof(cal);
    size_t got      = prefs.getBytesLength(NVS_KEY_BLOB);
    if (got == expected) {
        prefs.getBytes(NVS_KEY_BLOB, cal, expected);
        DBG_PRINTLN(F("[servo] calibration loaded from NVS"));
    } else {
        for (uint8_t i = 0; i < NUM_SERVO_CHANNELS; ++i) {
            defaultCal(cal[i], isWheelPwmChannel(i));
        }
        DBG_PRINTLN(F("[servo] using default calibration"));
    }
    prefs.end();
}

void writeToNvs() {
    prefs.begin(NVS_NAMESPACE, /*readOnly=*/false);
    prefs.putBytes(NVS_KEY_BLOB, cal, sizeof(cal));
    prefs.end();
    DBG_PRINTLN(F("[servo] calibration saved to NVS"));
}

// Convert a desired pulse width in µs to PCA9685 12-bit tick count.
// (Adafruit_PWMServoDriver already provides writeMicroseconds() that
// handles the maths; we use it directly.)
inline void writePulseUs(uint8_t channel, uint16_t pulseUs) {
    pca.writeMicroseconds(channel, pulseUs);
}

}  // namespace

namespace servoDriver {

bool begin() {
    Wire.begin(PIN_SDA, PIN_SCL);
    Wire.setClock(400000);

    if (!pca.begin()) {
        DBG_PRINTLN(F("[servo] ERROR: PCA9685 not found on I2C bus"));
        return false;
    }
    pca.setOscillatorFrequency(PCA9685_OSC_FREQ);
    pca.setPWMFreq(PCA9685_FREQ_HZ);

    loadFromNvs();
    return true;
}

void driveServo(uint8_t channel, float angleDegrees) {
    if (channel >= NUM_SERVO_CHANNELS) return;
    const ServoCal& c = cal[channel];

    float a = constrain(angleDegrees, c.minAngle, c.maxAngle);
    float t = (a - c.minAngle) / (c.maxAngle - c.minAngle);   // 0..1
    int32_t pulse = (int32_t)(c.minPulse + t * (c.maxPulse - c.minPulse));
    pulse += c.trimOffset;
    if (pulse < 0) pulse = 0;
    if (pulse > 4000) pulse = 4000;
    writePulseUs(channel, (uint16_t)pulse);
}

void driveWheel(uint8_t channel, float speed) {
    if (channel >= NUM_SERVO_CHANNELS) return;
    speed = constrain(speed, -100.0f, 100.0f);
    const ServoCal& c = cal[channel];
    // Map -100..+100 linearly to [minPulse..maxPulse] with neutral
    // at the midpoint. Per-channel trim still applies.
    int32_t mid    = ((int32_t)c.minPulse + (int32_t)c.maxPulse) / 2;
    int32_t halfRng = ((int32_t)c.maxPulse - (int32_t)c.minPulse) / 2;
    int32_t pulse  = mid + (int32_t)(speed * 0.01f * (float)halfRng);
    pulse += c.trimOffset;
    if (pulse < 0)    pulse = 0;
    if (pulse > 4000) pulse = 4000;
    writePulseUs(channel, (uint16_t)pulse);
}

void stopAll() {
    for (uint8_t ch = 0; ch < NUM_SERVO_CHANNELS; ++ch) {
        if (isWheelPwmChannel(ch)) {
            driveWheel(ch, 0.0f);
        } else {
            const ServoCal& c = cal[ch];
            // Send servos to mid-travel so the robot doesn't slam into a limit.
            float mid = 0.5f * (c.minAngle + c.maxAngle);
            driveServo(ch, mid);
        }
    }
}

const ServoCal& getCalibration(uint8_t channel) {
    static ServoCal dummy{};
    if (channel >= NUM_SERVO_CHANNELS) return dummy;
    return cal[channel];
}

void setTrimOffset(uint8_t channel, int16_t trimUs, bool persist) {
    if (channel >= NUM_SERVO_CHANNELS) return;
    cal[channel].trimOffset = trimUs;
    if (persist) writeToNvs();
}

void setPulseRange(uint8_t channel, uint16_t minUs, uint16_t maxUs, bool persist) {
    if (channel >= NUM_SERVO_CHANNELS) return;
    cal[channel].minPulse = minUs;
    cal[channel].maxPulse = maxUs;
    if (persist) writeToNvs();
}

void saveCalibration() {
    writeToNvs();
}

void resetCalibration(bool persist) {
    for (uint8_t i = 0; i < NUM_SERVO_CHANNELS; ++i) {
        defaultCal(cal[i], isWheelPwmChannel(i));
    }
    if (persist) writeToNvs();
}

}  // namespace servoDriver
