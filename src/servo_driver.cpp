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
        // Always re-derive minAngle/maxAngle from current firmware defaults:
        // NVS effectively only persists per-channel pulse range and trim.
        // Without this, a saved blob from an old firmware (0..180°) would
        // silently remap every commanded angle on a 0..270° build — this
        // is what caused trims to "feel forgotten" after a firmware bump.
        for (uint8_t i = 0; i < NUM_SERVO_CHANNELS; ++i) {
            ServoCal d;
            defaultCal(d, isWheelPwmChannel(i));
            cal[i].minAngle = d.minAngle;
            cal[i].maxAngle = d.maxAngle;
        }
        DBG_PRINTLN(F("[servo] calibration loaded from NVS (angle range from firmware defaults)"));
    } else {
        for (uint8_t i = 0; i < NUM_SERVO_CHANNELS; ++i) {
            defaultCal(cal[i], isWheelPwmChannel(i));
        }
        DBG_PRINTLN(F("[servo] using default calibration"));
    }
    prefs.end();
}

// Per-joint hardware clamp. Applied *before* the cal-table angle clamp so
// a buggy gait, typo in a SERVO command, or stale NVS can never command
// any leg servo outside its mechanical envelope.
inline float clampPerJoint(uint8_t channel, float angleDeg) {
    if (isWheelPwmChannel(channel)) return angleDeg;
    const uint8_t joint = channel % CHANNELS_PER_LEG;  // 0=hip,1=femur,2=tibia
    switch (joint) {
        case 0: return constrain(angleDeg, HIP_SAFE_MIN_DEG,   HIP_SAFE_MAX_DEG);
        case 1: {
            const uint8_t slot = (uint8_t)(channel / CHANNELS_PER_LEG);
            const bool left = (slot == SLOT_FL || slot == SLOT_BL);
            if (left) {
                return constrain(angleDeg, FEMUR_SAFE_MIN_DEG_L, FEMUR_SAFE_MAX_DEG_L);
            }
            return constrain(angleDeg, FEMUR_SAFE_MIN_DEG_R, FEMUR_SAFE_MAX_DEG_R);
        }
        case 2: return constrain(angleDeg, TIBIA_SAFE_MIN_DEG, TIBIA_SAFE_MAX_DEG);
        default: return angleDeg;
    }
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

    // Hardware safety clamp first (per-joint envelope). Anything outside
    // these bounds — gait IK, calibration sliders, anything — is squashed
    // before being mapped to pulse microseconds.
    float a = clampPerJoint(channel, angleDegrees);
    a = constrain(a, c.minAngle, c.maxAngle);
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
    // Park into a safe stance: hips at neutral, femur/tibia at the "stand"
    // pose (well inside hardware envelope). The per-joint clamp in
    // driveServo() would otherwise rescue an out-of-range mid-of-cal-range
    // command anyway, but parking at STAND avoids ever asking for it.
    for (uint8_t ch = 0; ch < NUM_SERVO_CHANNELS; ++ch) {
        if (isWheelPwmChannel(ch)) {
            driveWheel(ch, 0.0f);
            continue;
        }
        const uint8_t joint = ch % CHANNELS_PER_LEG;
        float angle = HIP_NEUTRAL_DEG;
        if (joint == 1) {
            const uint8_t slot = (uint8_t)(ch / CHANNELS_PER_LEG);
            angle = femurRightFrameToServoForSlot(slot, FEMUR_STAND_DEG);
        }
        else if (joint == 2) angle = TIBIA_STAND_DEG;
        driveServo(ch, angle);
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
