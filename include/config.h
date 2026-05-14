#pragma once

#include <Arduino.h>

// ============================================================
// HARDWARE CONFIGURATION
// ============================================================

// I2C bus pins for ESP32 (default I2C0)
#ifndef PIN_SDA
#define PIN_SDA 21
#endif
#ifndef PIN_SCL
#define PIN_SCL 22
#endif

// PCA9685 address (default 0x40 with all address jumpers open)
#define PCA9685_ADDR 0x40

// PWM frequency for the PCA9685. 50 Hz is standard for both hobby
// servos and ESCs that expect a 1000-2000 us pulse on a 20 ms frame.
#define PCA9685_FREQ_HZ 50

// 24 MHz internal oscillator on the PCA9685
#define PCA9685_OSC_FREQ 25000000UL

// ============================================================
// CHANNEL MAP
// ============================================================
// Body layout (viewed from above):
//   front-right (FR)   front-left (FL)
//   back-right  (BR)   back-left  (BL)
//
// PCA9685 channels — one contiguous block per leg (hip, thigh, shin, wheel):
//   FR: 0 hip, 1 thigh, 2 shin, 3 wheel ESC
//   FL: 4 hip, 5 thigh, 6 shin, 7 wheel ESC
//   BR: 8 hip, 9 thigh, 10 shin, 11 wheel ESC
//   BL: 12 hip, 13 thigh, 14 shin, 15 wheel ESC

#define NUM_SERVO_CHANNELS 16
#define CHANNELS_PER_LEG 4u

// Leg indices used by the kinematics function (kept compatible with the
// original openDog code): 1 = FR, 2 = FL, 3 = BL, 4 = BR.
enum Leg : uint8_t {
    LEG_FR = 1,
    LEG_FL = 2,
    LEG_BL = 3,
    LEG_BR = 4,
};

// Order within each PCA9685 channel block (FR, FL, BR, BL).
enum LegSlot : uint8_t {
    SLOT_FR = 0,
    SLOT_FL = 1,
    SLOT_BR = 2,
    SLOT_BL = 3,
};

// Convert openDog leg id (1..4) to channel-block slot (0..3).
constexpr uint8_t legToSlot(uint8_t leg) {
    return (leg == LEG_FR) ? SLOT_FR
         : (leg == LEG_FL) ? SLOT_FL
         : (leg == LEG_BR) ? SLOT_BR
         : (leg == LEG_BL) ? SLOT_BL
         :                   SLOT_FR;
}

constexpr uint8_t legBlockBase(uint8_t leg) {
    return (uint8_t)(CHANNELS_PER_LEG * legToSlot(leg));
}
constexpr uint8_t hipChannel(uint8_t leg) { return (uint8_t)(legBlockBase(leg) + 0u); }
constexpr uint8_t thighChannel(uint8_t leg) { return (uint8_t)(legBlockBase(leg) + 1u); }
constexpr uint8_t shinChannel(uint8_t leg) { return (uint8_t)(legBlockBase(leg) + 2u); }
constexpr uint8_t wheelChannel(uint8_t leg) { return (uint8_t)(legBlockBase(leg) + 3u); }

// Wheel ESCs live on the 4th channel of each leg block (3, 7, 11, 15).
constexpr bool isWheelPwmChannel(uint8_t ch) {
    return ch < NUM_SERVO_CHANNELS && ((ch % CHANNELS_PER_LEG) == 3u);
}

// ============================================================
// SERVO / ESC PULSE LIMITS
// ============================================================
// Default servo pulse range. Override per channel in the calibration
// table loaded from NVS.
// Leg servos are modelled as 270° travel over this PWM span (many 270°
// digital servos still use ~500–2500 µs end-to-end; trim in NVS if not).
#define SERVO_DEFAULT_MIN_US    500
#define SERVO_DEFAULT_MAX_US   2500
#define SERVO_DEFAULT_MIN_DEG    0.0f
#define SERVO_DEFAULT_MAX_DEG  270.0f
// Commanded angle at bench “neutral” (mid of min..max).
#define SERVO_NEUTRAL_DEG  ((SERVO_DEFAULT_MIN_DEG + SERVO_DEFAULT_MAX_DEG) * 0.5f)

// ============================================================
// PER-JOINT HARDWARE LIMITS (servo deg, 0..270 frame)
// ============================================================
// Hip (leg 1 = FR reference): same **numeric** band every leg; direction
// differences are only in software (`gait.cpp` `kLegs`, IK `g_legMap`).
#define HIP_NEUTRAL_DEG   SERVO_NEUTRAL_DEG
#define HIP_OUT_DEG       20.0f   // +Δ from neutral (abduction “out”)
#define HIP_IN_DEG        15.0f   // −Δ from neutral (“in”)
#define HIP_SAFE_MIN_DEG  (HIP_NEUTRAL_DEG - HIP_IN_DEG)
#define HIP_SAFE_MAX_DEG  (HIP_NEUTRAL_DEG + HIP_OUT_DEG)

// Femur — **right legs** (FR, BR): measured on leg 1 (FR), 150°..200°.
// **Left legs** (FL, BL): mirrored mount → same *logical* motion uses the
// opposite direction along the band: **70°..115°** (see
// `femurRightFrameToServoForSlot()` in config.h). Tibia stays one band on all four legs.
#define FEMUR_SAFE_MIN_DEG_R  150.0f
#define FEMUR_SAFE_MAX_DEG_R  200.0f
#define FEMUR_SAFE_MIN_DEG_L   70.0f
#define FEMUR_SAFE_MAX_DEG_L  115.0f

// Gait / POSE logic that still thinks in “right femur” numbers uses R band.
#define FEMUR_SAFE_MIN_DEG  FEMUR_SAFE_MIN_DEG_R
#define FEMUR_SAFE_MAX_DEG  FEMUR_SAFE_MAX_DEG_R

// Tibia (all four legs): 100°..200° (same numbers; mirror only via `kLegs`
// tibiaMul if needed on the bench).
#define TIBIA_SAFE_MIN_DEG  100.0f
#define TIBIA_SAFE_MAX_DEG  200.0f

// Named body-height poses — **(femur, tibia) in right-leg (FR) numbers**;
// left femur is derived by mapping 150..200 → 115..70.
#define FEMUR_LOW_DEG    150.0f
#define TIBIA_LOW_DEG    200.0f
#define FEMUR_HIGH_DEG   160.0f
#define TIBIA_HIGH_DEG   100.0f
#define FEMUR_STAND_DEG  155.0f
#define TIBIA_STAND_DEG  150.0f

// Map a femur angle in **FR/BR (right) numbers** to the servo command for
// `slot` (FL/BL use the mirrored 70°..115° band, opposite traverse vs right).
inline bool legSlotIsLeftFemurMirror(uint8_t slot) {
    return slot == SLOT_FL || slot == SLOT_BL;
}

inline float femurRightFrameToServoForSlot(uint8_t slot, float femurDegRight) {
    femurDegRight = constrain(femurDegRight, FEMUR_SAFE_MIN_DEG_R, FEMUR_SAFE_MAX_DEG_R);
    if (!legSlotIsLeftFemurMirror(slot)) {
        return femurDegRight;
    }
    const float spanR = FEMUR_SAFE_MAX_DEG_R - FEMUR_SAFE_MIN_DEG_R;
    const float spanL = FEMUR_SAFE_MAX_DEG_L - FEMUR_SAFE_MIN_DEG_L;
    const float t     = (femurDegRight - FEMUR_SAFE_MIN_DEG_R) / spanR;
    return FEMUR_SAFE_MAX_DEG_L - t * spanL;
}

// ESC channel range (standard hobby ESC).
#define ESC_MIN_US             1000
#define ESC_NEUTRAL_US         1500
#define ESC_MAX_US             2000

// ============================================================
// IK / SERVO MAPPING
// ============================================================
// The IK function emits joint angles in degrees. To translate those
// into physical servo positions we use, per leg:
//   servo_deg = neutralDeg + direction * (jointAngleDeg + jointOffsetDeg)
//
// jointOffsetDeg comes from the original openDog code (-45 for shoulder,
// -90 for knee, 0 for hip). direction flips the sign for mirrored sides
// of the body.

struct LegServoMap {
    // Channels
    uint8_t hipCh;
    uint8_t thighCh;
    uint8_t shinCh;
    uint8_t wheelCh;

    // Neutral servo angles (degrees, mid of 0..270 leg servos) for IK
    // joint angle zero. Use SERVO_NEUTRAL_DEG (typically 135°).
    float   hipNeutral;
    float   thighNeutral;
    float   shinNeutral;

    // +1 / -1 to flip rotation direction per side / per joint
    float   hipDir;
    float   thighDir;
    float   shinDir;

    // For the original IK the shoulder angle is composed of
    // shoulderAngle1 +/- shoulderAngle2 (sign depends on side).
    // +1 = front legs, -1 = rear legs.
    float   shoulderCombineSign;
};

// ============================================================
// CONTROL LOOP
// ============================================================
#define CONTROL_LOOP_PERIOD_MS 10   // 100 Hz main loop
#define SERIAL_BUF_LEN         96   // max length of a single ASCII command

// ============================================================
// DEBUG PRINTING
// ============================================================
// We can't put `-DDEBUG_SERIAL=1` in platformio.ini directly because the
// Adafruit BusIO library claims that macro name for its own diagnostics
// and expects it to expand to a Serial object. Instead the build flag
// is ROBODAWG_DEBUG and config.h locally re-exports it as DEBUG_SERIAL.
#if defined(ROBODAWG_DEBUG) && ROBODAWG_DEBUG
    #ifndef DEBUG_SERIAL
        #define DEBUG_SERIAL 1
    #endif
#endif
#ifndef DEBUG_SERIAL
    #define DEBUG_SERIAL 0
#endif

#if DEBUG_SERIAL
    #define DBG_PRINT(x)     Serial.print(x)
    #define DBG_PRINTLN(x)   Serial.println(x)
    #define DBG_PRINTF(...)  Serial.printf(__VA_ARGS__)
#else
    #define DBG_PRINT(x)     do {} while (0)
    #define DBG_PRINTLN(x)   do {} while (0)
    #define DBG_PRINTF(...)  do {} while (0)
#endif
