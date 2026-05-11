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
// PCA9685 channels:
//   0..3   hip servos       (FR, FL, BR, BL)
//   4..7   thigh servos     (FR, FL, BR, BL)
//   8..11  shin/knee servos (FR, FL, BR, BL)
//   12..15 wheel ESCs       (FR, FL, BR, BL)

#define NUM_SERVO_CHANNELS 16

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

constexpr uint8_t hipChannel  (uint8_t leg) { return  0 + legToSlot(leg); }
constexpr uint8_t thighChannel(uint8_t leg) { return  4 + legToSlot(leg); }
constexpr uint8_t shinChannel (uint8_t leg) { return  8 + legToSlot(leg); }
constexpr uint8_t wheelChannel(uint8_t leg) { return 12 + legToSlot(leg); }

// ============================================================
// SERVO / ESC PULSE LIMITS
// ============================================================
// Default servo pulse range. Override per channel in the calibration
// table loaded from NVS.
#define SERVO_DEFAULT_MIN_US    500
#define SERVO_DEFAULT_MAX_US   2500
#define SERVO_DEFAULT_MIN_DEG    0.0f
#define SERVO_DEFAULT_MAX_DEG  180.0f

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

    // Neutral servo angles (degrees, 0..180) corresponding to zero
    // joint angle. Most setups use 90 (mid-travel).
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
