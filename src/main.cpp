#include <Arduino.h>

#include "config.h"
#include "gamepad.h"
#include "gait.h"
#include "interpolation.h"
#include "kinematics.h"
#include "serial_cmd.h"
#include "servo_driver.h"
#include "state.h"

// ============================================================
// GLOBAL STATE
// ============================================================
RobotState g_state;
uint32_t   g_previousInterpMillis = 0;
uint8_t    g_interpFlag           = 0;
uint32_t   g_currentMillis        = 0;

// ============================================================
// LOOP TIMING
// ============================================================
namespace {
uint32_t g_previousLoopMs = 0;
}

// ============================================================
// SETUP
// ============================================================
void setup() {
    Serial.begin(115200);
    // Give the USB CDC a moment to come up before printing.
    uint32_t t0 = millis();
    while (!Serial && (millis() - t0 < 1500)) { /* spin */ }

    DBG_PRINTLN(F("[boot] RoboDawg starting"));

    if (!servoDriver::begin()) {
        DBG_PRINTLN(F("[boot] FATAL: PCA9685 init failed"));
    }
    servoDriver::stopAll();

    serialCmd::begin();
    gamepadBegin();

    g_previousInterpMillis = millis();
    g_state.mode = MODE_IDLE;

    DBG_PRINTLN(F("[boot] ready"));
}

// ============================================================
// LOOP
// ============================================================
void loop() {
    // Bluepad32 must be serviced every loop iteration (Bluetooth stack).
    gamepadPoll();

    // Serial parsing runs every iteration so commands respond fast even
    // when the 100 Hz control tick isn't due.
    serialCmd::poll();

    g_currentMillis = millis();
    if (g_currentMillis - g_previousLoopMs < CONTROL_LOOP_PERIOD_MS) return;
    g_previousLoopMs = g_currentMillis;

    uint32_t tStart = micros();

    // ---- mode dispatch (legs) ----
    switch (g_state.mode) {
        case MODE_IDLE:
            // outputs already zeroed by STOP; nothing to do
            break;

        case MODE_POSE:
            gait::poseTick();
            break;

        case MODE_WALK:
            gait::tick();
            break;

        case MODE_SERVO:
            // calibration mode: legs are driven manually via SERVO command,
            // no automatic IK on this tick.
            break;
    }

    // ---- wheels always update from g_state.wheel[] ----
    // Setting any wheel via WHEEL command remains active until overwritten
    // or STOP is issued.
    auto wheelSpeed = [](uint8_t slot) {
        float s = g_state.wheel[slot];
        if ((g_state.wheelOutputMask & (1u << slot)) == 0) s = 0.0f;
        return s;
    };
    servoDriver::driveWheel(wheelChannel(LEG_FR), wheelSpeed(SLOT_FR));
    servoDriver::driveWheel(wheelChannel(LEG_FL), wheelSpeed(SLOT_FL));
    servoDriver::driveWheel(wheelChannel(LEG_BR), wheelSpeed(SLOT_BR));
    servoDriver::driveWheel(wheelChannel(LEG_BL), wheelSpeed(SLOT_BL));

    // ---- loop timing diagnostics ----
    uint32_t dt = micros() - tStart;
    g_state.loopMicrosLast = dt;
    if (dt > g_state.loopMicrosMax) g_state.loopMicrosMax = dt;
}
