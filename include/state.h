#pragma once

#include <Arduino.h>
#include "config.h"

// Runtime control modes
enum RunMode : uint8_t {
    MODE_IDLE    = 0,   // outputs zeroed, no motion
    MODE_POSE    = 1,   // hold static pose (set via POSE command)
    MODE_WALK    = 2,   // gait state machine running with WALK targets
    MODE_SERVO   = 3,   // manual servo calibration (SERVO command)
};

struct RobotState {
    // --- mode ---
    RunMode mode = MODE_IDLE;

    // --- pose targets (POSE command) ---
    float poseRoll   = 0.0f;  // degrees
    float posePitch  = 0.0f;  // degrees
    float poseYaw    = 0.0f;  // degrees
    float poseHeight = 330.0f; // mm (hip-to-ground demand)

    // --- walk targets (WALK command) ---
    float walkX   = 0.0f;     // forward/back velocity scale
    float walkY   = 0.0f;     // strafe velocity scale
    float walkYaw = 0.0f;     // yaw rate scale

    // --- wheel speeds (WHEEL command), -100..+100 ---
    float wheel[4] = {0, 0, 0, 0};   // indexed by LegSlot (FR, FL, BR, BL)

    // --- output enables (bits 0..3 = FR, FL, BR, BL). Used by calibration
    // firmware and optional bench tests; normal firmware leaves these at 0x0F.
    uint8_t legOutputMask   = 0x0Fu;   // IK / leg servos
    uint8_t wheelOutputMask = 0x0Fu;   // wheel ESC channels

    // --- last computed joint angles per leg (degrees) ---
    // index 0..3 by LegSlot (FR, FL, BR, BL)
    float jointHip[4]   = {0, 0, 0, 0};
    float jointThigh[4] = {0, 0, 0, 0};
    float jointShin[4]  = {0, 0, 0, 0};

    // --- timing diagnostics ---
    uint32_t loopMicrosLast = 0;
    uint32_t loopMicrosMax  = 0;

    // --- Bluetooth gamepad (Bluepad32 build only) ---
    bool gamepadConnected = false;
};

// Single global state instance
extern RobotState g_state;

// Interpolation settling timer (matches original openDog behaviour)
extern uint32_t g_previousInterpMillis;
extern uint8_t  g_interpFlag;
extern uint32_t g_currentMillis;
