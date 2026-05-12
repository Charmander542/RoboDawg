#include "gamepad.h"

#include "app_modes.h"
#include "config.h"
#include "servo_driver.h"
#include "state.h"

#if defined(__has_include)
#if __has_include(<Bluepad32.h>)
#define ROBO_HAVE_BLUEPAD32 1
#endif
#endif
#ifndef ROBO_HAVE_BLUEPAD32
#define ROBO_HAVE_BLUEPAD32 0
#endif

#if ROBO_HAVE_BLUEPAD32

#include <Bluepad32.h>

namespace {

ControllerPtr g_controllers[BP32_MAX_GAMEPADS] = {};

constexpr int   kDeadzone   = 25;     // same order of magnitude as Rachel De Barros blog
constexpr int   kAxisMax    = 511;
constexpr float kInvertLY   = 1.0f;  // push stick up (negative axisY) -> positive walk forward
constexpr float kInvertLX   = 1.0f;
constexpr float kInvertRYaw = 1.0f;

bool g_dumpGamepad = false;

// Map one axis (-511..512) through a deadband to -100..+100.
float axisToPercent(int v) {
    if (v >= -kDeadzone && v <= kDeadzone) {
        return 0.0f;
    }
    float sign = (v < 0) ? -1.0f : 1.0f;
    float mag  = (float)(abs(v) - kDeadzone) / (float)(kAxisMax - kDeadzone);
    mag = constrain(mag, 0.0f, 1.0f);
    return sign * mag * 100.0f;
}

void onConnectedController(ControllerPtr ctl) {
    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        if (g_controllers[i] == nullptr) {
            DBG_PRINTF("[gamepad] connected index=%d model=%s\n", i,
                       ctl->getModelName());
            g_controllers[i] = ctl;
            g_state.gamepadConnected = true;
            return;
        }
    }
    DBG_PRINTLN(F("[gamepad] no free slot for new controller"));
}

void onDisconnectedController(ControllerPtr ctl) {
    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        if (g_controllers[i] == ctl) {
            DBG_PRINTF("[gamepad] disconnected index=%d\n", i);
            g_controllers[i] = nullptr;
            break;
        }
    }
    bool any = false;
    for (auto c : g_controllers) {
        if (c && c->isConnected()) { any = true; break; }
    }
    g_state.gamepadConnected = any;
    if (!any) {
        g_state.walkX = g_state.walkY = g_state.walkYaw = 0.0f;
        for (int w = 0; w < 4; ++w) g_state.wheel[w] = 0.0f;
        appEnterMode(MODE_IDLE);
        servoDriver::stopAll();
    }
}

void processOneGamepad(ControllerPtr ctl) {
    if (g_dumpGamepad) {
        Serial.printf(
            "pad dpad=0x%02x btns=0x%04x L(%4d,%4d) R(%4d,%4d) brk=%4d thr=%4d\n",
            ctl->dpad(),
            ctl->buttons(),
            ctl->axisX(), ctl->axisY(),
            ctl->axisRX(), ctl->axisRY(),
            ctl->brake(), ctl->throttle());
    }

    // Do not fight manual servo calibration or explicit POSE hold.
    if (g_state.mode == MODE_SERVO || g_state.mode == MODE_POSE) {
        return;
    }

    float lx = kInvertLX * axisToPercent(ctl->axisX());
    float ly = kInvertLY * axisToPercent(ctl->axisY());
    float yawStick = kInvertRYaw * axisToPercent(ctl->axisRX());
    float wheelStick = axisToPercent(ctl->axisRY());

    bool walkActive = (lx != 0.0f || ly != 0.0f || yawStick != 0.0f);
    bool wheelActive = (wheelStick != 0.0f);

    if (walkActive) {
        g_state.walkX   = ly;
        g_state.walkY   = lx;
        g_state.walkYaw = yawStick;
        if (g_state.mode != MODE_WALK) {
            appEnterMode(MODE_WALK);
        }
    } else if (g_state.mode == MODE_WALK) {
        g_state.walkX = g_state.walkY = g_state.walkYaw = 0.0f;
    }

    if (wheelActive) {
        g_state.wheel[SLOT_FL] = wheelStick;
        g_state.wheel[SLOT_FR] = wheelStick;
        g_state.wheel[SLOT_BL] = wheelStick;
        g_state.wheel[SLOT_BR] = wheelStick;
    } else {
        for (int w = 0; w < 4; ++w) g_state.wheel[w] = 0.0f;
    }
}

void processAllControllers() {
    for (auto ctl : g_controllers) {
        if (ctl && ctl->isConnected() && ctl->hasData() && ctl->isGamepad()) {
            processOneGamepad(ctl);
            return;
        }
    }
}

}  // namespace

void gamepadBegin() {
    DBG_PRINTF("[gamepad] Bluepad32 %s\n", BP32.firmwareVersion());
    const uint8_t* addr = BP32.localBdAddress();
    DBG_PRINTF("[gamepad] BT addr %02X:%02X:%02X:%02X:%02X:%02X\n",
               addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);

    BP32.setup(&onConnectedController, &onDisconnectedController);
    // Do not call forgetBluetoothKeys() by default — it breaks re-pairing
    // for controllers that were already bonded (see Bluepad32 docs / blog).
    BP32.enableVirtualDevice(false);
}

void gamepadPoll() {
    bool updated = BP32.update();
    if (updated) {
        processAllControllers();
    }
    yield();
}

bool gamepadIsConnected() {
    return g_state.gamepadConnected;
}

bool gamepadIsDumping() {
    return g_dumpGamepad;
}

void gamepadSetDump(bool on) {
    g_dumpGamepad = on;
}

#else  // !ROBO_HAVE_BLUEPAD32

void gamepadBegin() {
    DBG_PRINTLN(F("[gamepad] stub (build env esp32dev_gamepad for Bluepad32)"));
}

void gamepadPoll() {}

bool gamepadIsConnected() {
    return false;
}

bool gamepadIsDumping() {
    return false;
}

void gamepadSetDump(bool) {}

#endif  // ROBO_HAVE_BLUEPAD32
