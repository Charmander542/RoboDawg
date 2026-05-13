#include <Arduino.h>
#include <WebServer.h>
#include <WiFi.h>

#include <ESPmDNS.h>

#include "app_modes.h"
#include "calib_html.h"
#include "config.h"
#include "gamepad.h"
#include "gait.h"
#include "serial_cmd.h"
#include "servo_driver.h"
#include "state.h"

#ifndef CALIB_AP_SSID
#define CALIB_AP_SSID "RoboDawg-Cal"
#endif
#ifndef CALIB_AP_PASS
#define CALIB_AP_PASS ""
#endif

namespace {

WebServer           g_server(80);
uint32_t            g_prevLoopMs = 0;
uint16_t            g_calChMask = 0;
float               g_calServoDeg[NUM_SERVO_CHANNELS] = {};
float               g_calWheelSpd[NUM_SERVO_CHANNELS] = {};

bool isWheelCh(uint8_t ch) { return ch >= 12 && ch <= 15; }

uint8_t legIdForSlot(uint8_t slot) {
    static const uint8_t kIds[4] = {LEG_FR, LEG_FL, LEG_BR, LEG_BL};
    return (slot < 4) ? kIds[slot] : LEG_FR;
}

void applyBenchHoldOutputs() {
    if (g_state.mode != MODE_SERVO) return;
    for (uint8_t ch = 0; ch < NUM_SERVO_CHANNELS; ++ch) {
        if ((g_calChMask & (1u << ch)) == 0) continue;
        if (isWheelCh(ch)) {
            servoDriver::driveWheel(ch, g_calWheelSpd[ch]);
        } else {
            servoDriver::driveServo(ch, g_calServoDeg[ch]);
        }
    }
}

void handleRoot() {
    g_server.send(200, "text/html", calib_pages::kIndexHtml);
}

void sendJsonState() {
    char  buf[1024];
    char* p   = buf;
    char* end = buf + sizeof(buf);
    int   n   = snprintf(p, (size_t)(end - p),
                        "{\"mode\":%u,\"legMask\":%u,\"wheelMask\":%u,\"trim\":[",
                        (unsigned)g_state.mode,
                        (unsigned)g_state.legOutputMask,
                        (unsigned)g_state.wheelOutputMask);
    if (n < 0 || (size_t)n >= (size_t)(end - p)) {
        g_server.send(500, "text/plain", "overflow");
        return;
    }
    p += n;
    for (int i = 0; i < NUM_SERVO_CHANNELS; ++i) {
        int tr = (int)servoDriver::getCalibration((uint8_t)i).trimOffset;
        n = snprintf(p, (size_t)(end - p), "%s%d", (i == 0) ? "" : ",", tr);
        if (n < 0 || (size_t)n >= (size_t)(end - p)) {
            g_server.send(500, "text/plain", "overflow");
            return;
        }
        p += n;
    }
    if ((size_t)(end - p) < 3) {
        g_server.send(500, "text/plain", "overflow");
        return;
    }
    *p++ = ']';
    *p++ = '}';
    *p   = '\0';
    g_server.send(200, "application/json", buf);
}

void handleApiTrim() {
    if (!g_server.hasArg("ch") || !g_server.hasArg("trim")) {
        g_server.send(400, "text/plain", "missing ch/trim");
        return;
    }
    int ch   = g_server.arg("ch").toInt();
    int trim = g_server.arg("trim").toInt();
    if (ch < 0 || ch >= NUM_SERVO_CHANNELS) {
        g_server.send(400, "text/plain", "bad ch");
        return;
    }
    trim = constrain(trim, -2000, 2000);
    servoDriver::setTrimOffset((uint8_t)ch, (int16_t)trim, /*persist=*/false);

    if (g_calChMask & (1u << (unsigned)ch)) {
        if (isWheelCh((uint8_t)ch)) {
            servoDriver::driveWheel((uint8_t)ch, g_calWheelSpd[ch]);
        } else {
            servoDriver::driveServo((uint8_t)ch, g_calServoDeg[ch]);
        }
    }
    g_server.send(200, "text/plain", "ok");
}

void handleApiSave() {
    servoDriver::saveCalibration();
    g_server.send(200, "text/plain", "ok");
}

void handleApiMask() {
    if (!g_server.hasArg("legs") || !g_server.hasArg("wheels")) {
        g_server.send(400, "text/plain", "missing legs/wheels");
        return;
    }
    int legs   = g_server.arg("legs").toInt();
    int wheels = g_server.arg("wheels").toInt();
    legs       = constrain(legs, 0, 15);
    wheels     = constrain(wheels, 0, 15);
    g_state.legOutputMask   = (uint8_t)legs;
    g_state.wheelOutputMask = (uint8_t)wheels;
    g_server.send(200, "text/plain", "ok");
}

void handleApiHold() {
    if (!g_server.hasArg("slot") || !g_server.hasArg("on")) {
        g_server.send(400, "text/plain", "missing slot/on");
        return;
    }
    int slot = g_server.arg("slot").toInt();
    int on   = g_server.arg("on").toInt();
    if (slot < 0 || slot > 3) {
        g_server.send(400, "text/plain", "bad slot");
        return;
    }
    uint8_t leg = legIdForSlot((uint8_t)slot);
    uint8_t h   = hipChannel(leg);
    uint8_t t   = thighChannel(leg);
    uint8_t s   = shinChannel(leg);
    uint32_t m  = (1u << h) | (1u << t) | (1u << s);
    if (on) {
        g_calServoDeg[h] = 90.0f;
        g_calServoDeg[t] = 90.0f;
        g_calServoDeg[s] = 90.0f;
        g_calChMask      = (uint16_t)(g_calChMask | m);
        appEnterMode(MODE_SERVO);
    } else {
        g_calChMask = (uint16_t)(g_calChMask & ~m);
        if (g_calChMask == 0 && g_state.mode == MODE_SERVO) {
            g_state.mode = MODE_IDLE;
        }
    }
    g_server.send(200, "text/plain", "ok");
}

void handleApiWalk() {
    if (!g_server.hasArg("x") || !g_server.hasArg("y") || !g_server.hasArg("yaw")) {
        g_server.send(400, "text/plain", "missing x/y/yaw");
        return;
    }
    g_calChMask = 0;
    g_state.walkX   = constrain(g_server.arg("x").toFloat(), -100.0f, 100.0f);
    g_state.walkY   = constrain(g_server.arg("y").toFloat(), -100.0f, 100.0f);
    g_state.walkYaw = constrain(g_server.arg("yaw").toFloat(), -100.0f, 100.0f);
    appEnterMode(MODE_WALK);
    g_server.send(200, "text/plain", "ok");
}

void handleApiIdle() {
    g_state.walkX = g_state.walkY = g_state.walkYaw = 0.0f;
    g_state.poseRoll = g_state.posePitch = g_state.poseYaw = 0.0f;
    g_state.poseHeight                                   = 330.0f;
    g_calChMask                                          = 0;
    appEnterMode(MODE_POSE);
    g_server.send(200, "text/plain", "ok");
}

void handleApiStop() {
    g_calChMask = 0;
    g_state.mode = MODE_IDLE;
    g_state.walkX = g_state.walkY = g_state.walkYaw = 0.0f;
    for (auto& w : g_state.wheel) w = 0.0f;
    servoDriver::stopAll();
    g_server.send(200, "text/plain", "ok");
}

void registerHttp() {
    g_server.on("/", HTTP_GET, handleRoot);
    g_server.on("/api/state", HTTP_GET, sendJsonState);
    g_server.on("/api/trim", HTTP_POST, handleApiTrim);
    g_server.on("/api/save", HTTP_POST, handleApiSave);
    g_server.on("/api/mask", HTTP_POST, handleApiMask);
    g_server.on("/api/hold", HTTP_POST, handleApiHold);
    g_server.on("/api/walk", HTTP_POST, handleApiWalk);
    g_server.on("/api/idle", HTTP_POST, handleApiIdle);
    g_server.on("/api/stop", HTTP_POST, handleApiStop);
    g_server.begin();
}

}  // namespace

RobotState g_state;
uint32_t   g_previousInterpMillis = 0;
uint8_t    g_interpFlag           = 0;
uint32_t   g_currentMillis        = 0;

void setup() {
    Serial.begin(115200);
    uint32_t t0 = millis();
    while (!Serial && (millis() - t0 < 1500)) { /* wait for USB */ }

    DBG_PRINTLN(F("[cal] RoboDawg calibration firmware"));

    if (!servoDriver::begin()) {
        DBG_PRINTLN(F("[cal] FATAL: PCA9685 init failed"));
    }
    servoDriver::stopAll();

    g_state.legOutputMask   = 0x0F;
    g_state.wheelOutputMask = 0x0F;
    g_state.mode            = MODE_IDLE;

    serialCmd::begin();
    gamepadBegin();

    WiFi.persistent(false);
    WiFi.setSleep(false);
    WiFi.mode(WIFI_AP);
    const char* apPass =
        (CALIB_AP_PASS[0] != '\0') ? CALIB_AP_PASS : (const char*)nullptr;
    if (!WiFi.softAP(CALIB_AP_SSID, apPass)) {
        DBG_PRINTLN(F("[cal] softAP failed"));
    } else {
        DBG_PRINTF("[cal] AP '%s'  IP %s\n", CALIB_AP_SSID,
                   WiFi.softAPIP().toString().c_str());
    }

    if (MDNS.begin("robodawg-cal")) {
        MDNS.addService("http", "tcp", 80);
        DBG_PRINTLN(F("[cal] mDNS http://robodawg-cal.local"));
    }

    registerHttp();

    g_previousInterpMillis = millis();
    DBG_PRINTLN(F("[cal] HTTP server on :80"));
}

void loop() {
    g_server.handleClient();
    gamepadPoll();
    serialCmd::poll();

    g_currentMillis = millis();
    if (g_currentMillis - g_prevLoopMs < CONTROL_LOOP_PERIOD_MS) return;
    g_prevLoopMs = g_currentMillis;

    const uint32_t tStart = micros();

    switch (g_state.mode) {
        case MODE_IDLE:
            applyBenchHoldOutputs();
            break;
        case MODE_POSE:
            gait::poseTick();
            break;
        case MODE_WALK:
            gait::tick();
            break;
        case MODE_SERVO:
            applyBenchHoldOutputs();
            break;
    }

    auto wheelSpeed = [](uint8_t slot) {
        float s = g_state.wheel[slot];
        if ((g_state.wheelOutputMask & (1u << slot)) == 0) s = 0.0f;
        return s;
    };
    servoDriver::driveWheel(wheelChannel(LEG_FR), wheelSpeed(SLOT_FR));
    servoDriver::driveWheel(wheelChannel(LEG_FL), wheelSpeed(SLOT_FL));
    servoDriver::driveWheel(wheelChannel(LEG_BR), wheelSpeed(SLOT_BR));
    servoDriver::driveWheel(wheelChannel(LEG_BL), wheelSpeed(SLOT_BL));

    const uint32_t dt = micros() - tStart;
    g_state.loopMicrosLast = dt;
    if (dt > g_state.loopMicrosMax) g_state.loopMicrosMax = dt;
}
