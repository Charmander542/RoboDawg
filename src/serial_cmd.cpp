#include "serial_cmd.h"

#include <ctype.h>
#include <stdlib.h>
#include <string.h>

#include "config.h"
#include "gait.h"
#include "interpolation.h"
#include "servo_driver.h"
#include "state.h"

namespace serialCmd {

namespace {

char     g_buf[SERIAL_BUF_LEN];
uint16_t g_len = 0;

void printHelp() {
    Serial.println(F("RoboDawg serial commands:"));
    Serial.println(F("  WALK   x y yaw           set walking velocity targets"));
    Serial.println(F("  POSE   roll pitch yaw h  set body pose directly"));
    Serial.println(F("  WHEEL  fl fr bl br       set wheel speeds -100..+100"));
    Serial.println(F("  SERVO  ch angle          drive one channel for calibration"));
    Serial.println(F("  TRIM   ch offset_us      adjust trim and save to NVS"));
    Serial.println(F("  STOP                     zero all outputs immediately"));
    Serial.println(F("  STATUS                   print joint angles, pose, loop timing"));
    Serial.println(F("  HELP                     print this message"));
}

// Strip leading whitespace and return pointer to next token, plus advance *p
// past it. Returns nullptr when no token remains.
char* nextToken(char** p) {
    if (*p == nullptr) return nullptr;
    while (**p && isspace((unsigned char)**p)) (*p)++;
    if (**p == '\0') return nullptr;
    char* start = *p;
    while (**p && !isspace((unsigned char)**p)) (*p)++;
    if (**p) { **p = '\0'; (*p)++; }
    return start;
}

bool parseFloat(char* tok, float& out) {
    if (!tok) return false;
    char* end = nullptr;
    float v = strtof(tok, &end);
    if (end == tok) return false;
    out = v;
    return true;
}

bool parseInt(char* tok, long& out) {
    if (!tok) return false;
    char* end = nullptr;
    long v = strtol(tok, &end, 10);
    if (end == tok) return false;
    out = v;
    return true;
}

// Switch to a mode and reset whatever helpers it needs.
void enterMode(RunMode m) {
    g_state.mode = m;
    g_interpFlag = 0;
    g_previousInterpMillis = millis();
    if (m == MODE_WALK) {
        gait::reset();
    }
}

void handleWalk(char* rest) {
    float x, y, yaw;
    char* t1 = nextToken(&rest);
    char* t2 = nextToken(&rest);
    char* t3 = nextToken(&rest);
    if (!parseFloat(t1, x) || !parseFloat(t2, y) || !parseFloat(t3, yaw)) {
        Serial.println(F("ERR usage: WALK x y yaw"));
        return;
    }
    g_state.walkX   = x;
    g_state.walkY   = y;
    g_state.walkYaw = yaw;
    if (g_state.mode != MODE_WALK) enterMode(MODE_WALK);
    Serial.printf("OK WALK %.2f %.2f %.2f\n", x, y, yaw);
}

void handlePose(char* rest) {
    float r, p, y, h;
    char* t1 = nextToken(&rest);
    char* t2 = nextToken(&rest);
    char* t3 = nextToken(&rest);
    char* t4 = nextToken(&rest);
    if (!parseFloat(t1, r) || !parseFloat(t2, p) ||
        !parseFloat(t3, y) || !parseFloat(t4, h)) {
        Serial.println(F("ERR usage: POSE roll pitch yaw height"));
        return;
    }
    g_state.poseRoll   = r;
    g_state.posePitch  = p;
    g_state.poseYaw    = y;
    g_state.poseHeight = h;
    if (g_state.mode != MODE_POSE) enterMode(MODE_POSE);
    Serial.printf("OK POSE %.2f %.2f %.2f %.2f\n", r, p, y, h);
}

void handleWheel(char* rest) {
    float fl, fr, bl, br;
    char* t1 = nextToken(&rest);
    char* t2 = nextToken(&rest);
    char* t3 = nextToken(&rest);
    char* t4 = nextToken(&rest);
    if (!parseFloat(t1, fl) || !parseFloat(t2, fr) ||
        !parseFloat(t3, bl) || !parseFloat(t4, br)) {
        Serial.println(F("ERR usage: WHEEL fl fr bl br"));
        return;
    }
    g_state.wheel[SLOT_FL] = fl;
    g_state.wheel[SLOT_FR] = fr;
    g_state.wheel[SLOT_BL] = bl;
    g_state.wheel[SLOT_BR] = br;
    Serial.printf("OK WHEEL fl=%.1f fr=%.1f bl=%.1f br=%.1f\n", fl, fr, bl, br);
}

void handleServo(char* rest) {
    long ch;
    float angle;
    char* t1 = nextToken(&rest);
    char* t2 = nextToken(&rest);
    if (!parseInt(t1, ch) || !parseFloat(t2, angle)) {
        Serial.println(F("ERR usage: SERVO ch angle"));
        return;
    }
    if (ch < 0 || ch >= NUM_SERVO_CHANNELS) {
        Serial.println(F("ERR ch must be 0..15"));
        return;
    }
    if (g_state.mode != MODE_SERVO) enterMode(MODE_SERVO);
    servoDriver::driveServo((uint8_t)ch, angle);
    Serial.printf("OK SERVO %ld %.2f\n", ch, angle);
}

void handleTrim(char* rest) {
    long ch, offset;
    char* t1 = nextToken(&rest);
    char* t2 = nextToken(&rest);
    if (!parseInt(t1, ch) || !parseInt(t2, offset)) {
        Serial.println(F("ERR usage: TRIM ch offset_us"));
        return;
    }
    if (ch < 0 || ch >= NUM_SERVO_CHANNELS) {
        Serial.println(F("ERR ch must be 0..15"));
        return;
    }
    if (offset < -2000) offset = -2000;
    if (offset >  2000) offset =  2000;
    servoDriver::setTrimOffset((uint8_t)ch, (int16_t)offset, /*persist=*/true);
    Serial.printf("OK TRIM %ld %ld (saved)\n", ch, offset);
}

void handleStop() {
    g_state.mode = MODE_IDLE;
    g_state.walkX = g_state.walkY = g_state.walkYaw = 0.0f;
    for (int i = 0; i < 4; ++i) g_state.wheel[i] = 0.0f;
    servoDriver::stopAll();
    Serial.println(F("OK STOP"));
}

void handleStatus() {
    Serial.println(F("--- STATUS ---"));
    Serial.printf("mode=%u  loop_us=%lu  loop_us_max=%lu\n",
                  (unsigned)g_state.mode,
                  (unsigned long)g_state.loopMicrosLast,
                  (unsigned long)g_state.loopMicrosMax);
    Serial.printf("pose roll=%.2f pitch=%.2f yaw=%.2f height=%.1f\n",
                  g_state.poseRoll, g_state.posePitch,
                  g_state.poseYaw,  g_state.poseHeight);
    Serial.printf("walk x=%.2f y=%.2f yaw=%.2f\n",
                  g_state.walkX, g_state.walkY, g_state.walkYaw);
    Serial.printf("wheel FR=%.1f FL=%.1f BR=%.1f BL=%.1f\n",
                  g_state.wheel[SLOT_FR], g_state.wheel[SLOT_FL],
                  g_state.wheel[SLOT_BR], g_state.wheel[SLOT_BL]);
    static const char* names[] = {"FR", "FL", "BR", "BL"};
    for (int s = 0; s < 4; ++s) {
        Serial.printf("joints %s  hip=%.2f  thigh=%.2f  shin=%.2f\n",
                      names[s],
                      g_state.jointHip[s],
                      g_state.jointThigh[s],
                      g_state.jointShin[s]);
    }
    for (uint8_t ch = 0; ch < NUM_SERVO_CHANNELS; ++ch) {
        const ServoCal& c = servoDriver::getCalibration(ch);
        Serial.printf("cal ch%02u  min=%u max=%u trim=%d\n",
                      ch, c.minPulse, c.maxPulse, c.trimOffset);
    }
}

void dispatch(char* line) {
    while (*line && isspace((unsigned char)*line)) line++;
    if (*line == '\0' || *line == '#') return;

    char* cmd = nextToken(&line);
    if (!cmd) return;

    for (char* p = cmd; *p; ++p) *p = (char)toupper((unsigned char)*p);

    if      (!strcmp(cmd, "WALK"))   handleWalk(line);
    else if (!strcmp(cmd, "POSE"))   handlePose(line);
    else if (!strcmp(cmd, "WHEEL"))  handleWheel(line);
    else if (!strcmp(cmd, "SERVO"))  handleServo(line);
    else if (!strcmp(cmd, "TRIM"))   handleTrim(line);
    else if (!strcmp(cmd, "STOP"))   handleStop();
    else if (!strcmp(cmd, "STATUS")) handleStatus();
    else if (!strcmp(cmd, "HELP") || !strcmp(cmd, "?")) printHelp();
    else {
        Serial.print(F("ERR unknown command: "));
        Serial.println(cmd);
    }
}

}  // namespace

void begin() {
    g_len = 0;
    Serial.println();
    Serial.println(F("RoboDawg ready. Type HELP for command list."));
}

void poll() {
    while (Serial.available() > 0) {
        int c = Serial.read();
        if (c < 0) break;

        if (c == '\r') continue;
        if (c == '\n') {
            g_buf[g_len] = '\0';
            if (g_len > 0) dispatch(g_buf);
            g_len = 0;
            continue;
        }
        if (g_len < (SERIAL_BUF_LEN - 1)) {
            g_buf[g_len++] = (char)c;
        } else {
            g_len = 0;          // overflow: drop the line
            Serial.println(F("ERR line too long"));
        }
    }
}

}  // namespace serialCmd
