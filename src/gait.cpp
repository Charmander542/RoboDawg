#include "gait.h"

#include "config.h"
#include "interpolation.h"
#include "kinematics.h"
#include "state.h"

namespace gait {

// --- gait constants (ported from openDog runMode == 2) ---
namespace {

constexpr float LONG_LEG  = 340.0f;
constexpr float SHORT_LEG = 250.0f;
constexpr float FOOT_OFFSET = 0.0f;
constexpr uint16_t BASE_STEP_TIMER_MS = 75;

// Filtered velocity demands (degrees / mm)
float g_xFiltered   = 0.0f;
float g_yFiltered   = 0.0f;
float g_yawFiltered = 0.0f;

// Step machine state
uint8_t  g_stepFlag           = 0;
uint32_t g_previousStepMillis = 0;
float    g_timerScale         = BASE_STEP_TIMER_MS;

// Cached per-foot targets for the current step
float legLength1, legLength2;
float fr_RFB, fl_RFB, bl_RFB, br_RFB;
float fr_RLR, fl_RLR, bl_RLR, br_RLR;

void zeroFeet() {
    legLength1 = LONG_LEG;
    legLength2 = LONG_LEG;
    fr_RFB = fl_RFB = bl_RFB = br_RFB = 0.0f;
    fr_RLR =  FOOT_OFFSET;
    fl_RLR = -FOOT_OFFSET;
    bl_RLR = -FOOT_OFFSET;
    br_RLR =  FOOT_OFFSET;
}

}  // namespace

void reset() {
    g_stepFlag           = 0;
    g_previousStepMillis = millis();
    g_timerScale         = BASE_STEP_TIMER_MS;
    g_xFiltered = g_yFiltered = g_yawFiltered = 0.0f;
    zeroFeet();
    g_interpFlag          = 0;
    g_previousInterpMillis = millis();
}

void poseTick() {
    // Hold a static stance, applying the body pose. Interpolation off so
    // legs follow the commanded pose immediately (subject to the 300 ms
    // settling window).
    float roll   = g_state.poseRoll;
    float pitch  = g_state.posePitch;
    float yaw    = g_state.poseYaw;
    float height = g_state.poseHeight;

    kinematics(LEG_FR, 0, 0, height, roll, pitch, yaw, 0, 0);
    kinematics(LEG_FL, 0, 0, height, roll, pitch, yaw, 0, 0);
    kinematics(LEG_BL, 0, 0, height, roll, pitch, yaw, 0, 0);
    kinematics(LEG_BR, 0, 0, height, roll, pitch, yaw, 0, 0);
}

void tick() {
    // ---- demand smoothing ----
    // WALK x y yaw are dimensionless -100..+100. Map to mm / degrees.
    float xCmd   = constrain(g_state.walkX,   -100.0f, 100.0f);
    float yCmd   = constrain(g_state.walkY,   -100.0f, 100.0f);
    float yawCmd = constrain(g_state.walkYaw, -100.0f, 100.0f);

    float xMm   = xCmd   * 0.50f;   // -50..+50 mm step length
    float yMm   = yCmd   * 0.25f;   // -25..+25 mm step width
    float yawDeg = yawCmd * 0.25f;  // -25..+25 deg yaw per cycle

    g_xFiltered   = lowPassFilter(xMm,    g_xFiltered,   15);
    g_yFiltered   = lowPassFilter(yMm,    g_yFiltered,   15);
    g_yawFiltered = lowPassFilter(yawDeg, g_yawFiltered, 15);

    const float dead = 0.1f;
    bool centered = fabsf(g_xFiltered)   < dead
                 && fabsf(g_yFiltered)   < dead
                 && fabsf(g_yawFiltered) < dead;

    if (centered) {
        zeroFeet();
    } else {
        // ---- 4-state stepping machine (matches openDog) ----
        if (g_stepFlag == 0 && (g_currentMillis - g_previousStepMillis) > g_timerScale) {
            legLength1 = SHORT_LEG;  legLength2 = LONG_LEG;
            fr_RFB = -g_xFiltered;   fl_RFB =  g_xFiltered;
            bl_RFB = -g_xFiltered;   br_RFB =  g_xFiltered;
            fr_RLR = ( FOOT_OFFSET - g_yFiltered) + g_yawFiltered;
            fl_RLR = (-FOOT_OFFSET + g_yFiltered) - g_yawFiltered;
            bl_RLR = (-FOOT_OFFSET - g_yFiltered) - g_yawFiltered;
            br_RLR = ( FOOT_OFFSET + g_yFiltered) + g_yawFiltered;
            g_stepFlag = 1;
            g_previousStepMillis = g_currentMillis;
        } else if (g_stepFlag == 1 && (g_currentMillis - g_previousStepMillis) > g_timerScale) {
            legLength1 = LONG_LEG;   legLength2 = LONG_LEG;
            fr_RFB = -g_xFiltered;   fl_RFB =  g_xFiltered;
            bl_RFB = -g_xFiltered;   br_RFB =  g_xFiltered;
            fr_RLR = ( FOOT_OFFSET - g_yFiltered) + g_yawFiltered;
            fl_RLR = (-FOOT_OFFSET + g_yFiltered) - g_yawFiltered;
            bl_RLR = (-FOOT_OFFSET - g_yFiltered) - g_yawFiltered;
            br_RLR = ( FOOT_OFFSET + g_yFiltered) + g_yawFiltered;
            g_stepFlag = 2;
            g_previousStepMillis = g_currentMillis;
        } else if (g_stepFlag == 2 && (g_currentMillis - g_previousStepMillis) > g_timerScale) {
            legLength1 = LONG_LEG;   legLength2 = SHORT_LEG;
            fr_RFB =  g_xFiltered;   fl_RFB = -g_xFiltered;
            bl_RFB =  g_xFiltered;   br_RFB = -g_xFiltered;
            fr_RLR = ( FOOT_OFFSET + g_yFiltered) - g_yawFiltered;
            fl_RLR = (-FOOT_OFFSET - g_yFiltered) + g_yawFiltered;
            bl_RLR = (-FOOT_OFFSET + g_yFiltered) + g_yawFiltered;
            br_RLR = ( FOOT_OFFSET - g_yFiltered) - g_yawFiltered;
            g_stepFlag = 3;
            g_previousStepMillis = g_currentMillis;
        } else if (g_stepFlag == 3 && (g_currentMillis - g_previousStepMillis) > g_timerScale) {
            legLength1 = LONG_LEG;   legLength2 = LONG_LEG;
            fr_RFB =  g_xFiltered;   fl_RFB = -g_xFiltered;
            bl_RFB =  g_xFiltered;   br_RFB = -g_xFiltered;
            fr_RLR = ( FOOT_OFFSET + g_yFiltered) - g_yawFiltered;
            fl_RLR = (-FOOT_OFFSET - g_yFiltered) + g_yawFiltered;
            bl_RLR = (-FOOT_OFFSET + g_yFiltered) + g_yawFiltered;
            br_RLR = ( FOOT_OFFSET - g_yFiltered) - g_yawFiltered;
            g_stepFlag = 0;
            g_previousStepMillis = g_currentMillis;
        }

        // ---- adaptive timer based on step magnitude ----
        float stepLength = fabsf(fr_RFB);
        float stepWidth  = fabsf(fr_RLR);
        if (stepLength == 0.0f) stepLength = 0.01f;
        float stepAngle = atanf(stepLength / (stepWidth + 1e-3f));
        float stepHyp   = fabsf(stepLength / sinf(stepAngle));
        g_timerScale    = BASE_STEP_TIMER_MS + (stepHyp / 3.5f);
    }

    // ---- drive legs through IK with interpolation enabled ----
    int dur = (int)(g_timerScale * 0.8f);
    kinematics(LEG_FR, fr_RFB, fr_RLR, legLength1, 0, 0, 0, 1, dur);
    kinematics(LEG_FL, fl_RFB, fl_RLR, legLength2, 0, 0, 0, 1, dur);
    kinematics(LEG_BL, bl_RFB, bl_RLR, legLength1, 0, 0, 0, 1, dur);
    kinematics(LEG_BR, br_RFB, br_RLR, legLength2, 0, 0, 0, 1, dur);
}

}  // namespace gait
