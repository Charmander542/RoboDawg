#include "gait.h"

#include "config.h"
#include "interpolation.h"
#include "kinematics.h"
#include "state.h"

namespace gait {

// ---------------------------------------------------------------------------
// Continuous diagonal-trot gait
//
// The legacy openDog port drove "steps" by alternating LONG_LEG / SHORT_LEG
// (~90 mm) while snapping RFB targets — that reads as micro-hops. Here we
// keep the same IK interface (RFB, RLR, z demand per leg) but:
//   * z stays near the standing nominal (same order as POSE height) with a
//     modest dip only during swing for ground clearance;
//   * horizontal motion is a smooth stance / swing split on a phase clock;
//   * FR+BL share one phase, FL+BR the opposite (classic diagonal trot).
//
// Joint neutrals use SERVO_NEUTRAL_DEG (mid of 0..270° leg servos).
// ---------------------------------------------------------------------------

namespace {

// --- geometry / timing (mm, seconds) --------------------------------------
// Standing height demand passed to kinematics (matches default poseHeight).
constexpr float kNominalZ = 330.0f;

// During swing, commanded z dips slightly (same sense as the old SHORT_LEG
// trick: shorter demand lifts the foot in this IK formulation).
constexpr float kSwingZDropMm = 18.0f;

// Fraction of each cycle each foot spends on the ground (0..1).
constexpr float kStanceDuty = 0.58f;

// Map WALK -100..+100 filtered mm-ish commands into step size (mm peak).
constexpr float kStepFbScale = 0.95f;   // forward/back stroke gain
constexpr float kStepLrScale = 0.55f; // strafe stroke gain

constexpr float kMinStepMm = 5.0f;
constexpr float kMaxStepMm = 52.0f;

// Gait frequency (Hz) from command magnitude — faster input → quicker legs.
constexpr float kMinGaitHz = 0.55f;
constexpr float kMaxGaitHz = 3.0f;
constexpr float kHzPerCmd  = 0.038f;  // tuned for -100..100 → pleasant cadence

constexpr float kYawStepCoupling = 0.35f; // couples yaw demand into step size

constexpr float FOOT_OFFSET = 0.0f;

constexpr float kTwoPi = 6.28318530717958647692f;

// Filtered velocity demands (mm / deg) — same low-pass idea as the legacy.
float g_xFiltered   = 0.0f;
float g_yFiltered   = 0.0f;
float g_yawFiltered = 0.0f;

// Phase integrator (radians). FR/BL use g_phase; FL/BR use g_phase + π.
float g_phase = 0.0f;

uint32_t g_lastTickMillis = 0;

inline float smoothstep01(float t) {
    if (t <= 0.0f) return 0.0f;
    if (t >= 1.0f) return 1.0f;
    return t * t * (3.0f - 2.0f * t);
}

// Piecewise stance/swing template on phase φ ∈ [0, 2π):
//   Returns roughly [-1, +1] with slow stance transit and smooth swing return.
inline float stanceSwingWave(float phi) {
    float p = phi;
    // wrap to [0, 2π)
    p = fmodf(p, kTwoPi);
    if (p < 0.0f) p += kTwoPi;
    const float p01 = p / kTwoPi;  // [0,1)

    if (p01 < kStanceDuty) {
        // Stance: linear move from +1 → -1 (foot travels backward in body
        // frame while supporting weight).
        const float u = p01 / kStanceDuty;
        return 1.0f - 2.0f * u;
    }
    // Swing: smooth return from -1 → +1.
    const float u = (p01 - kStanceDuty) / (1.0f - kStanceDuty);
    return -1.0f + 2.0f * smoothstep01(u);
}

// Z demand for one diagonal group sharing `phi`.
inline float zForPhase(float phi) {
    float p = phi;
    p = fmodf(p, kTwoPi);
    if (p < 0.0f) p += kTwoPi;
    const float p01 = p / kTwoPi;

    if (p01 < kStanceDuty) {
        return kNominalZ;
    }
    const float u = (p01 - kStanceDuty) / (1.0f - kStanceDuty);
    // Bell-shaped lift: 0 at lift-off/touch-down, peak mid-swing.
    const float bell = sinf(PI * u);
    return kNominalZ - kSwingZDropMm * bell;
}

inline float clampStep(float v) {
    float a = fabsf(v);
    if (a < kMinStepMm) return (v >= 0.0f) ? kMinStepMm : -kMinStepMm;
    if (a > kMaxStepMm) return (v >= 0.0f) ? kMaxStepMm : -kMaxStepMm;
    return v;
}

}  // namespace

void reset() {
    g_phase            = 0.0f;
    g_lastTickMillis   = millis();
    g_xFiltered = g_yFiltered = g_yawFiltered = 0.0f;
    g_interpFlag          = 0;
    g_previousInterpMillis = millis();
}

void poseTick() {
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
    // ---- command filtering (same IO scale as before: WALK -100..+100) ----
    float xCmd   = constrain(g_state.walkX,   -100.0f, 100.0f);
    float yCmd   = constrain(g_state.walkY,   -100.0f, 100.0f);
    float yawCmd = constrain(g_state.walkYaw, -100.0f, 100.0f);

    float xMm    = xCmd   * 0.50f;   // -50..+50 mm forward/back demand
    float yMm    = yCmd   * 0.25f;   // -25..+25 mm lateral demand
    float yawDeg = yawCmd * 0.25f;   // -25..+25 deg yaw per cycle demand

    g_xFiltered   = lowPassFilter(xMm,    g_xFiltered,   15);
    g_yFiltered   = lowPassFilter(yMm,    g_yFiltered,   15);
    g_yawFiltered = lowPassFilter(yawDeg, g_yawFiltered, 15);

    const float dead = 0.12f;
    const bool centered = fabsf(g_xFiltered) < dead
                       && fabsf(g_yFiltered) < dead
                       && fabsf(g_yawFiltered) < dead;

    // ---- timing dt (robust to jitter if the main loop slips a tick) -------
    if (g_lastTickMillis == 0) g_lastTickMillis = g_currentMillis;
    float dt_s = (g_currentMillis - g_lastTickMillis) * 0.001f;
    if (dt_s < 0.001f) dt_s = CONTROL_LOOP_PERIOD_MS * 0.001f;
    if (dt_s > 0.05f) dt_s = 0.05f;  // clamp huge pauses
    g_lastTickMillis = g_currentMillis;

    // ---- neutral lateral offsets (same structure as legacy zeroFeet) -----
    float fr_RLR = (FOOT_OFFSET - g_yFiltered) + g_yawFiltered;
    float fl_RLR = (-FOOT_OFFSET + g_yFiltered) - g_yawFiltered;
    float bl_RLR = (-FOOT_OFFSET - g_yFiltered) - g_yawFiltered;
    float br_RLR = (FOOT_OFFSET + g_yFiltered) + g_yawFiltered;

    float fr_RFB = 0.0f, fl_RFB = 0.0f, bl_RFB = 0.0f, br_RFB = 0.0f;
    float zFR = kNominalZ, zFL = kNominalZ, zBL = kNominalZ, zBR = kNominalZ;

    if (!centered) {
        // Omnidirectional step magnitude (mm).
        const float magXY = sqrtf(g_xFiltered * g_xFiltered + g_yFiltered * g_yFiltered);
        const float magCmd = magXY + kYawStepCoupling * fabsf(g_yawFiltered);

        float stepFb = clampStep(kStepFbScale * fabsf(g_xFiltered));
        float stepLr = clampStep(kStepLrScale * fabsf(g_yFiltered));

        // When both forward and strafe are meaningful, use one stroke size on
        // a blended heading so the feet stay inside a comfortable workspace.
        if (fabsf(g_xFiltered) > 1.2f && fabsf(g_yFiltered) > 1.2f) {
            const float blended =
                constrain(kStepFbScale * magXY, kMinStepMm, kMaxStepMm);
            stepFb = stepLr = blended;
        } else if (magXY <= 1e-3f && fabsf(g_yawFiltered) > dead) {
            // In-place pivoting: modest FB stroke driven by yaw demand.
            stepFb = clampStep(10.0f + 0.45f * fabsf(g_yawFiltered));
            stepLr = 0.0f;
        }

        // Gait frequency ramps with how hard the operator asks to move.
        float hz = kHzPerCmd * magCmd;
        if (hz < kMinGaitHz) hz = kMinGaitHz;
        if (hz > kMaxGaitHz) hz = kMaxGaitHz;

        g_phase += kTwoPi * hz * dt_s;

        // Diagonal trot: FR & BL share φ, FL & BR share φ + π.
        const float wA = stanceSwingWave(g_phase);
        const float wB = stanceSwingWave(g_phase + PI);

        // Forward/back component (body X). Signs match the legacy diagonal
        // pairing: FR/BL opposite to FL/BR for a given command direction.
        fr_RFB += -wA * stepFb;
        bl_RFB += -wA * stepFb;
        fl_RFB += wB * stepFb;
        br_RFB += wB * stepFb;

        // Strafe component (body Y): sinusoid in quadrature with FB motion,
        // with left/right mirror pattern similar to the static RLR offsets.
        const float sA = sinf(g_phase);
        const float sB = sinf(g_phase + PI);
        fr_RLR += sA * stepLr;
        bl_RLR -= sA * stepLr;
        fl_RLR -= sB * stepLr;
        br_RLR += sB * stepLr;

        zFR = zForPhase(g_phase);
        zBL = zFR;
        zFL = zForPhase(g_phase + PI);
        zBR = zFL;
    } else {
        // Decay phase when stopped so the next start does not jump from a
        // random angle in the cycle.
        g_phase *= 0.85f;
        if (fabsf(g_phase) < 0.05f) g_phase = 0.0f;
    }

    // ---- drive legs through IK with interpolation -------------------------
    // Duration tracks gait speed a bit so ramps stay smooth at high cadence.
    const float magXY2 =
        sqrtf(g_xFiltered * g_xFiltered + g_yFiltered * g_yFiltered);
    const float magCmd2 = magXY2 + kYawStepCoupling * fabsf(g_yawFiltered);
    float hzEst = kHzPerCmd * magCmd2;
    if (hzEst < kMinGaitHz) hzEst = kMinGaitHz;
    if (hzEst > kMaxGaitHz) hzEst = kMaxGaitHz;
    int dur = (int)(constrain(520.0f / hzEst, 22.0f, 85.0f));

    kinematics(LEG_FR, fr_RFB, fr_RLR, zFR, 0, 0, 0, 1, dur);
    kinematics(LEG_FL, fl_RFB, fl_RLR, zFL, 0, 0, 0, 1, dur);
    kinematics(LEG_BL, bl_RFB, bl_RLR, zBL, 0, 0, 0, 1, dur);
    kinematics(LEG_BR, br_RFB, br_RLR, zBR, 0, 0, 0, 1, dur);
}

}  // namespace gait
