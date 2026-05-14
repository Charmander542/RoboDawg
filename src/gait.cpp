#include "gait.h"

#include "config.h"
#include "interpolation.h"
#include "servo_driver.h"
#include "state.h"

namespace gait {

// ---------------------------------------------------------------------------
// Direct servo-angle gait.
//
// We do *not* use the body-frame IK in kinematics.cpp here. That IK was
// inherited from openDog and assumes ~340 mm leg geometry; our actual
// thigh+shin only reaches ~175 mm, so any commanded body height saturates
// the IK and the legs snap to extreme/invalid angles ("legs move strange
// then freeze").
//
// Instead the gait is parametrised directly in servo angle space, with
// the user's measured mechanical envelopes baked in as hard limits:
//
//   Hip:   HIP_NEUTRAL + out +20° / in −15° (same numbers every leg;
//          left/right mirror only in `gait.cpp` `kLegs` and IK `g_legMap`)
//   Femur: logical gait uses FR band 150°..200°; FL/BL servos use mirrored
//          band 115°..70° (`femurRightFrameToServoForSlot()` in config.h).
//   Tibia: TIBIA_SAFE_MIN_DEG .. TIBIA_SAFE_MAX_DEG        (100° .. 200°)
//
// Named body-height poses (also user-supplied):
//   LOW   = (FEMUR 150, TIBIA 200)   lowest body
//   HIGH  = (FEMUR 160, TIBIA 100)   highest body
//   STAND = mid-of-walk baseline      (~155, ~150)
//
// Walking forward, every leg shares the same *logical* femur stroke in the
// right-leg band; FL/BL map to their mirrored servo band. `hipMul` /
// `femurMul` / `tibiaMul` handle bench wiring flips on top of that.
//
// Gait pattern: a 4-beat *crawl* (LF, RH, RF, LH) — only one foot is ever
// in swing, three feet always on the ground. Slower than a trot but very
// stable while bench-testing sign conventions and limits.
// ---------------------------------------------------------------------------

namespace {

// ----- Stroke amplitudes within the femur/tibia envelopes ------------------
// Femur swing during stance pushes the foot from "forward" -> "back" inside
// the safe band, then returns during swing.
constexpr float FEMUR_FWD_DEG  = 170.0f;
constexpr float FEMUR_BACK_DEG = 155.0f;
// Tibia stance keeps the leg extended; swing tucks the knee to lift the
// foot, then re-extends for ground contact.
constexpr float TIBIA_STANCE_DEG = 175.0f;
constexpr float TIBIA_SWING_DEG  = 110.0f;

// Stance/swing split. 0.75 = 3 legs always down (classic crawl).
constexpr float STANCE_DUTY = 0.75f;

// Cadence (Hz) scales with command magnitude.
constexpr float MIN_HZ      = 0.45f;
constexpr float MAX_HZ      = 1.8f;
constexpr float HZ_PER_CMD  = 0.020f;

// ----- WHERE TO INVERT (mirroring) ----------------------------------------
// **Hip mirroring (leg 1 = FR reference):** FR and BL share the same hip
// sign convention; FL and BR share the opposite. Femur/tibia stay
// leg‑1‑style (+1,+1) on every slot unless you invert a leg on the bench.
//
// If a leg walks backward, flip that leg’s femurMul / tibiaMul (±1) only;
// flip hipMul only if hip abduction still disagrees with hardware after
// the FR/BL vs FL/BR pairing above.
//
// IK / POSE direction flips live in `kinematics.cpp` → `g_legMap[]` hipDir,
// thighDir, shinDir (separate from this table).
//
struct LegConfig {
    float hipMul;       // ±1 — flips hip offset sign for mirrored mounts
    float femurMul;     // ±1 — if −1, femur angle is mirrored in safe band
    float tibiaMul;     // ±1 — if −1, tibia angle is mirrored in safe band
    float phaseShift;   // 0..1 crawl phase offset
};

// Indexed by LegSlot: SLOT_FR=0, SLOT_FL=1, SLOT_BR=2, SLOT_BL=3
// Hip: FR+BL same (hipMul +1); FL+BR flipped (hipMul −1).
constexpr LegConfig kLegs[4] = {
    /* FR  leg 1 — reference */ { +1.0f, +1.0f, +1.0f, 0.25f },
    /* FL  — hip with BR */     { -1.0f, +1.0f, +1.0f, 0.75f },
    /* BR  — hip with FL */     { -1.0f, +1.0f, +1.0f, 0.50f },
    /* BL  — hip with FR */     { +1.0f, +1.0f, +1.0f, 0.00f },
};

// ----- Runtime state -------------------------------------------------------
float    g_xFilt   = 0.0f;
float    g_yawFilt = 0.0f;
float    g_phase   = 0.0f;        // [0, 1)
uint32_t g_lastTickMs = 0;

inline float clampF(float v, float lo, float hi) {
    return (v < lo) ? lo : (v > hi ? hi : v);
}

inline float smoothstep01(float t) {
    if (t <= 0.0f) return 0.0f;
    if (t >= 1.0f) return 1.0f;
    return t * t * (3.0f - 2.0f * t);
}

inline float bellLift(float u) {
    if (u <= 0.0f || u >= 1.0f) return 0.0f;
    return sinf(u * PI);
}

inline float wrap01(float p) {
    p = fmodf(p, 1.0f);
    if (p < 0.0f) p += 1.0f;
    return p;
}

// Logical foot target for a single leg, given its local phase and the
// signed forward gain in [-1, +1].  We return three absolute servo angles
// (NOT yet sign-mirrored per leg).
struct LegTarget {
    float hipOffset;   // delta from HIP_NEUTRAL_DEG
    float femurDeg;    // absolute servo angle (logical, before mirror)
    float tibiaDeg;    // absolute servo angle
};

LegTarget computeLegTarget(float localPhase, float fwdGain) {
    LegTarget out;
    out.hipOffset = 0.0f;

    const float midF  = 0.5f * (FEMUR_FWD_DEG + FEMUR_BACK_DEG);
    const float halfF = 0.5f * (FEMUR_FWD_DEG - FEMUR_BACK_DEG);

    if (localPhase < STANCE_DUTY) {
        // Stance: foot on ground, sweep femur fwd -> back so the leg
        // pushes the body forward when fwdGain > 0.
        const float u = localPhase / STANCE_DUTY;          // 0..1
        const float strokeAmp = halfF * fwdGain;
        out.femurDeg = midF + strokeAmp * (1.0f - 2.0f * u);
        out.tibiaDeg = TIBIA_STANCE_DEG;
    } else {
        // Swing: foot in air, return femur back -> fwd with a smooth
        // ease, and lift the tibia via a sin-bell so the foot clears.
        const float u  = (localPhase - STANCE_DUTY) / (1.0f - STANCE_DUTY);
        const float us = smoothstep01(u);
        const float strokeAmp = halfF * fwdGain;
        out.femurDeg = midF + strokeAmp * (2.0f * us - 1.0f);
        const float lift = bellLift(u);
        out.tibiaDeg = TIBIA_STANCE_DEG + (TIBIA_SWING_DEG - TIBIA_STANCE_DEG) * lift;
    }

    out.femurDeg = clampF(out.femurDeg, FEMUR_SAFE_MIN_DEG, FEMUR_SAFE_MAX_DEG);
    out.tibiaDeg = clampF(out.tibiaDeg, TIBIA_SAFE_MIN_DEG, TIBIA_SAFE_MAX_DEG);
    return out;
}

void driveOneLeg(uint8_t slot, const LegTarget& tgt) {
    const uint8_t hipCh   = (uint8_t)(CHANNELS_PER_LEG * slot + 0u);
    const uint8_t femCh   = (uint8_t)(CHANNELS_PER_LEG * slot + 1u);
    const uint8_t tibCh   = (uint8_t)(CHANNELS_PER_LEG * slot + 2u);

    const bool left   = legSlotIsLeftFemurMirror(slot);
    const float femLo = left ? FEMUR_SAFE_MIN_DEG_L : FEMUR_SAFE_MIN_DEG_R;
    const float femHi = left ? FEMUR_SAFE_MAX_DEG_L : FEMUR_SAFE_MAX_DEG_R;

    if ((g_state.legOutputMask & (1u << slot)) == 0) {
        // Disabled — park hip neutral, femur/tibia at STAND (safe & out
        // of the way). Hard limits in driveServo would catch anything
        // else anyway, but this keeps the joint cache sensible.
        const float standFem = femurRightFrameToServoForSlot(slot, FEMUR_STAND_DEG);
        servoDriver::driveServo(hipCh, HIP_NEUTRAL_DEG);
        servoDriver::driveServo(femCh, standFem);
        servoDriver::driveServo(tibCh, TIBIA_STAND_DEG);
        g_state.jointHip  [slot] = 0.0f;
        g_state.jointThigh[slot] = standFem - 0.5f * (femLo + femHi);
        g_state.jointShin [slot] = TIBIA_STAND_DEG - 0.5f * (TIBIA_SAFE_MIN_DEG + TIBIA_SAFE_MAX_DEG);
        return;
    }

    const LegConfig& s = kLegs[slot];

    // Hip command: HIP_NEUTRAL +/- offset (signed by mirroring).
    float hipDelta = s.hipMul * tgt.hipOffset;
    if (hipDelta >  HIP_OUT_DEG) hipDelta =  HIP_OUT_DEG;
    if (hipDelta < -HIP_IN_DEG)  hipDelta = -HIP_IN_DEG;
    float hipCmd = HIP_NEUTRAL_DEG + hipDelta;

    // Femur: right-frame logical angle → per-side servo band; optional
    // `femurMul` flips within that leg's own min..max.
    float femurCmd = femurRightFrameToServoForSlot(slot, tgt.femurDeg);
    if (s.femurMul < 0.0f) {
        femurCmd = (femLo + femHi) - femurCmd;
    }
    float tibiaCmd = tgt.tibiaDeg;
    if (s.tibiaMul < 0.0f) {
        tibiaCmd = (TIBIA_SAFE_MIN_DEG + TIBIA_SAFE_MAX_DEG) - tibiaCmd;
    }

    servoDriver::driveServo(hipCh, hipCmd);
    servoDriver::driveServo(femCh, femurCmd);
    servoDriver::driveServo(tibCh, tibiaCmd);

    g_state.jointHip  [slot] = hipCmd   - HIP_NEUTRAL_DEG;
    g_state.jointThigh[slot] = femurCmd - 0.5f * (femLo + femHi);
    g_state.jointShin [slot] = tibiaCmd - 0.5f * (TIBIA_SAFE_MIN_DEG + TIBIA_SAFE_MAX_DEG);
}

void driveFixedAll(float femurDeg, float tibiaDeg) {
    LegTarget t;
    t.hipOffset = 0.0f;
    t.femurDeg  = clampF(femurDeg, FEMUR_SAFE_MIN_DEG, FEMUR_SAFE_MAX_DEG);
    t.tibiaDeg  = clampF(tibiaDeg, TIBIA_SAFE_MIN_DEG, TIBIA_SAFE_MAX_DEG);
    for (uint8_t slot = 0; slot < 4; ++slot) {
        driveOneLeg(slot, t);
    }
}

}  // namespace

// ----- Public API ----------------------------------------------------------

void reset() {
    g_phase        = 0.0f;
    g_lastTickMs   = millis();
    g_xFilt        = 0.0f;
    g_yawFilt      = 0.0f;
    g_interpFlag          = 0;
    g_previousInterpMillis = millis();
}

void driveLow()   { driveFixedAll(FEMUR_LOW_DEG,   TIBIA_LOW_DEG);   }
void driveHigh()  { driveFixedAll(FEMUR_HIGH_DEG,  TIBIA_HIGH_DEG);  }
void driveStand() { driveFixedAll(FEMUR_STAND_DEG, TIBIA_STAND_DEG); }

void poseTick() {
    // POSE roll/pitch/yaw are ignored in the direct-servo gait; height is
    // reinterpreted as a 0..1 interpolation between LOW and HIGH.
    // Legacy mm values (e.g. 330) clamp to HIGH so old scripts behave.
    float h = g_state.poseHeight;
    float t;
    if (h <= 1.5f) {
        t = clampF(h, 0.0f, 1.0f);
    } else {
        // ~100mm => LOW, ~200mm+ => HIGH; values like 330 saturate at HIGH.
        t = clampF((h - 100.0f) / 100.0f, 0.0f, 1.0f);
    }
    const float femurDeg = FEMUR_LOW_DEG + t * (FEMUR_HIGH_DEG - FEMUR_LOW_DEG);
    const float tibiaDeg = TIBIA_LOW_DEG + t * (TIBIA_HIGH_DEG - TIBIA_LOW_DEG);
    driveFixedAll(femurDeg, tibiaDeg);
}

void tick() {
    // ---- demand smoothing (WALK x = forward, walkYaw = turning) -----------
    const float xCmd   = clampF(g_state.walkX,   -100.0f, 100.0f);
    const float yawCmd = clampF(g_state.walkYaw, -100.0f, 100.0f);
    g_xFilt   = lowPassFilter(xCmd,   g_xFilt,   12);
    g_yawFilt = lowPassFilter(yawCmd, g_yawFilt, 12);

    const float deadband = 3.0f;
    const bool moving = (fabsf(g_xFilt) > deadband) || (fabsf(g_yawFilt) > deadband);

    // ---- timing (robust to occasional slipped ticks) ---------------------
    if (g_lastTickMs == 0) g_lastTickMs = g_currentMillis;
    float dt_s = (g_currentMillis - g_lastTickMs) * 0.001f;
    if (dt_s < 0.001f) dt_s = CONTROL_LOOP_PERIOD_MS * 0.001f;
    if (dt_s > 0.05f) dt_s = 0.05f;
    g_lastTickMs = g_currentMillis;

    if (!moving) {
        // Park in the STAND pose so the dog stays upright between
        // commands. Phase decays so the next start is from a known place.
        driveFixedAll(FEMUR_STAND_DEG, TIBIA_STAND_DEG);
        g_phase = wrap01(g_phase * 0.85f);
        return;
    }

    // ---- advance phase clock ---------------------------------------------
    const float speedMag = fmaxf(fabsf(g_xFilt), 0.5f * fabsf(g_yawFilt));
    float hz = HZ_PER_CMD * speedMag;
    if (hz < MIN_HZ) hz = MIN_HZ;
    if (hz > MAX_HZ) hz = MAX_HZ;
    g_phase = wrap01(g_phase + hz * dt_s);

    // Forward gain: sign of xCmd drives direction; magnitude scales stroke.
    float fwdGain = 0.0f;
    if (fabsf(g_xFilt) > deadband) {
        fwdGain = clampF(g_xFilt / 100.0f, -1.0f, 1.0f);
    }

    // Small hip bias when yawing in place: nudge hips outward proportional
    // to yaw demand so the body can pivot. Disabled if not yawing.
    const float yawHipBias = clampF(g_yawFilt / 100.0f, -1.0f, 1.0f) * 8.0f;  // deg

    for (uint8_t slot = 0; slot < 4; ++slot) {
        const float lp = wrap01(g_phase + kLegs[slot].phaseShift);
        LegTarget tgt  = computeLegTarget(lp, fwdGain);

        // Yaw hip bias: FR+BL diagonal share one offset sign, FL+BR the other
        // (matches hipMul pairing, not geometric left/right of the body).
        if (fabsf(yawHipBias) > 0.1f) {
            const bool hipPairFrBl =
                (slot == SLOT_FR || slot == SLOT_BL);
            tgt.hipOffset = (hipPairFrBl ? +1.0f : -1.0f) * yawHipBias;
        }

        driveOneLeg(slot, tgt);
    }
}

}  // namespace gait
