#include "kinematics.h"

#include "interpolation.h"
#include "servo_driver.h"
#include "state.h"

// ============================================================
// PER-LEG SERVO MAPPING (IK / POSE)
// ============================================================
// Index by openDog leg id (1..4). Slot 0 is unused.
//
// **Leg 1 = FR (front right)** is the reference for thighDir / shinDir.
// **Hip:** FR and BL use the same hipDir (+1); FL and BR use the opposite
// (−1), matching the gait `kLegs[]` hipMul pairing. Thigh/shin still
// follow left‑ vs right‑leg / front‑ vs rear‑leg IK conventions below.
//
// For each leg, the IK function emits three joint angles (degrees):
//   hipAngle1Degrees   : hip abduction
//   shoulderAngleDegrees = (shoulderAngle1Degrees - 45) ± shoulderAngle2Degrees
//                       (+ for front legs, - for back legs)
//   kneeAngleDegrees - 90
//
// Physical servo command is:
//   servoDeg = neutralDeg + direction * jointAngleDeg
//
// Neutrals are SERVO_NEUTRAL_DEG (mid of 0..270° leg servos). Sign flips
// and trim are applied via the per-channel calibration table in NVS.

LegServoMap g_legMap[5] = {
    // index 0: unused
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, +1.0f},

    // leg 1 : front right (slot FR)
    {  /*hipCh*/   hipChannel  (LEG_FR),
       /*thighCh*/ thighChannel(LEG_FR),
       /*shinCh*/  shinChannel (LEG_FR),
       /*wheelCh*/ wheelChannel(LEG_FR),
       /*neutrals*/ SERVO_NEUTRAL_DEG, SERVO_NEUTRAL_DEG, SERVO_NEUTRAL_DEG,
       /*dirs    */ +1.0f, +1.0f, +1.0f,
       /*shoulderCombineSign*/ +1.0f },

    // leg 2 : front left (slot FL)
    {  hipChannel  (LEG_FL),
       thighChannel(LEG_FL),
       shinChannel (LEG_FL),
       wheelChannel(LEG_FL),
       SERVO_NEUTRAL_DEG, SERVO_NEUTRAL_DEG, SERVO_NEUTRAL_DEG,
       -1.0f, -1.0f, +1.0f,    // mirror hip/thigh on the left side
       +1.0f },

    // leg 3 : back left (slot BL) — hip same sign as FR; thigh like FL
    {  hipChannel  (LEG_BL),
       thighChannel(LEG_BL),
       shinChannel (LEG_BL),
       wheelChannel(LEG_BL),
       SERVO_NEUTRAL_DEG, SERVO_NEUTRAL_DEG, SERVO_NEUTRAL_DEG,
       +1.0f, -1.0f, +1.0f,
       -1.0f },                 // rear legs combine with -

    // leg 4 : back right (slot BR) — hip same sign as FL; thigh like FR
    {  hipChannel  (LEG_BR),
       thighChannel(LEG_BR),
       shinChannel (LEG_BR),
       wheelChannel(LEG_BR),
       SERVO_NEUTRAL_DEG, SERVO_NEUTRAL_DEG, SERVO_NEUTRAL_DEG,
       -1.0f, +1.0f, +1.0f,
       -1.0f },
};

// ============================================================
// INTERPOLATION OBJECTS (definitions for the externs)
// ============================================================
Interpolation interpFRX, interpFRY, interpFRZ, interpFRT;
Interpolation interpFLX, interpFLY, interpFLZ, interpFLT;
Interpolation interpBRX, interpBRY, interpBRZ, interpBRT;
Interpolation interpBLX, interpBLY, interpBLZ, interpBLT;

// ============================================================
// IK FUNCTION (math identical to the supplied kinematics_new.ino)
// ============================================================
void kinematics(int leg, float xIn, float yIn, float zIn,
                float roll, float pitch, float yawIn,
                int interOn, int dur) {

    const uint8_t slotEarly = legToSlot((uint8_t)leg);
    if ((g_state.legOutputMask & (1u << slotEarly)) == 0) {
        const LegServoMap& m0 = g_legMap[leg];
        servoDriver::driveServo(m0.hipCh,   SERVO_NEUTRAL_DEG);
        servoDriver::driveServo(m0.thighCh, SERVO_NEUTRAL_DEG);
        servoDriver::driveServo(m0.shinCh,  SERVO_NEUTRAL_DEG);
        g_state.jointHip[slotEarly]   = 0.0f;
        g_state.jointThigh[slotEarly] = 0.0f;
        g_state.jointShin[slotEarly]  = 0.0f;
        return;
    }

    // ------------------------------------------------------------
    // ROBOT PARAMETERS
    // ------------------------------------------------------------
    constexpr float hipOffset   = 3.0f;       // hip pivot -> leg plane (mm), inward
    constexpr float bodyWidth   = 45.0f;      // half hip-pivot lateral spacing (mm)
    constexpr float bodyLength  = 79.5f;      // half hip-pivot longitudinal spacing (mm)
    constexpr float thighLength = 55.0f;      // hip pivot -> knee pivot (mm)
    constexpr float shinLength  = 120.61f;    // knee pivot -> wheel axle (mm)
    constexpr float wheelRadius = 30.0f;      // axle -> ground contact (mm)

    // ------------------------------------------------------------
    // INTERPOLATION
    // ------------------------------------------------------------
    // Pre-initialise so the compiler can prove we always have a value
    // before the wheel-radius / yaw maths below; the interpFlag block
    // overwrites these when appropriate.
    float x = xIn, y = yIn, z = zIn, yaw = yawIn;

    if (interOn == 1) {
        if (leg == LEG_FR) {
            z   = interpFRZ.go(zIn, dur);
            x   = interpFRX.go(xIn, dur);
            y   = interpFRY.go(yIn, dur);
            yaw = interpFRT.go(yawIn, dur);
        } else if (leg == LEG_FL) {
            z   = interpFLZ.go(zIn, dur);
            x   = interpFLX.go(xIn, dur);
            y   = interpFLY.go(yIn, dur);
            yaw = interpFLT.go(yawIn, dur);
        } else if (leg == LEG_BR) {
            z   = interpBRZ.go(zIn, dur);
            x   = interpBRX.go(xIn, dur);
            y   = interpBRY.go(yIn, dur);
            yaw = interpBRT.go(yawIn, dur);
        } else /* LEG_BL */ {
            z   = interpBLZ.go(zIn, dur);
            x   = interpBLX.go(xIn, dur);
            y   = interpBLY.go(yIn, dur);
            yaw = interpBLT.go(yawIn, dur);
        }
    }

    // Wait for filters to settle before using interpolated values
    if (g_interpFlag == 0) {
        z = zIn; x = xIn; y = yIn; yaw = yawIn;
        if (g_currentMillis - g_previousInterpMillis >= 300) {
            g_interpFlag = 1;
        }
    } else if (g_interpFlag == 1 && interOn == 0) {
        z = zIn; x = xIn; y = yIn; yaw = yawIn;
    }

    // ------------------------------------------------------------
    // WHEEL RADIUS OFFSET
    // ------------------------------------------------------------
    z = z - wheelRadius;

    // ------------------------------------------------------------
    // YAW AXIS
    // ------------------------------------------------------------
    float yawAngle = (PI / 180.0f) * yaw;

    if (leg == LEG_FR) { y +=  (bodyWidth + hipOffset); x -= bodyLength; }
    else if (leg == LEG_FL) { y -=  (bodyWidth + hipOffset); x -= bodyLength; }
    else if (leg == LEG_BL) { y -=  (bodyWidth + hipOffset); x += bodyLength; }
    else if (leg == LEG_BR) { y +=  (bodyWidth + hipOffset); x += bodyLength; }

    float existingAngle = atan2(y, x);
    float radius        = sqrtf(x*x + y*y);
    float demandYaw     = existingAngle + yawAngle;

    float xx3 = radius * cosf(demandYaw);
    float yy3 = radius * sinf(demandYaw);

    if (leg == LEG_FR) { yy3 -=  (bodyWidth + hipOffset); xx3 += bodyLength; }
    else if (leg == LEG_FL) { yy3 +=  (bodyWidth + hipOffset); xx3 += bodyLength; }
    else if (leg == LEG_BL) { yy3 +=  (bodyWidth + hipOffset); xx3 -= bodyLength; }
    else if (leg == LEG_BR) { yy3 -=  (bodyWidth + hipOffset); xx3 -= bodyLength; }

    // ------------------------------------------------------------
    // PITCH AXIS
    // ------------------------------------------------------------
    if (leg == LEG_FR || leg == LEG_FL) {
        pitch = -pitch;
        xx3   = -xx3;
    }

    float pitchAngle  = (PI / 180.0f) * pitch;
    float legDiffPitch  = sinf(pitchAngle) * bodyLength;
    float bodyDiffPitch = cosf(pitchAngle) * bodyLength;
    legDiffPitch = z - legDiffPitch;

    float footDisplacementPitch      = ((bodyDiffPitch - bodyLength) * -1.0f) + xx3;
    float footDisplacementAnglePitch = atanf(footDisplacementPitch / legDiffPitch);

    float zz2a = legDiffPitch / cosf(footDisplacementAnglePitch);
    float footWholeAnglePitch = footDisplacementAnglePitch + pitchAngle;

    float zz2 = cosf(footWholeAnglePitch) * zz2a;
    float xx1 = sinf(footWholeAnglePitch) * zz2a;

    if (leg == LEG_FR || leg == LEG_FL) {
        xx1 = -xx1;
    }

    // ------------------------------------------------------------
    // ROLL AXIS
    // ------------------------------------------------------------
    if (leg == LEG_FL || leg == LEG_BL) {   // left side
        roll = -roll;
        yy3  = -yy3;
    }
    // (right side keeps roll as-is)

    float rollAngle    = (PI / 180.0f) * roll;
    float legDiffRoll  = sinf(rollAngle) * bodyWidth;
    float bodyDiffRoll = cosf(rollAngle) * bodyWidth;
    legDiffRoll = zz2 - legDiffRoll;

    float footDisplacementRoll      = (((bodyDiffRoll - bodyWidth) * -1.0f) - hipOffset) - yy3;
    float footDisplacementAngleRoll = atanf(footDisplacementRoll / legDiffRoll);

    float zz1a = legDiffRoll / cosf(footDisplacementAngleRoll);
    float footWholeAngleRoll = footDisplacementAngleRoll + rollAngle;

    float zz1 = cosf(footWholeAngleRoll) * zz1a;
    float yy1 = (sinf(footWholeAngleRoll) * zz1a) + hipOffset;

    // ------------------------------------------------------------
    // HIP ABDUCTION
    // ------------------------------------------------------------
    float hipOffsetSigned = hipOffset;
    if (leg == LEG_FR || leg == LEG_BR) {
        hipOffsetSigned =  hipOffset;
        yy1 = -yy1;
    } else {
        hipOffsetSigned = -hipOffset;
    }

    yy1 += hipOffsetSigned;

    float hipAngle1a       = atanf(yy1 / zz1);
    float hipHyp           = zz1 / cosf(hipAngle1a);
    float hipAngle1b       = asinf(hipOffset / hipHyp);
    float hipAngle1        = (PI - (PI / 2.0f) - hipAngle1b) + hipAngle1a;
    hipAngle1              = hipAngle1 - 1.5708f;
    float hipAngle1Degrees = hipAngle1 * (180.0f / PI);

    float z2 = hipOffset / tanf(hipAngle1b);

    // ------------------------------------------------------------
    // SHOULDER FORWARD/BACK
    // ------------------------------------------------------------
    float shoulderAngle2        = atanf(xx1 / z2);
    float shoulderAngle2Degrees = shoulderAngle2 * (180.0f / PI);
    float z3                    = z2 / cosf(shoulderAngle2);

    // ------------------------------------------------------------
    // KNEE + SHOULDER (sagittal plane IK)
    // ------------------------------------------------------------
    z3 = constrain(z3, 70.0f, 174.0f);

    float shoulderAngle1a = sq(thighLength) + sq(z3) - sq(shinLength);
    float shoulderAngle1b = 2.0f * thighLength * z3;
    float shoulderAngle1c = shoulderAngle1a / shoulderAngle1b;
    float shoulderAngle1  = acosf(shoulderAngle1c);

    float kneeAngle       = PI - (shoulderAngle1 * 2.0f);

    float shoulderAngle1Degrees = shoulderAngle1 * (180.0f / PI);
    float kneeAngleDegrees      = kneeAngle      * (180.0f / PI);

    // ------------------------------------------------------------
    // WRITE TO SERVOS
    // ------------------------------------------------------------
    // Compose joint angles (degrees) using the same offsets the original
    // openDog code applied before scaling to ODrive turns:
    //   shoulder = (shoulderAngle1Degrees - 45) ± shoulderAngle2Degrees
    //   knee     =  kneeAngleDegrees - 90
    //   hip      =  hipAngle1Degrees
    const LegServoMap& m = g_legMap[leg];

    float shoulderJointDeg = (shoulderAngle1Degrees - 45.0f)
                           + m.shoulderCombineSign * shoulderAngle2Degrees;
    float kneeJointDeg     = kneeAngleDegrees - 90.0f;
    float hipJointDeg      = hipAngle1Degrees;

    float hipCmd   = m.hipNeutral   + m.hipDir   * hipJointDeg;
    float thighCmd = m.thighNeutral + m.thighDir * shoulderJointDeg;
    float shinCmd  = m.shinNeutral  + m.shinDir  * kneeJointDeg;

    servoDriver::driveServo(m.hipCh,   hipCmd);
    servoDriver::driveServo(m.thighCh, thighCmd);
    servoDriver::driveServo(m.shinCh,  shinCmd);

    // Cache last commands for STATUS reporting
    uint8_t slot = legToSlot(leg);
    g_state.jointHip  [slot] = hipJointDeg;
    g_state.jointThigh[slot] = shoulderJointDeg;
    g_state.jointShin [slot] = kneeJointDeg;
}
