#pragma once

#include <Arduino.h>
#include "config.h"

// Per-leg servo mapping table. Defined in kinematics.cpp.
extern LegServoMap g_legMap[5];   // indexed 1..4 (slot 0 unused)

// Inverse kinematics for one leg. Identical math to the supplied
// kinematics_new.ino, with every driveJoints() call replaced by
// driveServo() that targets the PCA9685 channels in `g_legMap`.
//
// leg : 1=FR, 2=FL, 3=BL, 4=BR
// xIn, yIn, zIn : foot position relative to hip pivot (mm)
// roll, pitch, yawIn : body orientation (degrees)
// interOn : 1 to enable per-axis interpolation
// dur     : interpolation duration (ms)
void kinematics(int  leg,
                float xIn, float yIn, float zIn,
                float roll, float pitch, float yawIn,
                int  interOn, int dur);
