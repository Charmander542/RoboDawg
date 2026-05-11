# RoboDawg

ESP32 / PlatformIO firmware for a quadruped robot dog driven by 12 hobby
servos on a PCA9685 plus 4 wheel ESCs.

This is the ESP32 port of the original openDogV3 Teensy + ODrive
codebase. The inverse kinematics are unchanged; only the motor
backend (`driveJoints` -> `driveServo` / `driveWheel`) and the control
interface (nRF24 + LCD menu -> USB Serial ASCII protocol) have been
replaced.

## Hardware

| PCA9685 channels | Hardware                              |
| ---------------- | ------------------------------------- |
| `0` `1` `2` `3`  | Hip servos:   FR, FL, BR, BL          |
| `4` `5` `6` `7`  | Thigh servos: FR, FL, BR, BL          |
| `8` `9` `10` `11`| Shin/knee servos: FR, FL, BR, BL      |
| `12` `13` `14` `15`| Wheel ESCs:   FR, FL, BR, BL        |

I2C wiring (default ESP32 pins, override with `-DPIN_SDA=` / `-DPIN_SCL=`):

| Signal | ESP32 pin |
| ------ | --------- |
| SDA    | GPIO 21   |
| SCL    | GPIO 22   |
| 5 V    | external BEC (do **not** power the PCA9685 V+ rail off the ESP32) |

## Build & flash

```
pio run                # compile
pio run -t upload      # flash
pio device monitor     # 115200 baud serial monitor
```

PlatformIO will pull these libraries automatically (see `platformio.ini`):

- `Adafruit PWM Servo Driver Library` ^3.0.2
- `Adafruit BusIO` ^1.16.1
- `Preferences` (bundled with the ESP32 Arduino core)

## Serial protocol

All commands are ASCII lines terminated with `\n`, parsed
non-blockingly in the main loop. Numbers may be integer or float.

| Command | Args | Effect |
| ------- | ---- | ------ |
| `WALK x y yaw`               | -100..+100 each | Switch to walk mode and update velocity targets |
| `POSE roll pitch yaw height` | deg, deg, deg, mm | Switch to pose mode and hold body pose |
| `WHEEL fl fr bl br`          | -100..+100 each | Set the four wheel ESC speeds |
| `SERVO ch angle`             | ch=0..15, deg | Drive one PCA9685 channel directly (calibration mode) |
| `TRIM ch offset_us`          | ch=0..15, µs | Adjust trim offset, persist to NVS |
| `STOP`                       | — | Zero all outputs immediately, go to idle |
| `STATUS`                     | — | Print joint angles, pose, loop timing, full cal table |
| `HELP` / `?`                 | — | Print help |

Replies are `OK ...` for success or `ERR ...` for parse / range errors.

Example session:

```
HELP
POSE 0 0 0 330
WALK 50 0 0
WHEEL 80 80 80 80
SERVO 4 95
TRIM 4 -120
STOP
STATUS
```

## Per-servo calibration

Each of the 16 PCA9685 channels has an independent calibration record
stored in NVS (ESP32 `Preferences`):

```c
struct ServoCal {
    uint16_t minPulse;    // µs at minAngle
    uint16_t maxPulse;    // µs at maxAngle
    int16_t  trimOffset;  // µs added to every pulse
    float    minAngle;    // ° corresponding to minPulse
    float    maxAngle;    // ° corresponding to maxPulse
};
```

Use `TRIM ch offset_us` to nudge individual servos to their mechanical
zero without recompiling. The change is written to NVS immediately.

`driveWheel(channel, speed)` maps `-100..+100` linearly across
`[minPulse..maxPulse]` with neutral at the midpoint, so the same
calibration record works for the ESCs on channels 12..15.

## Project layout

```
RoboDawg/
├─ platformio.ini
├─ README.md
├─ include/
│  ├─ config.h           pin map, channel map, debug macros
│  ├─ state.h            global RobotState
│  ├─ servo_driver.h     PCA9685 + calibration API
│  ├─ interpolation.h    Ramp-style helpers
│  ├─ kinematics.h       IK + per-leg servo map
│  ├─ gait.h             gait state machine
│  └─ serial_cmd.h       USB Serial protocol
└─ src/
   ├─ main.cpp           setup / 100 Hz control loop
   ├─ servo_driver.cpp
   ├─ kinematics.cpp     (math identical to original kinematics_new.ino)
   ├─ gait.cpp           (ported from openDogV3 runMode == 2)
   └─ serial_cmd.cpp
```

## Compile-time options

Edit `platformio.ini`:

- `-DROBODAWG_DEBUG=1` — enable boot / calibration debug prints.
  Comment out to compile every `DBG_PRINT*` call out of the binary.
  (Internally this turns on a local `DEBUG_SERIAL` macro guarded inside
  `config.h`; the build flag itself uses a different name because
  Adafruit BusIO claims the `DEBUG_SERIAL` symbol globally and expects
  it to expand to a `Serial`-class object.)
- `-DPIN_SDA=`, `-DPIN_SCL=` — override the I2C pins.

## Notes on porting

- `delay()` is never used. The main loop is gated on `millis()` to run
  at 100 Hz; all internal timers (gait step timer, interpolation
  settling, serial input) use `millis()` deltas.
- Joint angle math is unchanged from the supplied `kinematics_new.ino`.
  Only the final `driveJoints(id, counts)` calls were replaced with
  `driveServo(channel, neutralDeg + dir * jointAngleDeg)`.
- The original interpolation settling window (300 ms after mode change)
  is preserved via `g_previousInterpMillis` / `g_interpFlag`.
- The radio (`nRF24L01`) and `LiquidCrystal_I2C` LCD have been removed;
  the same diagnostic info is available through `STATUS`.
- There is no IMU integration in the original openDogV3 source, so
  none was ported. Adding one is straightforward — read it in the
  control loop and fold the angles into `g_state.poseRoll` /
  `g_state.posePitch` before `gait::poseTick()` runs.
