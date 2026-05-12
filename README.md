# RoboDawg

ESP32 quadruped firmware (PCA9685 servos + wheel ESCs), PlatformIO.

## Build

```text
pio run -e esp32dev              # default: USB serial only, no Bluepad32
pio run -e esp32dev_gamepad      # Bluetooth gamepad (Bluepad32 Arduino core)
pio run -t upload -e esp32dev_gamepad
pio device monitor
```

## Bluetooth gamepad (Bluepad32)

This follows the approach in [Connect Your Game Controller to an ESP32](https://racheldebarros.com/esp32-projects/connect-your-game-controller-to-an-esp32/) (Rachel De Barros): pair a modern Bluetooth controller using **Bluepad32**.

On **Arduino IDE**, you install the **ESP32 + Bluepad32** board package from the URLs in that article. On **PlatformIO**, the stock `framework-arduinoespressif32` package does not include Bluepad32, so this repo adds a second environment `esp32dev_gamepad` that swaps the framework using the community workaround described [here](https://community.platformio.org/t/use-bluepad32-library-in-pio/46745/4) (`maxgerhardt/pio-framework-bluepad32`).

After flashing `esp32dev_gamepad`, open the serial monitor (115200), press **EN** on the board if needed, then put your controller in pairing mode. When connected, the firmware maps:

| Control | Robot |
| ------- | ----- |
| Left stick X / Y | `WALK` strafe / forward-back (scaled to ±100, deadzone 25) |
| Right stick X | `WALK` yaw |
| Right stick Y | All four wheel ESCs same speed (±100) |

While `POSE` or `SERVO` mode is active, stick input is ignored so calibration / static poses are not disturbed. On disconnect, walk and wheels are zeroed and outputs stop (same idea as the original radio timeout).

Serial commands `GAMEPADDUMP 1` / `GAMEPADDUMP 0` toggle a compact raw dump (like the tutorial’s `dumpGamepad` helper) for mapping buttons on new pads.

## Serial protocol

115200 baud, line-based: `WALK`, `POSE`, `WHEEL`, `SERVO`, `TRIM`, `STOP`, `STATUS`, `HELP`, `GAMEPADDUMP`.

## Notes

- Default `esp32dev` build compiles **stub** gamepad code (`gamepadBegin` / `gamepadPoll` are no-ops) so you do not need Bluepad32 unless you want wireless control.
- `esp32dev_gamepad` uses `espressif32@6.10.0` plus the Bluepad32 framework override; RAM/flash use is higher than the default build.
- We do **not** call `BP32.forgetBluetoothKeys()` in `setup()` so bonded controllers can reconnect; clear keys only when debugging pairing (see Bluepad32 docs).
