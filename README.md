# Glyph C6 wired control firmware

PlatformIO firmware for the [PCB Cupid Glyph C6](https://shop.pcbcupid.com/product/gd002), an ESP32-C6 development board. It uses the board's USB connection as a wired serial console for live input status and manual control of two external stepper drivers. The firmware does not start Wi-Fi.

## Requirements

- Visual Studio Code with the PlatformIO IDE extension, or PlatformIO Core
- A USB data cable

## Build and upload

Open this folder in VS Code, then use the PlatformIO actions for the `glyph-c6` environment:

1. **Build** to compile the firmware.
2. Connect the board over USB and select its serial port if prompted.
3. **Upload** to flash the firmware.
4. Open the **Serial Monitor** at 115200 baud and set its line ending to **Newline**.

PlatformIO is configured in [`platformio.ini`](platformio.ini) for the `esp32-c6-devkitc-1` board target using the native ESP-IDF framework.

## USB serial controls

Connect the board to a computer with a USB data cable, open the PlatformIO Serial Monitor (or another serial terminal) at **115200 baud**, and send commands terminated by a newline. Sensor and limit state changes are printed automatically.

| Command | Action |
|---|---|
| `help` | Show available commands |
| `status` | Print current sensor, limit, motor, and SSR state |
| `jog 1 f 500 100` | Jog axis 1 forward for 500 ms at 100 steps/s |
| `jog 2 r 1000 300` | Jog axis 2 in reverse for 1000 ms at 300 steps/s |
| `accel 2 8000` | Set axis 2 acceleration to 8000 steps/s² |
| `stop 1` | Stop axis 1 immediately |
| `stop all` | Stop both axes immediately |
| `ssr on` / `ssr off` | Set the SSR output |

Jog duration is limited to 1–5000 ms, pulse rate to 50–20000 steps/s, and acceleration to 100–200000 steps/s² (default 8000). Motors ramp up to the requested speed, and a direction change while running slows to 200 steps/s, reverses, then speeds back up. Sending `jog` again before it ends extends it and changes speed or direction without restarting the ramp; this is how the control panel runs a motor continuously. `stop` and the end of a jog stop the motor immediately. Each axis has its own speed and direction, and the two axes run independently of each other and of the sensors.

## Control panel

[`dashboard/index.html`](dashboard/index.html) is a browser control panel that talks to the board over the same USB serial port. Open it in desktop Chrome or Edge, click **Connect board**, and choose the Espressif USB JTAG/serial port. Close any other serial monitor first. Each axis has its own speed (in RPM and steps/s), direction, continuous run, hold-to-move and timed moves, plus driver settings for pulses per revolution and acceleration. It also provides SSR on/off, and live TRUE/FALSE feedback for the IR sensors and limit switches.

## Pin assignments

GPIO numbers and printed labels are based on the [Glyph C6 pinout documentation](https://learn.pcbcupid.com/documentation/modules/glyph/glyph-esp32c6/glyphc6-pinouts).

| Function | Board label | GPIO |
|---|---:|---:|
| T86S STEP/PUL logic output | D6 | 6 |
| T86S DIR logic output | D7 | 7 |
| Second driver STEP/PUL logic output | D14 | 14 |
| Second driver DIR logic output | D19 | 19 |
| SSR 1 control | D18 | 18 |
| IR sensor 1 digital output | A0 | 0 |
| IR sensor 2 digital output | A1 | 1 |
| Limit switch 1 | A2 | 2 |
| Limit switch 2 | A3 | 3 |

Inputs use internal pull-ups. With normally-closed dry-contact switches wired between the assigned GPIO and GND, LOW is normal and HIGH is tripped. Verify the switch wiring and state before enabling motor power. Sensor values are displayed as raw logic levels because sensor output polarity depends on the module.

## Electrical safety

- The ESP32-C6 GPIOs are 3.3 V only. Do not connect 5 V, 12 V, or 24 V signals directly to a GPIO.
- STEP/DIR pins are logic outputs, not motor-driver power outputs. The T86S requires the correct 5 V input interface; use a compatible transistor/opto interface between GPIO6/7 and the T86S pulse/direction inputs. Choose an interface appropriate for the second driver as well.
- D18 is set LOW by firmware and HIGH when the SSR is enabled. Confirm the SSR input is compatible with 3.3 V and the selected polarity; add a driver stage if needed. Use hardware pull-down/driver circuitry that keeps the SSR off while the C6 is resetting or unpowered. Keep mains wiring isolated and properly enclosed.
- GPIO outputs and the board USB 5 V pin cannot power motors. Use each motor driver's correctly rated motor supply.
- The IR sensors and limit switches are reported for feedback only. The firmware does not stop or block either motor when a sensor or limit switch triggers, so a jog runs for its full duration regardless of their state. Keep jogs short near the ends of travel, and use a hardware limit or E-stop circuit if the mechanism can be damaged by over-travel.
- Test with motor power disabled first. Secure the mechanisms, keep clear of moving parts, and have a physical way to remove motor power.
