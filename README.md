# Lucy Embedded Firmware

Rust firmware for Lucy RP2040 boards using **Modbus RTU over USB CDC**.

## Quick start (Pixi)

From the `lucy_ws` root:

```bash
pixi run firmware-setup   # installs rustup (if needed), thumbv6m target, elf2uf2-rs
pixi run firmware-build
pixi run firmware-test
pixi run firmware-flash
```

## Layout

| Path | Role |
|------|------|
| `crates/lucy_embedded_firmware_core` | `no_std` Modbus + drivers + `BoardLayout` |
| `crates/builder` | YAML → `$OUT_DIR/config.rs` codegen (auto virtual pins) |
| `firmwares/rp2040_internal_pwm` | On-board PWM / Servo2040 (`Servo1..18` → GPIO0..17) |
| `firmwares/rp2040_bus_servo` | UART bus-servo stub |
| `firmwares/rp2040_i2c_pwm` | Internal PWM + I2C/PCA9685 stub |
| `firmwares/sim` | Host-side Modbus client (dev) |

## Config

Place architecture-shaped `config.yaml` in the selected board crate (or let the
Lucy config pipeline install `config_<board>.yaml`). `build.rs` calls
`builder::build_config` when the file contains `actuators:`.

Angles in YAML are **radians (float)**; codegen emits **milliradian `u16`**
(`rad × 1000`, `+0.5` cast). `virtual_pin` is **not** authored in YAML — the
builder assigns contiguous Modbus blocks from enabled actuators then sensors.

## Register map (build-assigned)

| Driver | Registers |
|--------|-----------|
| PwmServo | 2 — cmd, angle (millirad) |
| BusServo | 3 — id, angle, cmd |
| PressureSensor | 2 — cmd, value |

## Board class → crate

| `board_class` | Crate package |
|---------------|---------------|
| `internal_servo_only` | `lucy_embedded_firmware_rp2040_internal_pwm` |
| `bus_servo_only` | `lucy_embedded_firmware_rp2040_bus_servo` |
| `internal_servo_i2c_pwm` | `lucy_embedded_firmware_rp2040_i2c_pwm` |

## PWM notes

Pimoroni Servo2040 silk `N` is channel `ServoN` → GPIO `N-1` (1→GPIO0 …
18→GPIO17). Codegen emits `GENERATED_PWM_DEVICES` for **enabled** actuators
only; disabled silk ports are not claimed or driven.

Milliradian→pulse maps within `min_angle`..`max_angle` (millirad) into duty
counts. Unit tests cover midpoints and half-step rounding (`no_std`, no
`f32::round`). Note GPIO0/16 and GPIO1/17 share a PWM slice channel — do not
enable both ends of a shared pair.

## Troubleshooting

- `cargo not found` → `pixi run firmware-setup`
- Linux serial permission → `sudo usermod -aG dialout $USER` then re-login
- Flash verify fails → check USB cable / BOOTSEL / Modbus slave address
