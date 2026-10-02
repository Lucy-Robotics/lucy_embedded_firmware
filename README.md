# Lucy Embedded Firmware

Rust firmware for Lucy RP2040 boards using **Modbus RTU over USB CDC**.

**Architecture:** [docs/architecture/firmware.md](docs/architecture/firmware.md) ·  
**Index:** [`lucy_ws/docs/architecture/README.md`](../../docs/architecture/README.md) ·  
**System overview:** [`lucy_ws/docs/architecture/overview.md`](../../docs/architecture/overview.md)

## Quick start (Pixi)

From the `lucy_ws` root - **no sudo, no manual rustup/PATH config**:

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
| `crates/rp2040_support` | Shared RP2040 bring-up: USB CDC, Modbus poll, PWM / UART / I2C-PWM / ADC banks, `PicoToolReset` |
| `firmwares/rp2040_servo2040` | Pimoroni Servo2040 - thin main; PWM + I2C-PWM + ADC gated by YAML |
| `firmwares/rp2040_bus_servo` | UART bus-servo board (Feetech/STS @ UART0 GPIO0/1, DIR GPIO2) |
| `firmwares/sim` | Host-side Modbus client (dev) |
| `config/` | **Generated** `config_<board_id>.yaml` (gitignored except README) |

## Config

Board YAML is produced by the Lucy config pipeline from the robot package
hardware mapping (e.g. `so_arm101_urdf`) into:

```text
lucy_embedded_firmware/config/config_<board_id>.yaml
```

At build time the pipeline copies that file to the selected crate’s
`config.yaml` (gitignored). Do **not** commit robot configs under `firmwares/`.

Local cargo without the pipeline:

```bash
export LUCY_FIRMWARE_CONFIG=/abs/path/to/config_rp2040_so_arm.yaml
cargo build --release -p lucy_embedded_firmware_rp2040_bus_servo \
  --target thumbv6m-none-eabi
```

Angles in YAML are **radians (float)**; codegen emits **milliradian `u16`**
(`rad × 1000`, `+0.5` cast). `virtual_pin` may be authored by the host so
Modbus bases match `LucySystemHardware`; otherwise the builder assigns densely.

## Register map (build-assigned)

| Driver | Registers |
|--------|-----------|
| PwmServo | 2 - cmd, angle (millirad) |
| BusServo | 3 - id, angle, cmd |
| PressureSensor | 2 - cmd, value (placeholder until ADC is wired) |

## Board class → crate

| `board_class` | Crate package |
|---------------|---------------|
| `internal_servo_only` | `lucy_embedded_firmware_rp2040_servo2040` |
| `internal_servo_i2c_pwm` | `lucy_embedded_firmware_rp2040_servo2040` |
| `bus_servo_only` | `lucy_embedded_firmware_rp2040_bus_servo` |

One physical Servo2040 board → one firmware binary. YAML enables PWM servos,
I2C/PCA9685 channels (`I2C0:PCA9685:N`), and ADC/pressure placeholders
(`ADC0`…). Build-time gates `GENERATED_HAS_PWM` / `HAS_I2C_PWM` / `HAS_ADC` /
`HAS_BUS` select which banks initialize. UART bus servos use the separate
`rp2040_bus_servo` board package (different pinout).

## PWM notes

Pimoroni Servo2040 silk `N` is channel `ServoN` → GPIO `N-1` (1→GPIO0 …
18→GPIO17). Codegen emits `GENERATED_PWM_DEVICES` for **enabled** actuators
only; disabled silk ports are not claimed or driven.

Host `servo_type` is `180` | `270` | `300` only. Milliradian→pulse maps within
`min_angle`..`max_angle` (millirad) into duty counts. Note GPIO0/16 and GPIO1/17
share a PWM slice channel - do not enable both ends of a shared pair.

## Troubleshooting

- `cargo not found` → `pixi run firmware-setup`
- Linux serial permission → `sudo usermod -aG dialout $USER` then re-login
- Flash verify fails → check USB cable / BOOTSEL / Modbus slave address
