# Lucy Embedded Firmware

Rust firmware for Lucy RP2040 boards using **Modbus RTU over USB CDC**.

## Quick start (Pixi)

From the `lucy_ws` root — **no sudo, no manual rustup/PATH config**:

```bash
pixi run firmware-setup   # once (or whenever the Pixi env is recreated)
pixi run firmware-build
pixi run firmware-test
pixi run firmware-flash
```

`firmware-build` / `firmware-test` / `firmware-flash` already depend on
`firmware-setup`, so a single `pixi run firmware-build` is enough after clone.

Toolchain files live inside the Pixi env (`$CONDA_PREFIX/cargo` +
`$CONDA_PREFIX/rustup`), activated automatically via
`scripts/firmware_cargo_env.sh` (`.bat` on Windows).

## Layout

| Path | Role |
|------|------|
| `crates/lucy_embedded_firmware_core` | `no_std` Modbus + drivers + `BoardLayout` |
| `crates/builder` | YAML → `$OUT_DIR/config.rs` codegen (auto virtual pins) |
| `crates/rp2040_support` | Shared RP2040 USB helpers (`PicoToolReset`) |
| `firmwares/rp2040_servo2040` | Pimoroni Servo2040 — PWM (+ optional I2C/ADC/UART via YAML) |
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
| PwmServo | 2 — cmd, angle (millirad) |
| BusServo | 3 — id, angle, cmd |
| PressureSensor | 2 — cmd, value (placeholder until ADC is wired) |

## Board class → crate

| `board_class` | Crate package |
|---------------|---------------|
| `internal_servo_only` | `lucy_embedded_firmware_rp2040_servo2040` |
| `internal_servo_i2c_pwm` | `lucy_embedded_firmware_rp2040_servo2040` |
| `bus_servo_only` | `lucy_embedded_firmware_rp2040_bus_servo` |

One physical Servo2040 board → one firmware binary. YAML enables PWM servos,
pressure-sensor placeholders, and (later) I2C/UART peripherals.

## PWM notes

Pimoroni Servo2040 silk `N` is channel `ServoN` → GPIO `N-1` (1→GPIO0 …
18→GPIO17). Codegen emits `GENERATED_PWM_DEVICES` for **enabled** actuators
only; disabled silk ports are not claimed or driven.

Host `servo_type` is `180` | `270` | `300` only. Milliradian→pulse maps within
`min_angle`..`max_angle` (millirad) into duty counts. Note GPIO0/16 and GPIO1/17
share a PWM slice channel — do not enable both ends of a shared pair.

## Troubleshooting

- `cargo not found` → `pixi run firmware-setup`
- Linux serial permission → `sudo usermod -aG dialout $USER` then re-login
- Flash verify fails → check USB cable / BOOTSEL / Modbus slave address
