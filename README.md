# Lucy Embedded Firmware

[![CI: dev](https://github.com/Lucy-Robotics/lucy_embedded_firmware/actions/workflows/build.yml/badge.svg?branch=dev)](https://github.com/Lucy-Robotics/lucy_embedded_firmware/actions/workflows/build.yml?query=branch%3Adev)
[![CI: master](https://github.com/Lucy-Robotics/lucy_embedded_firmware/actions/workflows/build.yml/badge.svg?branch=master)](https://github.com/Lucy-Robotics/lucy_embedded_firmware/actions/workflows/build.yml?query=branch%3Amaster)
[![codecov](https://codecov.io/gh/Lucy-Robotics/lucy_embedded_firmware/branch/master/graph/badge.svg)](https://codecov.io/gh/Lucy-Robotics/lucy_embedded_firmware)
[![license](https://img.shields.io/github/license/Lucy-Robotics/lucy_embedded_firmware)](LICENSE)
[![Discord](https://img.shields.io/discord/1272611839775084564?label=Discord&logo=discord&logoColor=white&color=5865F2)](https://discord.com/invite/g4KNZ3eeBd)

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
| `crates/lucy_embedded_firmware_core` | `no_std` Modbus + PWM servo drivers |
| `crates/builder` | YAML → `config.rs` codegen |
| `firmwares/rp2040` | Bare-metal RP2040 binary |
| `firmwares/sim` | Host-side Modbus client (dev) |

## Config

Place `firmwares/rp2040/config.yaml` (or let the Lucy config pipeline install
`config_<board>.yaml` under `config/`). `build.rs` calls `builder::build_config`.

## Register map

Per actuator at `virtual_pin * 2`:

| Offset | Meaning |
|--------|---------|
| 0 | cmd (`1` = move, `2` = reset) |
| 1 | angle **milliradians** (`rad × 1000`) |

Config `default_angle` / `min_angle` / `max_angle` remain in **degrees**; the driver converts on reset and when mapping pulses.

## PWM notes

Milliradian→pulse uses rounded mapping into duty counts (default 1250–2500 ≈ 1–2 ms at 50 Hz).
Unit tests cover 180° / 270° / 300° midpoints and half-step rounding.

## Troubleshooting

- `cargo not found` → `pixi run firmware-setup`
- Linux serial permission → `sudo usermod -aG dialout $USER` then re-login
- Flash verify fails → check USB cable / BOOTSEL / Modbus slave address
