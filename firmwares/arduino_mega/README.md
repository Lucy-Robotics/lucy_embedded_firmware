# Arduino Mega 2560 firmware

One Mega per board of the robot hardware YAML (`board_class: i2c_pwm_only`). Servos hang off
PCA9685s chained on the Mega's I2C bus (D20 SDA / D21 SCL).

| Cargo feature (`compile_definition`) | Board (inmoov_i2_urdf) | PCA9685 addresses | Servos |
|---|---|---|---|
| `torso_head` | `mega_torso_head` | `0x41` | 12 |
| `arms` | `mega_arms` | `0x40`, `0x41` | 18 |

Outputs are numbered across chips: `physical_pin` 1-16 in the YAML is the first PCA's
channels 0-15, 17-32 the second's. The tables in `src/config/` mirror the YAML by hand
(slot = `virtual_pin`), and the build fails if a limit leaves the 0..180 deg servo travel or
two slots share an output.

## Build and flash

From the workspace root, with `avr-gcc` and `avrdude` on the host:

```sh
pixi run firmware-flash-mega mega_arms   # builds with the board's feature, flashes its serial_id
```

Or by hand, from this directory: `cargo build --release --features arms`.

## Run

One bridge per board, matched by the Mega's USB serial number (`serial_id`):

```sh
lucy-embedded-firmware-mcu-bridge torso_head <serial_id>
lucy-embedded-firmware-mcu-bridge arms <serial_id>
```
