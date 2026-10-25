# Arduino Mega 2560 firmware

One Mega per `i2c_pwm_only` board of the robot hardware YAML. Servos hang off the PCA9685s
chained on the Mega's I2C bus (D20 SDA / D21 SCL), listed in the board's `pca_addresses`.

The board tables (PCA addresses, per-slot limits, which slot drives which output) are not in
this crate: `lucy_config_generator` writes them from the YAML as `config_<board>.rs`, and the
build includes the file named by `LUCY_MEGA_BOARD_CONFIG`. Outputs are numbered across chips:
`physical_pin` 1-16 is the first address's channels 0-15, 17-32 the second's. Only enabled
actuators are driven, and the build fails if a limit leaves the 0..180 deg servo travel or two
driven slots share an output.

## Build and flash

From the LCP: activate a configuration with the board selected; the config pipeline generates,
builds and flashes it. From the workspace root:

```sh
pixi run firmware-flash-mega mega_right   # --robot <pkg>, --port <dev>, --build-only
```

Both need `avr-gcc` and `avrdude` on the host and find the Mega by its USB serial (`serial_id`).

## Run

`lucy.launch.py real:=true` starts one bridge per Mega board that has a `serial_id`:

```sh
lucy-embedded-firmware-mcu-bridge <node_name> <serial_id>   # e.g. right 1453230323235110C0D0
```

While a board is flashed, `/tmp/.lucy_mcu_flashing_<serial_id>` exists and its bridge waits.
