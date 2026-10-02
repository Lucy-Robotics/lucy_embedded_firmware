# Generated firmware board configs

Do **not** commit board YAML here. The Lucy config pipeline writes:

```text
config/config_<board_id>.yaml
```

from the active robot package hardware mapping (e.g. `so_arm101_urdf`).
At build time the pipeline copies the matching file into
`firmwares/<crate>/config.yaml` for `build.rs` (that path is gitignored).

## Local cargo without the pipeline

```bash
export LUCY_FIRMWARE_CONFIG=/absolute/path/to/config_rp2040_so_arm.yaml
cargo build --release -p lucy_embedded_firmware_rp2040_servo2040 \
  --target thumbv6m-none-eabi
```

Or drop `config_<board_id>.yaml` into this directory after running the
config pipeline / generator.
