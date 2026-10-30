use std::env;
use std::fs::File;
use std::io::Write;
use std::path::{Path, PathBuf};

fn main() {
    let out = &PathBuf::from(env::var_os("OUT_DIR").unwrap());
    File::create(out.join("memory.x"))
        .unwrap()
        .write_all(include_bytes!("memory.x"))
        .unwrap();
    println!("cargo:rustc-link-search={}", out.display());
    // Sole source of bare-metal link args. Do not also set these in
    // `.cargo/config.toml` rustflags — double -Tlink.x redefines BOOT2.
    // build.rs is required because Pixi/pipeline may invoke cargo from a cwd
    // where the workspace `.cargo/config.toml` is not on the config search path.
    println!("cargo:rustc-link-arg=--nmagic");
    println!("cargo:rustc-link-arg=-Tlink.x");

    println!("cargo:rerun-if-changed=memory.x");
    println!("cargo:rerun-if-changed=build.rs");
    println!("cargo:rerun-if-changed=config.yaml");
    println!("cargo:rerun-if-env-changed=DEVICE_SERIAL_NUMBER");
    println!("cargo:rerun-if-env-changed=LUCY_FIRMWARE_CONFIG");

    let config_path = env::var("LUCY_FIRMWARE_CONFIG")
        .map(PathBuf::from)
        .unwrap_or_else(|_| PathBuf::from(env::var("CARGO_MANIFEST_DIR").unwrap()).join("config.yaml"));
    println!("cargo:rerun-if-changed={}", config_path.display());

    // USB serial: env override, else config.yaml `serial_id`. Never hardcode a
    // board string — that bricks multi-Pico desks. Missing → omit (main.rs).
    if let Some(serial) = resolve_device_serial(&config_path) {
        println!("cargo:rustc-env=DEVICE_SERIAL_NUMBER={serial}");
    }

    if !config_path.exists() {
        panic!(
            "firmware config missing at {} (set LUCY_FIRMWARE_CONFIG or provide firmwares/rp2040/config.yaml). \
             Refusing to build an empty no-op UF2.",
            config_path.display()
        );
    }
    let yaml = std::fs::read_to_string(&config_path).unwrap_or_else(|err| {
        panic!("failed to read {}: {err}", config_path.display())
    });
    if yaml.trim().is_empty() {
        panic!(
            "{} is empty; expected a projected firmware spec with pwm:/i2c_pwm:",
            config_path.display()
        );
    }
    let has_pwm_key = yaml.lines().any(|line| line.trim_start().starts_with("pwm:"));
    let has_i2c_pwm_key = yaml.lines().any(|line| line.trim_start().starts_with("i2c_pwm:"));
    let has_actuators_key = yaml.lines().any(|line| line.trim_start().starts_with("actuators:"));
    if has_actuators_key && !has_pwm_key {
        panic!(
            "{} looks like a hardware mapping; expected a projected firmware spec with pwm:/i2c_pwm:",
            config_path.display()
        );
    }
    if !has_pwm_key && !has_i2c_pwm_key {
        panic!(
            "{} has neither pwm: nor i2c_pwm: keys (wrong file?). \
             Pass a projected firmware spec (pipeline sets LUCY_FIRMWARE_CONFIG); \
             empty pwm:/i2c_pwm: lists are allowed.",
            config_path.display()
        );
    }
    let table = builder::render_static_table(&yaml);
    std::fs::write(out.join("header_table.rs"), table).unwrap();
}

/// Prefer `DEVICE_SERIAL_NUMBER`, else non-empty `serial_id` from the board YAML.
fn resolve_device_serial(config_path: &Path) -> Option<String> {
    if let Ok(from_env) = env::var("DEVICE_SERIAL_NUMBER") {
        let trimmed = from_env.trim();
        if !trimmed.is_empty() {
            return Some(trimmed.to_string());
        }
    }
    serial_id_from_config_yaml(config_path)
}

/// Minimal parse for `serial_id: value` / `serial_id: "value"` (no YAML crate).
fn serial_id_from_config_yaml(config_path: &Path) -> Option<String> {
    let contents = std::fs::read_to_string(config_path).ok()?;
    for line in contents.lines() {
        let line = line.trim();
        if line.starts_with('#') {
            continue;
        }
        let Some(rest) = line.strip_prefix("serial_id:") else {
            continue;
        };
        let value = rest.trim().trim_matches('"').trim_matches('\'').trim();
        if !value.is_empty() {
            return Some(value.to_string());
        }
        return None;
    }
    None
}
