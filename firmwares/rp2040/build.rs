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

    println!("cargo:rerun-if-changed=memory.x");
    println!("cargo:rerun-if-changed=build.rs");
    println!("cargo:rerun-if-env-changed=DEVICE_SERIAL_NUMBER");

    let config_path = PathBuf::from(env::var("CARGO_MANIFEST_DIR").unwrap()).join("config.yaml");
    if config_path.exists() {
        println!("cargo:rerun-if-changed=config.yaml");
    }

    // USB serial: env override, else config.yaml `serial_id`. Never hardcode a
    // board string — that bricks multi-Pico desks. Missing → omit (main.rs).
    if let Some(serial) = resolve_device_serial(&config_path) {
        println!("cargo:rustc-env=DEVICE_SERIAL_NUMBER={serial}");
    }

    if config_path.exists() {
        if let Ok(contents) = std::fs::read_to_string(&config_path) {
            if contents.contains("actuators:") && contents.contains("virtual_pin:") {
                // TODO: re-enable once crates/builder compiles and is a build-dependency.
                // build_config(config_path.to_string_lossy().into_owned());
            }
        }
    }
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
