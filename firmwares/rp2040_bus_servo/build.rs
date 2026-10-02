//! Resolve board YAML for codegen and write `$OUT_DIR/config.rs` + `memory.x`.
use std::env;
use std::fs::File;
use std::io::Write;
use std::path::PathBuf;

use builder::{build_config, write_empty_config};

fn main() {
    let out = &PathBuf::from(env::var_os("OUT_DIR").unwrap());
    File::create(out.join("memory.x"))
        .unwrap()
        .write_all(include_bytes!("memory.x"))
        .unwrap();
    println!("cargo:rustc-link-search={}", out.display());
    println!("cargo:rerun-if-changed=memory.x");
    println!("cargo:rerun-if-changed=build.rs");
    println!("cargo:rerun-if-env-changed=LUCY_FIRMWARE_CONFIG");
    println!("cargo:rerun-if-changed=config.yaml");

    let manifest = PathBuf::from(env::var("CARGO_MANIFEST_DIR").unwrap());
    println!(
        "cargo:rerun-if-changed={}",
        manifest.join("../../config").display()
    );

    if let Some(path) = resolve_config_path(&manifest) {
        if let Ok(contents) = std::fs::read_to_string(&path) {
            if contents.contains("actuators:") {
                println!("cargo:rerun-if-changed={}", path.display());
                build_config(path.to_string_lossy().into_owned());
                return;
            }
        }
    }
    write_empty_config();
}

fn resolve_config_path(manifest_dir: &PathBuf) -> Option<PathBuf> {
    if let Ok(p) = env::var("LUCY_FIRMWARE_CONFIG") {
        let path = PathBuf::from(p);
        if path.is_file() {
            return Some(path);
        }
    }
    let crate_cfg = manifest_dir.join("config.yaml");
    if crate_cfg.is_file() {
        return Some(crate_cfg);
    }
    None
}
