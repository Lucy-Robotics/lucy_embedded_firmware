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
    println!("cargo:rerun-if-changed=config.yaml");

    let config_path = PathBuf::from(env::var("CARGO_MANIFEST_DIR").unwrap()).join("config.yaml");
    if config_path.exists() {
        if let Ok(contents) = std::fs::read_to_string(&config_path) {
            if contents.contains("actuators:") {
                build_config(config_path.to_string_lossy().into_owned());
                return;
            }
        }
    }
    write_empty_config();
}
