//! This build script copies the `memory.x` file from the crate root into
//! a directory where the linker can always find it at build time.
//! For many projects this is optional, but it is needed when using `embed-qemu`.
//!
//! By default, Cargo will re-run a build script whenever any file in the project
//! changes. By specifying `memory.x` here, we ensure the build script is only
//! re-run when `memory.x` is changed.
//!
use std::env;
use std::fs::File;
use std::io::Write;
use std::path::PathBuf;

use builder::build_config;

fn main() {
    // Put `memory.x` in our output directory and ensure it's on the linker search path.
    let out = &PathBuf::from(env::var_os("OUT_DIR").unwrap());
    File::create(out.join("memory.x"))
        .unwrap()
        .write_all(include_bytes!("memory.x"))
        .unwrap();
    println!("cargo:rustc-link-search={}", out.display());

    // By default, Cargo will re-run a build script whenever any file in the project
    // changes. By specifying `memory.x` here, we ensure the build script is only
    // re-run when `memory.x` is changed.
    println!("cargo:rerun-if-changed=memory.x");
    println!("cargo:rerun-if-changed=build.rs");
    println!("cargo:rerun-if-changed=config.yaml");

    // Generate board configuration Rust from YAML when it matches the builder schema
    // (pipeline emits board_id/actuators; ignore WIP hardware-catalog dumps).
    let config_path = PathBuf::from(env::var("CARGO_MANIFEST_DIR").unwrap()).join("config.yaml");
    if config_path.exists() {
        if let Ok(contents) = std::fs::read_to_string(&config_path) {
            if contents.contains("actuators:") && contents.contains("virtual_pin:") {
                build_config(config_path.to_string_lossy().into_owned());
            }
        }
    }
}
