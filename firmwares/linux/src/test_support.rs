//! Test fixtures shared by the maintenance test suite.

pub(crate) fn temp_socket(name: &str) -> std::path::PathBuf {
    let dir = std::env::temp_dir().join(format!("lucy-test-{}-{name}", std::process::id()));
    dir.join("node.sock")
}
