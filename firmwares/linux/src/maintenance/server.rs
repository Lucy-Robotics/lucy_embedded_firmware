//! Unix-socket transport for the maintenance protocol: one request per line, one reply per line.

mod poll;

use std::io::ErrorKind;
use std::os::unix::fs::{DirBuilderExt, MetadataExt, PermissionsExt};
use std::os::unix::net::{UnixListener, UnixStream};
use std::path::{Path, PathBuf};

/// `$XDG_RUNTIME_DIR/lucy/<node>.sock`, or a per-user directory under the temp dir without it.
pub fn socket_path(node: &str) -> PathBuf {
    let dir = match std::env::var_os("XDG_RUNTIME_DIR").filter(|d| !d.is_empty()) {
        Some(runtime) => PathBuf::from(runtime).join("lucy"),
        None => std::env::temp_dir().join(format!("lucy-{}", unsafe { libc::geteuid() })),
    };
    dir.join(format!("{node}.sock"))
}

struct Client {
    stream: UnixStream,
    pending: Vec<u8>,
}

pub struct Server {
    listener: UnixListener,
    clients: Vec<Client>,
    path: PathBuf,
}

impl Server {
    pub fn bind(path: &Path) -> std::io::Result<Self> {
        if let Some(dir) = path.parent() {
            std::fs::DirBuilder::new()
                .recursive(true)
                .mode(0o700)
                .create(dir)?;
            // The fallback directory lives in a world-writable place, so check who made it.
            let owner = std::fs::metadata(dir)?.uid();
            if owner != unsafe { libc::geteuid() } {
                return Err(std::io::Error::new(
                    ErrorKind::PermissionDenied,
                    format!("{} is owned by uid {owner}", dir.display()),
                ));
            }
        }
        // A previous run that was killed leaves its socket behind.
        match std::fs::remove_file(path) {
            Err(e) if e.kind() != ErrorKind::NotFound => return Err(e),
            _ => {}
        }
        let listener = UnixListener::bind(path)?;
        listener.set_nonblocking(true)?;
        std::fs::set_permissions(path, std::fs::Permissions::from_mode(0o600))?;
        Ok(Self {
            listener,
            clients: Vec::new(),
            path: path.to_path_buf(),
        })
    }
}

impl Drop for Server {
    fn drop(&mut self) {
        let _ = std::fs::remove_file(&self.path);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::test_support::temp_socket;

    #[test]
    fn server_replaces_a_stale_socket() {
        let path = temp_socket("stale");
        let first = Server::bind(&path).unwrap();
        std::mem::forget(first); // As if killed: the socket file stays behind.
        let second = Server::bind(&path).unwrap();
        assert!(UnixStream::connect(&path).is_ok());
        drop(second);
        std::fs::remove_dir_all(path.parent().unwrap()).unwrap();
    }
}
