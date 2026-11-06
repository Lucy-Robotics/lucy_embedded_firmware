//! Local socket for calibration tooling: bus health and EEPROM homing.
//!
//! One request per line, one JSON reply per line. This firmware owns the serial port, so anything
//! that needs the bus outside the control loop has to come through here.

use std::io::{ErrorKind, Read, Write};
use std::os::unix::fs::{DirBuilderExt, MetadataExt, PermissionsExt};
use std::os::unix::net::{UnixListener, UnixStream};
use std::path::{Path, PathBuf};
use std::time::Duration;

const MAX_LINE: usize = 4096;
const MAX_CLIENTS: usize = 8;
const REPLY_TIMEOUT: Duration = Duration::from_millis(100);

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct RestoreEntry {
    pub id: u8,
    pub homing_offset: i16,
    pub min_limit: u16,
    pub max_limit: u16,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Request {
    Status,
    Select(String),
    ReadHoming,
    /// EEPROM writes name the adapter they are meant for, so that a stray client or a test
    /// can't reach whichever arm happens to be plugged in.
    SetHalfTurn {
        serial: String,
    },
    RestoreHoming {
        serial: String,
        entries: Vec<RestoreEntry>,
    },
}

pub fn parse_request(line: &str) -> Result<Request, String> {
    let line = line.trim();
    let (command, argument) = line.split_once(' ').unwrap_or((line, ""));
    let argument = argument.trim();
    match (command, argument) {
        ("status", "") => Ok(Request::Status),
        ("read_homing", "") => Ok(Request::ReadHoming),
        ("set_half_turn", serial) if !serial.is_empty() => Ok(Request::SetHalfTurn {
            serial: serial.to_string(),
        }),
        ("select", serial) if !serial.is_empty() => Ok(Request::Select(serial.to_string())),
        ("restore_homing", rest) => {
            let (serial, entries) = rest
                .rsplit_once(' ')
                .ok_or_else(|| format!("bad request: {line}"))?;
            Ok(Request::RestoreHoming {
                serial: serial.trim().to_string(),
                entries: entries
                    .split(',')
                    .map(parse_restore_entry)
                    .collect::<Result<_, _>>()?,
            })
        }
        _ => Err(format!("bad request: {line}")),
    }
}

/// `id:homing_offset:min_limit:max_limit`
fn parse_restore_entry(entry: &str) -> Result<RestoreEntry, String> {
    let bad = || format!("bad restore entry: {entry}");
    let mut fields = entry.split(':');
    let mut next = || fields.next().ok_or_else(bad);
    let parsed = RestoreEntry {
        id: next()?.parse().map_err(|_| bad())?,
        homing_offset: next()?.parse().map_err(|_| bad())?,
        min_limit: next()?.parse().map_err(|_| bad())?,
        max_limit: next()?.parse().map_err(|_| bad())?,
    };
    if fields.next().is_some() {
        return Err(bad());
    }
    Ok(parsed)
}

pub fn json_string(value: &str) -> String {
    let mut out = String::with_capacity(value.len() + 2);
    out.push('"');
    for c in value.chars() {
        match c {
            '"' => out.push_str("\\\""),
            '\\' => out.push_str("\\\\"),
            c if (c as u32) < 0x20 => out.push_str(&format!("\\u{:04x}", c as u32)),
            c => out.push(c),
        }
    }
    out.push('"');
    out
}

pub fn json_error(code: &str) -> String {
    format!("{{\"ok\":false,\"error\":{}}}", json_string(code))
}

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

    /// Accept clients and answer every complete request. Never blocks on a silent client.
    pub fn poll(&mut self, mut handle: impl FnMut(Result<Request, String>) -> String) {
        while let Ok((stream, _)) = self.listener.accept() {
            if self.clients.len() < MAX_CLIENTS && stream.set_nonblocking(true).is_ok() {
                self.clients.push(Client {
                    stream,
                    pending: Vec::new(),
                });
            }
        }
        self.clients.retain_mut(|client| serve(client, &mut handle));
    }
}

impl Drop for Server {
    fn drop(&mut self) {
        let _ = std::fs::remove_file(&self.path);
    }
}

/// Returns whether to keep this client.
fn serve(client: &mut Client, handle: &mut impl FnMut(Result<Request, String>) -> String) -> bool {
    let mut chunk = [0u8; 512];
    loop {
        match client.stream.read(&mut chunk) {
            Ok(0) => return false,
            Ok(n) => client.pending.extend_from_slice(&chunk[..n]),
            Err(e) if e.kind() == ErrorKind::WouldBlock => break,
            Err(e) if e.kind() == ErrorKind::Interrupted => continue,
            Err(_) => return false,
        }
        if client.pending.len() > MAX_LINE {
            return false;
        }
    }

    while let Some(end) = client.pending.iter().position(|&b| b == b'\n') {
        let line: Vec<u8> = client.pending.drain(..=end).collect();
        let request = std::str::from_utf8(&line)
            .map_err(|_| "request is not UTF-8".to_string())
            .and_then(parse_request);
        let mut reply = handle(request);
        reply.push('\n');
        if !send(&client.stream, reply.as_bytes()) {
            return false;
        }
    }
    true
}

fn send(mut stream: &UnixStream, bytes: &[u8]) -> bool {
    stream.set_nonblocking(false).is_ok()
        && stream.set_write_timeout(Some(REPLY_TIMEOUT)).is_ok()
        && stream.write_all(bytes).is_ok()
        && stream.set_nonblocking(true).is_ok()
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::io::{BufRead, BufReader};

    #[test]
    fn parses_plain_commands() {
        assert_eq!(parse_request("status\n"), Ok(Request::Status));
        assert_eq!(parse_request("read_homing"), Ok(Request::ReadHoming));
        assert_eq!(
            parse_request("set_half_turn 5A46"),
            Ok(Request::SetHalfTurn {
                serial: "5A46".to_string()
            })
        );
        assert_eq!(
            parse_request("select 5A46"),
            Ok(Request::Select("5A46".to_string()))
        );
    }

    #[test]
    fn parses_restore_entries_with_negative_offsets() {
        assert_eq!(
            parse_request("restore_homing 5A46 1:-250:700:3300,2:0:0:4095"),
            Ok(Request::RestoreHoming {
                serial: "5A46".to_string(),
                entries: vec![
                    RestoreEntry {
                        id: 1,
                        homing_offset: -250,
                        min_limit: 700,
                        max_limit: 3300
                    },
                    RestoreEntry {
                        id: 2,
                        homing_offset: 0,
                        min_limit: 0,
                        max_limit: 4095
                    },
                ],
            })
        );
    }

    #[test]
    fn rejects_malformed_requests() {
        for line in [
            "",
            "unknown",
            "status now",
            "select",
            "set_half_turn",
            "restore_homing",
            "restore_homing 1:2:3:4",
            "restore_homing 5A46 1:2:3",
            "restore_homing 5A46 1:2:3:4:5",
            "restore_homing 5A46 a:2:3:4",
            "restore_homing 5A46 1:2:3:70000",
        ] {
            assert!(parse_request(line).is_err(), "{line:?} should be rejected");
        }
    }

    #[test]
    fn json_strings_cannot_break_out() {
        assert_eq!(json_string(r#"a"b\c"#), r#""a\"b\\c""#);
        assert_eq!(json_string("a\nb"), "\"a\\u000ab\"");
    }

    fn temp_socket(name: &str) -> PathBuf {
        let dir = std::env::temp_dir().join(format!("lucy-test-{}-{name}", std::process::id()));
        dir.join("node.sock")
    }

    #[test]
    fn server_answers_over_a_private_socket_and_cleans_up() {
        let path = temp_socket("roundtrip");
        let mut server = Server::bind(&path).unwrap();
        assert_eq!(
            std::fs::metadata(&path).unwrap().permissions().mode() & 0o777,
            0o600
        );

        let mut client = UnixStream::connect(&path).unwrap();
        client.write_all(b"status\nnonsense\n").unwrap();
        client
            .set_read_timeout(Some(Duration::from_secs(2)))
            .unwrap();

        // One poll accepts the client and answers both lines.
        server.poll(|request| match request {
            Ok(Request::Status) => "{\"ok\":true}".to_string(),
            _ => json_error("bad_request"),
        });

        let mut replies = BufReader::new(client.try_clone().unwrap());
        let mut line = String::new();
        replies.read_line(&mut line).unwrap();
        assert_eq!(line, "{\"ok\":true}\n");
        line.clear();
        replies.read_line(&mut line).unwrap();
        assert_eq!(line, "{\"ok\":false,\"error\":\"bad_request\"}\n");

        drop(server);
        assert!(!path.exists());
        std::fs::remove_dir_all(path.parent().unwrap()).unwrap();
    }

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

    #[test]
    fn partial_line_waits_for_the_rest() {
        let path = temp_socket("partial");
        let mut server = Server::bind(&path).unwrap();
        let mut client = UnixStream::connect(&path).unwrap();
        let mut calls = 0;

        client.write_all(b"sta").unwrap();
        server.poll(|_| {
            calls += 1;
            String::new()
        });
        assert_eq!(calls, 0);

        client.write_all(b"tus\n").unwrap();
        server.poll(|request| {
            calls += 1;
            assert_eq!(request, Ok(Request::Status));
            String::new()
        });
        assert_eq!(calls, 1);

        drop(server);
        std::fs::remove_dir_all(path.parent().unwrap()).unwrap();
    }
}
