//! The read-a-line, answer-a-line loop: accepting clients and serving each complete request.

use std::io::{ErrorKind, Read, Write};
use std::os::unix::net::UnixStream;
use std::time::Duration;

use super::super::protocol::{Request, parse_request};
use super::{Client, Server};

const MAX_LINE: usize = 4096;
const MAX_CLIENTS: usize = 8;
const REPLY_TIMEOUT: Duration = Duration::from_millis(100);

impl Server {
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
    use crate::maintenance::json_error;
    use crate::test_support::temp_socket;
    use std::io::{BufRead, BufReader};
    use std::os::unix::fs::PermissionsExt;

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

    #[test]
    fn a_line_over_the_limit_disconnects_the_client() {
        let path = temp_socket("overlong");
        let mut server = Server::bind(&path).unwrap();
        let mut client = UnixStream::connect(&path).unwrap();

        client.write_all(&vec![b'a'; MAX_LINE + 1]).unwrap();
        server.poll(|_| String::new());
        assert_eq!(server.clients.len(), 0);

        drop(server);
        std::fs::remove_dir_all(path.parent().unwrap()).unwrap();
    }
}
