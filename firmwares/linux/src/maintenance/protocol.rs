//! Wire format for maintenance-socket requests and replies: parsing lines in, JSON strings out.

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

#[cfg(test)]
mod tests {
    use super::*;

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
}
