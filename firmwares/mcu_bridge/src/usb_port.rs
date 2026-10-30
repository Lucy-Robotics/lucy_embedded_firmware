use std::fs;
use std::io::{self, Read, Write};
use std::path::Path;
use std::time::Duration;

use lucy_embedded_firmware_feetech::serial_channel::SerialChannel;
use serialport::{SerialPortType, UsbPortInfo};

/// Raspberry Pi / Pico USB vendor (bootrom + our firmware).
pub const PICO_VID: u16 = 0x2e8a;
/// Shared with RP2 Boot — distinguish via product string / CDC presence.
pub const PICO_PID: u16 = 0x0003;

#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct UsbPortConfig {
    pub target_vid: u16,
    pub target_pid: u16,
    pub baud_rate: u32,
    pub serial: String,
}

#[derive(Debug, Clone, PartialEq, Eq)]
struct UsbDeviceHint {
    serial: String,
    product: String,
}

#[derive(Debug, Clone, Default, PartialEq, Eq)]
struct UsbSnapshot {
    /// 2e8a:0003 devices whose product looks like the RP2 bootrom mass-storage.
    bootsel: Vec<UsbDeviceHint>,
    /// CDC serial ports matching our VID/PID (running firmware).
    cdc_serials: Vec<String>,
    /// Other 2e8a USB devices (for the message).
    other_pico: Vec<UsbDeviceHint>,
}

/// Rich open-failure text: BOOTSEL vs wrong serial vs missing vs permissions.
pub fn diagnose_open_failure(cfg: &UsbPortConfig, device: &str, err: &io::Error) -> String {
    let snap = UsbSnapshot::capture(cfg.target_vid, cfg.target_pid);
    diagnose_failure(err.kind(), device, &cfg.serial, &snap)
}

fn diagnose_failure(kind: io::ErrorKind, device: &str, expected_serial: &str, snap: &UsbSnapshot) -> String {
    match kind {
        io::ErrorKind::PermissionDenied => {
            format!("Lucy is not allowed to use {device}. Grant USB access (pixi run setup-udev). Then relaunch Lucy.")
        }
        io::ErrorKind::ResourceBusy | io::ErrorKind::AddrInUse => {
            format!("{device} is in use by another program. Close that program. Then relaunch Lucy.")
        }
        io::ErrorKind::NotFound => diagnose_not_found(device, expected_serial, snap),
        _ => {
            let detail = diagnose_not_found(device, expected_serial, snap);
            if detail.contains("BOOTSEL") || detail.contains("expected USB serial") {
                detail
            } else {
                format!("Lucy could not open {device}. {detail}")
            }
        }
    }
}

fn diagnose_not_found(device: &str, expected_serial: &str, snap: &UsbSnapshot) -> String {
    if !snap.bootsel.is_empty() {
        let ids: Vec<String> = snap
            .bootsel
            .iter()
            .map(|d| {
                if d.serial.is_empty() {
                    d.product.clone()
                } else {
                    format!("{} ({})", d.product, d.serial)
                }
            })
            .collect();
        return format!(
            "{device}: board is in BOOTSEL / RP2 Boot [{ids}], not running firmware — \
             there is no CDC serial yet. Flash the UF2 (Lucy Activate → flash, or copy \
             the .uf2 onto the RPI-RP2 drive), wait for reboot as Servo2040 Serial \
             with serial_id={expected}, then relaunch Lucy.",
            ids = ids.join(", "),
            expected = if expected_serial.is_empty() {
                "(configured serial_id)"
            } else {
                expected_serial
            },
        );
    }

    if !expected_serial.is_empty()
        && !snap.cdc_serials.is_empty()
        && !snap.cdc_serials.iter().any(|s| s == expected_serial)
    {
        return format!(
            "{device}: board serial_id={expected_serial} is not on USB. \
             Only other Pico CDC serial(s) are plugged in: {}. \
             Plug in this board (or remove it from hardware YAML / use a single-board preset), \
             then relaunch Lucy.",
            snap.cdc_serials.join(", ")
        );
    }

    if snap.cdc_serials.is_empty() && snap.other_pico.is_empty() {
        return format!(
            "{device}: no Pico/Servo2040 USB device found (want serial {expected}). \
             Plug the board in with application firmware running (not BOOTSEL), then relaunch Lucy.",
            expected = if expected_serial.is_empty() {
                "(configured serial_id)"
            } else {
                expected_serial
            },
        );
    }

    format!(
        "{device}: no CDC serial matching serial_id={expected}. \
         Disconnect and reconnect the board, then relaunch Lucy.",
        expected = if expected_serial.is_empty() {
            "(configured)"
        } else {
            expected_serial
        },
    )
}

impl UsbSnapshot {
    fn capture(target_vid: u16, target_pid: u16) -> Self {
        let mut snap = Self::default();
        snap.cdc_serials = list_cdc_serials(target_vid, target_pid);
        for hint in list_sysfs_pico_devices(target_vid) {
            let product_l = hint.product.to_ascii_lowercase();
            let is_bootsel = product_l.contains("rp2 boot") || product_l.contains("bootsel");
            if is_bootsel {
                snap.bootsel.push(hint);
            } else if hint.serial.is_empty()
                || !snap.cdc_serials.iter().any(|s| s == &hint.serial)
            {
                // Same VID/PID as bootrom when running our image; if it is not
                // exposing CDC (or serial differs), keep it for the message.
                snap.other_pico.push(hint);
            }
        }
        let _ = target_pid; // reserved: filter when we split app PID from bootrom
        snap
    }
}

fn list_cdc_serials(target_vid: u16, target_pid: u16) -> Vec<String> {
    let Ok(ports) = serialport::available_ports() else {
        return Vec::new();
    };
    let mut out = Vec::new();
    for port in ports {
        let SerialPortType::UsbPort(UsbPortInfo {
            vid,
            pid,
            serial_number,
            ..
        }) = port.port_type
        else {
            continue;
        };
        if vid != target_vid || pid != target_pid {
            continue;
        }
        if let Some(serial) = serial_number {
            if !serial.is_empty() {
                out.push(serial);
            }
        }
    }
    out.sort();
    out.dedup();
    out
}

/// Walk `/sys/bus/usb/devices` for Raspberry Pi Pico VID devices.
fn list_sysfs_pico_devices(target_vid: u16) -> Vec<UsbDeviceHint> {
    let root = Path::new("/sys/bus/usb/devices");
    let Ok(entries) = fs::read_dir(root) else {
        return Vec::new();
    };
    let want = format!("{target_vid:04x}");
    let mut out = Vec::new();
    for entry in entries.flatten() {
        let path = entry.path();
        let vid_path = path.join("idVendor");
        if !vid_path.is_file() {
            continue;
        }
        let Ok(vid) = fs::read_to_string(&vid_path) else {
            continue;
        };
        if vid.trim().eq_ignore_ascii_case(&want) {
            let product = fs::read_to_string(path.join("product"))
                .unwrap_or_default()
                .trim()
                .to_string();
            let serial = fs::read_to_string(path.join("serial"))
                .unwrap_or_default()
                .trim()
                .to_string();
            out.push(UsbDeviceHint { serial, product });
        }
    }
    out
}

impl UsbPortConfig {
    pub fn new(target_vid: u16, target_pid: u16, baud_rate: u32, serial: impl Into<String>) -> Self {
        Self {
            target_vid,
            target_pid,
            baud_rate,
            serial: serial.into(),
        }
    }

    pub fn open(&self) -> Result<UsbPort, io::Error> {
        let ports = serialport::available_ports()?;
        let port_info = self.select(ports)?;
        let port = serialport::new(&port_info.port_name, self.baud_rate)
            .timeout(Duration::from_millis(50))
            .open()
            .map_err(|err| annotate_open(&port_info.port_name, err))?;
        Ok(UsbPort { port })
    }

    fn select(
        &self,
        ports: Vec<serialport::SerialPortInfo>,
    ) -> Result<serialport::SerialPortInfo, io::Error> {
        ports
            .into_iter()
            .find(|port| match &port.port_type {
                SerialPortType::UsbPort(UsbPortInfo {
                    vid,
                    pid,
                    serial_number,
                    ..
                }) => {
                    *vid == self.target_vid
                        && *pid == self.target_pid
                        && serial_number.as_deref() == Some(self.serial.as_str())
                }
                _ => false,
            })
            .ok_or_else(|| io::Error::new(io::ErrorKind::NotFound, "no matching USB CDC serial"))
    }
}

fn annotate_open(port_name: &str, err: serialport::Error) -> io::Error {
    let io_err = io::Error::from(err);
    io::Error::new(io_err.kind(), format!("{port_name}: {io_err}"))
}

pub struct UsbPort {
    pub port: Box<dyn serialport::SerialPort>,
}

impl SerialChannel for UsbPort {
    type Error = std::io::Error;

    fn write(&mut self, bytes: &[u8]) -> Result<(), Self::Error> {
        self.port.write_all(bytes)?;
        Ok(())
    }

    fn read(&mut self, buffer: &mut [u8]) -> Result<usize, Self::Error> {
        self.port.read_exact(buffer)?;
        Ok(buffer.len())
    }

    fn clear(&mut self) -> Result<(), Self::Error> {
        self.port.clear(serialport::ClearBuffer::All)?;
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::{diagnose_failure, UsbDeviceHint, UsbPortConfig, UsbSnapshot};
    use serialport::{SerialPortInfo, SerialPortType, UsbPortInfo};

    fn usb(name: &str, vid: u16, pid: u16, serial: Option<&str>) -> SerialPortInfo {
        SerialPortInfo {
            port_name: name.into(),
            port_type: SerialPortType::UsbPort(UsbPortInfo {
                vid,
                pid,
                serial_number: serial.map(str::to_string),
                manufacturer: None,
                product: None,
            }),
        }
    }

    #[test]
    fn config_stores_serial_filter() {
        let cfg = UsbPortConfig::new(0x2e8a, 0x0003, 1_000_000, "E6617C93E3858429");
        assert_eq!(cfg.serial, "E6617C93E3858429");
        assert_eq!(cfg.target_vid, 0x2e8a);
        assert_eq!(cfg.target_pid, 0x0003);
    }

    #[test]
    fn missing_board_is_not_found() {
        let cfg = UsbPortConfig::new(0x2e8a, 0x0003, 1_000_000, "E6617C93E3858429");
        let err = cfg
            .select(vec![
                usb("/dev/ttyACM0", 0x2e8a, 0x0003, Some("OTHER")),
                usb("/dev/ttyUSB0", 0x1a86, 0x55d3, None),
            ])
            .expect_err("no board with that serial");
        assert_eq!(err.kind(), std::io::ErrorKind::NotFound);
    }

    #[test]
    fn bootsel_message_is_explicit() {
        let snap = UsbSnapshot {
            bootsel: vec![UsbDeviceHint {
                serial: "E0C9125B0D9B".into(),
                product: "RP2 Boot".into(),
            }],
            ..UsbSnapshot::default()
        };
        let msg = diagnose_failure(
            std::io::ErrorKind::NotFound,
            "left_arm (E6617C93E3858429)",
            "E6617C93E3858429",
            &snap,
        );
        assert!(msg.contains("BOOTSEL"), "{msg}");
        assert!(msg.contains("RP2 Boot"), "{msg}");
        assert!(msg.contains("Flash the UF2"), "{msg}");
        assert!(msg.contains("E6617C93E3858429"), "{msg}");
    }

    #[test]
    fn wrong_serial_lists_seen_cdc() {
        let snap = UsbSnapshot {
            cdc_serials: vec!["OTHERBOARD".into()],
            ..UsbSnapshot::default()
        };
        let msg = diagnose_failure(
            std::io::ErrorKind::NotFound,
            "left_arm (E6617C93E3858429)",
            "E6617C93E3858429",
            &snap,
        );
        assert!(msg.contains("expected USB serial E6617C93E3858429"), "{msg}");
        assert!(msg.contains("OTHERBOARD"), "{msg}");
    }

    #[test]
    fn selects_matching_serial() {
        let cfg = UsbPortConfig::new(0x2e8a, 0x0003, 1_000_000, "E6617C93E3858429");
        let found = cfg
            .select(vec![
                usb("/dev/ttyACM0", 0x2e8a, 0x0003, Some("OTHER")),
                usb("/dev/ttyACM1", 0x2e8a, 0x0003, Some("E6617C93E3858429")),
            ])
            .expect("matching board");
        assert_eq!(found.port_name, "/dev/ttyACM1");
    }
}
