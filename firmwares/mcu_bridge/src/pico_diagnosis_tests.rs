use super::{UsbDeviceHint, UsbSnapshot, diagnose, not_found_reason};
use serialport::{SerialPortInfo, SerialPortType, UsbPortInfo};
use std::path::{Path, PathBuf};

fn port(vid: u16, pid: u16, serial: Option<&str>) -> SerialPortInfo {
    SerialPortInfo {
        port_name: "/dev/ttyACM0".into(),
        port_type: SerialPortType::UsbPort(UsbPortInfo {
            vid,
            pid,
            serial_number: serial.map(str::to_string),
            manufacturer: None,
            product: None,
        }),
    }
}

struct FakeSysfs(PathBuf);

impl FakeSysfs {
    fn new(name: &str) -> Self {
        let root = std::env::temp_dir().join(format!("lucy_sysfs_{name}_{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&root);
        std::fs::create_dir_all(&root).unwrap();
        Self(root)
    }

    fn device(&self, name: &str, vendor: &str, product: &str, serial: &str) -> &Self {
        let dir = self.0.join(name);
        std::fs::create_dir(&dir).unwrap();
        std::fs::write(dir.join("idVendor"), format!("{vendor}\n")).unwrap();
        std::fs::write(dir.join("product"), format!("{product}\n")).unwrap();
        std::fs::write(dir.join("serial"), format!("{serial}\n")).unwrap();
        self
    }

    fn path(&self) -> &Path {
        &self.0
    }
}

impl Drop for FakeSysfs {
    fn drop(&mut self) {
        let _ = std::fs::remove_dir_all(&self.0);
    }
}

#[test]
fn bootsel_device_on_the_bus_is_reported_as_bootsel() {
    let sysfs = FakeSysfs::new("bootsel");
    sysfs.device("1-1", "2e8a", "RP2 Boot", "");
    let snap = UsbSnapshot::capture(Vec::new(), sysfs.path());
    let msg = diagnose("", &snap);
    assert!(msg.contains("BOOTSEL / RP2 Boot [RP2 Boot]"), "{msg}");
    assert!(msg.contains("serial_id=(configured serial_id)"), "{msg}");
}

#[test]
fn running_pico_with_its_cdc_port_is_not_listed_as_silent() {
    let sysfs = FakeSysfs::new("running");
    sysfs.device("1-1", "2e8a", "Servo2040", "OTHER").device(
        "1-2",
        "1a86",
        "USB Single Serial",
        "X",
    );
    let snap = UsbSnapshot::capture(vec![port(0x2e8a, 0x0003, Some("OTHER"))], sysfs.path());
    assert_eq!(snap.cdc_serials, ["OTHER"]);
    assert!(snap.other_pico.is_empty() && snap.bootsel.is_empty());
    assert!(diagnose("WANTED", &snap).contains("is not on USB, only OTHER"));
}

#[test]
fn pico_without_a_cdc_port_asks_for_a_replug() {
    let sysfs = FakeSysfs::new("silent");
    sysfs.device("1-1", "2E8A", "Servo2040", "WANTED");
    let snap = UsbSnapshot::capture(Vec::new(), sysfs.path());
    assert_eq!(
        diagnose("WANTED", &snap),
        "not found: no CDC serial matching serial_id=WANTED, replug the board"
    );
}

#[test]
fn only_pico_ports_with_a_serial_count_once_each() {
    let ports = vec![
        port(0x2e8a, 0x0003, Some("B")),
        port(0x2e8a, 0x0003, Some("A")),
        port(0x2e8a, 0x0003, Some("B")),
        port(0x2e8a, 0x0003, Some("")),
        port(0x2e8a, 0x000a, Some("C")),
        port(0x2341, 0x0042, Some("MEGA")),
        port(0x2e8a, 0x0003, None),
    ];
    let snap = UsbSnapshot::capture(ports, Path::new("/nonexistent"));
    assert_eq!(snap.cdc_serials, ["A", "B"]);
}

#[test]
fn real_bus_reason_starts_with_not_found_for_the_supervisor() {
    assert!(not_found_reason("NO_SUCH_BOARD").starts_with("not found"));
}

#[test]
fn bootsel_with_a_serial_shows_both() {
    let snap = UsbSnapshot {
        bootsel: vec![UsbDeviceHint {
            serial: "E0C9".into(),
            product: "RP2 Boot".into(),
        }],
        ..UsbSnapshot::default()
    };
    assert!(diagnose("", &snap).contains("[RP2 Boot (E0C9)]"));
}

#[test]
fn bootsel_board_is_told_to_flash_the_uf2() {
    let snap = UsbSnapshot {
        bootsel: vec![UsbDeviceHint {
            serial: "E0C9125B0D9B".into(),
            product: "RP2 Boot".into(),
        }],
        ..UsbSnapshot::default()
    };
    let msg = diagnose("E6617C93E3858429", &snap);
    assert!(msg.starts_with("not found"), "{msg}");
    assert!(msg.contains("BOOTSEL"), "{msg}");
    assert!(msg.contains("RP2 Boot"), "{msg}");
    assert!(msg.contains("Flash the UF2"), "{msg}");
    assert!(msg.contains("E6617C93E3858429"), "{msg}");
}

#[test]
fn wrong_serial_lists_the_boards_seen() {
    let snap = UsbSnapshot {
        cdc_serials: vec!["OTHERBOARD".into()],
        ..UsbSnapshot::default()
    };
    let msg = diagnose("E6617C93E3858429", &snap);
    assert!(
        msg.contains("serial_id=E6617C93E3858429 is not on USB"),
        "{msg}"
    );
    assert!(msg.contains("OTHERBOARD"), "{msg}");
}

#[test]
fn nothing_plugged_in_names_the_wanted_serial() {
    let msg = diagnose("E6617C93E3858429", &UsbSnapshot::default());
    assert_eq!(
        msg,
        "not found: no Pico/Servo2040 on USB (want serial_id=E6617C93E3858429)"
    );
}
