use std::fs;
use std::path::Path;

use serialport::{SerialPortInfo, SerialPortType, UsbPortInfo};

const PICO_VID: u16 = 0x2e8a;
const PICO_PID: u16 = 0x0003;

#[derive(Debug, Clone, PartialEq, Eq)]
struct UsbDeviceHint {
    serial: String,
    product: String,
}

#[derive(Debug, Clone, Default, PartialEq, Eq)]
struct UsbSnapshot {
    bootsel: Vec<UsbDeviceHint>,
    cdc_serials: Vec<String>,
    other_pico: Vec<UsbDeviceHint>,
}

/// Why no Pico/Servo2040 answers: BOOTSEL, another board's serial, or nothing plugged in.
pub fn not_found_reason(expected_serial: &str) -> String {
    let ports = serialport::available_ports().unwrap_or_default();
    let snap = UsbSnapshot::capture(ports, Path::new("/sys/bus/usb/devices"));
    diagnose(expected_serial, &snap)
}

fn diagnose(expected_serial: &str, snap: &UsbSnapshot) -> String {
    let expected = if expected_serial.is_empty() {
        "(configured serial_id)"
    } else {
        expected_serial
    };
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
            "not found: board is in BOOTSEL / RP2 Boot [{}], not running firmware. \
             Flash the UF2 (Lucy Activate, or copy the .uf2 onto the RPI-RP2 drive) \
             so it reboots as serial_id={expected}",
            ids.join(", ")
        );
    }
    if !expected_serial.is_empty()
        && !snap.cdc_serials.is_empty()
        && !snap.cdc_serials.iter().any(|s| s == expected_serial)
    {
        return format!(
            "not found: serial_id={expected_serial} is not on USB, only {}",
            snap.cdc_serials.join(", ")
        );
    }
    if snap.cdc_serials.is_empty() && snap.other_pico.is_empty() {
        return format!("not found: no Pico/Servo2040 on USB (want serial_id={expected})");
    }
    format!("not found: no CDC serial matching serial_id={expected}, replug the board")
}

impl UsbSnapshot {
    fn capture(ports: Vec<SerialPortInfo>, sysfs: &Path) -> Self {
        let mut snap = Self {
            cdc_serials: cdc_serials(ports),
            ..Self::default()
        };
        for hint in sysfs_pico_devices(sysfs) {
            let product = hint.product.to_ascii_lowercase();
            if product.contains("rp2 boot") || product.contains("bootsel") {
                snap.bootsel.push(hint);
            } else if hint.serial.is_empty() || !snap.cdc_serials.contains(&hint.serial) {
                snap.other_pico.push(hint);
            }
        }
        snap
    }
}

fn cdc_serials(ports: Vec<SerialPortInfo>) -> Vec<String> {
    let mut out: Vec<String> = ports
        .into_iter()
        .filter_map(|port| match port.port_type {
            SerialPortType::UsbPort(UsbPortInfo {
                vid,
                pid,
                serial_number: Some(serial),
                ..
            }) if vid == PICO_VID && pid == PICO_PID && !serial.is_empty() => Some(serial),
            _ => None,
        })
        .collect();
    out.sort();
    out.dedup();
    out
}

fn sysfs_pico_devices(sysfs: &Path) -> Vec<UsbDeviceHint> {
    let Ok(entries) = fs::read_dir(sysfs) else {
        return Vec::new();
    };
    let want = format!("{PICO_VID:04x}");
    let read = |path: &Path, name: &str| {
        fs::read_to_string(path.join(name))
            .unwrap_or_default()
            .trim()
            .to_string()
    };
    entries
        .flatten()
        .map(|entry| entry.path())
        .filter(|path| read(path, "idVendor").eq_ignore_ascii_case(&want))
        .map(|path| UsbDeviceHint {
            serial: read(&path, "serial"),
            product: read(&path, "product"),
        })
        .collect()
}

#[cfg(test)]
#[path = "pico_diagnosis_tests.rs"]
mod tests;
