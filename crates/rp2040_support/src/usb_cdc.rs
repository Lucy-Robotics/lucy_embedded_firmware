//! Shared USB CDC + picotool device builder for Lucy RP2040 firmwares.

use crate::picotool_reset::PicoToolReset;
use usb_device::{class_prelude::UsbBusAllocator, prelude::*};
use usbd_serial::SerialPort;

/// Prefer generated serial id; fall back to `"TEST"` when empty.
pub fn resolve_usb_serial(generated: &'static str) -> &'static str {
    if generated.is_empty() {
        "TEST"
    } else {
        generated
    }
}

/// Build composite USB device: CDC serial + picotool reset (VID 0x2e8a).
pub fn build_usb_device<'a, B: usb_device::bus::UsbBus>(
    usb_bus: &'a UsbBusAllocator<B>,
    product: &'static str,
    serial_number: &'static str,
) -> (
    SerialPort<'a, B>,
    PicoToolReset<'a, B>,
    UsbDevice<'a, B>,
) {
    let serial = SerialPort::new(usb_bus);
    let picotool = PicoToolReset::new(usb_bus);
    let usb_dev = UsbDeviceBuilder::new(usb_bus, UsbVidPid(0x2e8a, 0x000a))
        .strings(&[StringDescriptors::default()
            .manufacturer("Sentience")
            .product(product)
            .serial_number(serial_number)])
        .unwrap()
        .composite_with_iads()
        .max_packet_size_0(64)
        .unwrap()
        .build();
    (serial, picotool, usb_dev)
}
