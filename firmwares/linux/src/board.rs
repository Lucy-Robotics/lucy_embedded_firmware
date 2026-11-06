use lucy_embedded_firmware_host::usb_port::{UsbPort, UsbPortConfig};
use crate::resources::Resources;

pub fn init() -> Resources {
    let cfg = UsbPortConfig::new(0x1a86, 0x55d3, 1_000_000);
    Resources {
        usb0: UsbPort::new(cfg),
    }
}
