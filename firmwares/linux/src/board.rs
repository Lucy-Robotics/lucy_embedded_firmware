use crate::resources::Resources;
use crate::usb_port::UsbPortConfig;

pub fn init() -> Resources {
    let cfg = UsbPortConfig::new(0x1a86, 0x55d3, 1_000_000);
    Resources {
        usb0: cfg.open().expect("Failed to initialize USB port"),
    }
}
