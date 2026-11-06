use crate::resources::Resources;
use crate::usb_port::UsbPortConfig;

/// Never fails: with no adapter yet, the maintenance socket still reports why.
pub fn init() -> Resources {
    let cfg = UsbPortConfig::new(0x1a86, 0x55d3, 1_000_000);
    let mut resources = Resources::new(cfg);
    resources.refresh();
    resources
}
