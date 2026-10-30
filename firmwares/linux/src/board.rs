use crate::usb_port::UsbPortConfig;
use crate::resources::Resources;

pub fn init(vid: u16, pid: u16) -> std::io::Result<Resources> {
    let cfg = UsbPortConfig::new(vid, pid, 1_000_000);
    Ok(Resources { usb0: cfg.open()? })
}
