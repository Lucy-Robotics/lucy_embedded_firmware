use crate::resources::Resources;
use crate::usb_port::lucy_usb_config;

/// Never fails: with no adapter yet, the maintenance socket still reports why.
pub fn init() -> Resources {
    let mut resources = Resources::new(lucy_usb_config());
    resources.refresh();
    resources
}
