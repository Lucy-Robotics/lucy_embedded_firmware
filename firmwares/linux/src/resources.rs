use lucy_embedded_firmware_host::usb_port::UsbPort;

pub struct Resources {
    pub usb0: UsbPort,
}

impl Resources {
    pub fn connected(&self) -> bool {
        self.usb0.connected()
    }
}
