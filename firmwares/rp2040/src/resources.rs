use rp2040_hal::{
    gpio,
    clocks,
    uart,
    pac,
    usb,

    Watchdog,
    Sio,
};

use crate::channel::Rp2040UartChannel;

use usb_device::{class_prelude::*, prelude::*, device};
use usbd_serial::SerialPort;

