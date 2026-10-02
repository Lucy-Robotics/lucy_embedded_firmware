//! Picotool force-reset USB vendor interface (VID 0x2e8a compatible).
//!
//! Mirrors ``usbd-picotool-reset`` without pulling an older ``rp2040-hal``.
//! ``picotool load -f`` / ``picotool reboot -f`` talk to this interface.

use core::marker::PhantomData;

use rp2040_hal::rom_data::reset_to_usb_boot;
use usb_device::class_prelude::{InterfaceNumber, StringIndex, UsbBus, UsbBusAllocator};
use usb_device::control::{Recipient, RequestType};
use usb_device::LangID;

const CLASS_VENDOR_SPECIFIC: u8 = 0xff;
const RESET_INTERFACE_SUBCLASS: u8 = 0x00;
const RESET_INTERFACE_PROTOCOL: u8 = 0x01;
const RESET_REQUEST_BOOTSEL: u8 = 0x01;

/// UsbClass that reboots into BOOTSEL when picotool requests it.
pub struct PicoToolReset<'a, B: UsbBus> {
    intf: InterfaceNumber,
    str_idx: StringIndex,
    _bus: PhantomData<&'a B>,
}

impl<'a, B: UsbBus> PicoToolReset<'a, B> {
    pub fn new(alloc: &'a UsbBusAllocator<B>) -> Self {
        Self {
            intf: alloc.interface(),
            str_idx: alloc.string(),
            _bus: PhantomData,
        }
    }
}

impl<B: UsbBus> usb_device::class::UsbClass<B> for PicoToolReset<'_, B> {
    fn get_configuration_descriptors(
        &self,
        writer: &mut usb_device::descriptor::DescriptorWriter,
    ) -> usb_device::Result<()> {
        writer.interface_alt(
            self.intf,
            0,
            CLASS_VENDOR_SPECIFIC,
            RESET_INTERFACE_SUBCLASS,
            RESET_INTERFACE_PROTOCOL,
            Some(self.str_idx),
        )
    }

    fn get_string(&self, index: StringIndex, _lang_id: LangID) -> Option<&str> {
        (index == self.str_idx).then_some("Reset")
    }

    fn control_out(&mut self, xfer: usb_device::class_prelude::ControlOut<B>) {
        let req = xfer.request();
        if !(req.request_type == RequestType::Class
            && req.recipient == Recipient::Interface
            && req.index == u8::from(self.intf) as u16)
        {
            return;
        }

        match req.request {
            RESET_REQUEST_BOOTSEL => {
                let mut gpio_mask = 0u32;
                if req.value & 0x100 != 0 {
                    gpio_mask = 1 << (req.value >> 9);
                }
                reset_to_usb_boot(gpio_mask, u32::from(req.value & 0x7f));
                unreachable!()
            }
            _ => {
                let _ = xfer.reject();
            }
        }
    }

    fn control_in(&mut self, xfer: usb_device::class_prelude::ControlIn<B>) {
        let req = xfer.request();
        if !(req.request_type == RequestType::Class
            && req.recipient == Recipient::Interface
            && req.index == u8::from(self.intf) as u16)
        {
            return;
        }
        let _ = xfer.reject();
    }
}
