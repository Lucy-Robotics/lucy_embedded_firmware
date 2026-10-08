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

/// Activity-LED GPIO bitmask from picotool `wValue` (bits 8 + 9..).
///
/// Pin is masked to 0..31 so a hostile USB host cannot trigger shift UB.
pub(crate) fn bootsel_activity_led_mask(w_value: u16) -> u32 {
    if w_value & 0x100 == 0 {
        return 0;
    }
    let pin = (w_value >> 9) & 0x1f;
    1u32 << pin
}

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
                let gpio_mask = bootsel_activity_led_mask(req.value);
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

#[cfg(test)]
mod tests {
    use super::bootsel_activity_led_mask;

    #[test]
    fn bootsel_mask_unset_when_bit8_clear() {
        assert_eq!(bootsel_activity_led_mask(0), 0);
        assert_eq!(bootsel_activity_led_mask(0x7f), 0);
    }

    #[test]
    fn bootsel_mask_clamps_pin_to_five_bits() {
        // bit8 set, pin field = 25 → GPIO25
        assert_eq!(bootsel_activity_led_mask(0x100 | (25 << 9)), 1u32 << 25);
        // Hostile: huge shift field still masks to pin 31
        assert_eq!(bootsel_activity_led_mask(0x100 | (0xffff << 9)), 1u32 << 31);
    }
}
