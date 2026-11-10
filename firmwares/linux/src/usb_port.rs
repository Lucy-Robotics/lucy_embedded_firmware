use std::time::Duration;

use lucy_embedded_firmware_host::usb_port::{Adapter, UsbPortConfig};

use crate::config::{USB_BAUD_RATE, USB_PID, USB_VID};

const REPLY_TIMEOUT: Duration = Duration::from_millis(5);

pub(crate) fn lucy_usb_config() -> UsbPortConfig {
    UsbPortConfig::new(USB_VID, USB_PID, USB_BAUD_RATE).with_io_timeout(REPLY_TIMEOUT)
}

/// Which adapter to drive, given what is connected and what was asked for.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AdapterChoice<'a> {
    One(&'a Adapter),
    None,
    /// Several adapters and no selection. Guessing could drive the wrong arm.
    Ambiguous,
    SelectedMissing,
}

pub fn choose_adapter<'a>(adapters: &'a [Adapter], selected: Option<&str>) -> AdapterChoice<'a> {
    if let Some(wanted) = selected {
        return match adapters
            .iter()
            .find(|a| a.serial.as_deref() == Some(wanted))
        {
            Some(adapter) => AdapterChoice::One(adapter),
            None => AdapterChoice::SelectedMissing,
        };
    }
    match adapters {
        [] => AdapterChoice::None,
        [only] => AdapterChoice::One(only),
        _ => AdapterChoice::Ambiguous,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn adapter(port: &str, serial: Option<&str>) -> Adapter {
        Adapter {
            port_name: port.to_string(),
            serial: serial.map(str::to_string),
        }
    }

    #[test]
    fn single_adapter_needs_no_selection() {
        let adapters = [adapter("/dev/ttyACM0", Some("A"))];
        assert_eq!(
            choose_adapter(&adapters, None),
            AdapterChoice::One(&adapters[0])
        );
    }

    #[test]
    fn single_adapter_without_iserial_still_opens() {
        let adapters = [adapter("/dev/ttyACM0", None)];
        assert_eq!(
            choose_adapter(&adapters, None),
            AdapterChoice::One(&adapters[0])
        );
    }

    #[test]
    fn no_adapter() {
        assert_eq!(choose_adapter(&[], None), AdapterChoice::None);
    }

    #[test]
    fn several_adapters_are_never_guessed() {
        let adapters = [
            adapter("/dev/ttyACM0", Some("A")),
            adapter("/dev/ttyACM1", Some("B")),
        ];
        assert_eq!(choose_adapter(&adapters, None), AdapterChoice::Ambiguous);
    }

    #[test]
    fn selection_picks_among_several() {
        let adapters = [
            adapter("/dev/ttyACM0", Some("A")),
            adapter("/dev/ttyACM1", Some("B")),
        ];
        assert_eq!(
            choose_adapter(&adapters, Some("B")),
            AdapterChoice::One(&adapters[1])
        );
    }

    #[test]
    fn selected_adapter_gone_does_not_fall_back_to_another_arm() {
        let adapters = [adapter("/dev/ttyACM0", Some("A"))];
        assert_eq!(
            choose_adapter(&adapters, Some("B")),
            AdapterChoice::SelectedMissing
        );
    }
}
