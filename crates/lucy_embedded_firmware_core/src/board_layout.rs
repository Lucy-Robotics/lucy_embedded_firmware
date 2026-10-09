#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum HardwareIdentity {
    PwmGpio { servo_index: u8, gpio: u8 },
    Adc { channel: u8 },
    UartBus { uart: u8, device_id: Option<u8> },
    I2cPwm {
        bus: u8,
        device: u8,
        channel: u8,
    },
}

pub trait BoardLayout {
    fn resolve(&self, channel: &str) -> Option<HardwareIdentity>;
}

pub fn parse_uart_channel(channel: &str) -> Option<(u8, Option<u8>)> {
    let rest = channel.strip_prefix("UART")?;
    let (uart_str, device) = match rest.split_once(':') {
        Some((u, d)) => (u, Some(d.parse::<u8>().ok()?)),
        None => (rest, None),
    };
    let uart = uart_str.parse::<u8>().ok()?;
    Some((uart, device))
}

pub fn parse_adc_channel(channel: &str) -> Option<u8> {
    let rest = channel.strip_prefix("ADC")?;
    rest.parse::<u8>().ok()
}

pub fn parse_servo_channel(channel: &str) -> Option<u8> {
    let rest = channel.strip_prefix("Servo")?;
    let idx = rest.parse::<u8>().ok()?;
    if idx == 0 {
        None
    } else {
        Some(idx)
    }
}
