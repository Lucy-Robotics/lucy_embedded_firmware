use lucy_embedded_firmware_core::board_layout::{
    parse_adc_channel, parse_servo_channel, parse_uart_channel,
};

#[test]
fn parse_servo_adc_uart_channels() {
    assert_eq!(parse_servo_channel("Servo1"), Some(1));
    assert_eq!(parse_servo_channel("Servo18"), Some(18));
    assert_eq!(parse_servo_channel("Servo0"), None);
    assert_eq!(parse_servo_channel("Servo19"), Some(19));
    assert_eq!(parse_adc_channel("ADC0"), Some(0));
    assert_eq!(parse_adc_channel("ADC3"), Some(3));
    assert_eq!(parse_uart_channel("UART0"), Some((0, None)));
    assert_eq!(parse_uart_channel("UART0:1"), Some((0, Some(1))));
    assert_eq!(parse_uart_channel("UART1:3"), Some((1, Some(3))));
    assert_eq!(parse_uart_channel("ADC0"), None);
}
