use lucy_embedded_firmware_core::board_layout::{
    layout_for_board, BoardLayout, HardwareIdentity, Rp2040Servo2040Layout,
};


    #[test]
    fn servo2040_servo_gpio_table() {
        let layout = Rp2040Servo2040Layout;
        assert_eq!(
            layout.resolve("Servo1"),
            Some(HardwareIdentity::PwmGpio {
                servo_index: 1,
                gpio: 0
            })
        );
        assert_eq!(
            layout.resolve("Servo18"),
            Some(HardwareIdentity::PwmGpio {
                servo_index: 18,
                gpio: 17
            })
        );
        assert_eq!(layout.resolve("Servo19"), None);
        assert_eq!(layout.resolve("Servo0"), None);
    }

    #[test]
    fn servo2040_adc_uart_i2c() {
        let layout = Rp2040Servo2040Layout;
        assert_eq!(
            layout.resolve("ADC0"),
            Some(HardwareIdentity::Adc { channel: 0 })
        );
        assert_eq!(layout.resolve("ADC4"), None);
        assert_eq!(
            layout.resolve("UART0:1"),
            Some(HardwareIdentity::UartBus {
                uart: 0,
                device_id: Some(1)
            })
        );
        assert_eq!(
            layout.resolve("I2C0:PCA9685:3"),
            Some(HardwareIdentity::I2cPwm {
                bus: 0,
                device: 0x40,
                channel: 3
            })
        );
    }

    #[test]
    fn layout_for_board_classes() {
        assert!(layout_for_board("internal_servo_only", None).is_some());
        assert!(layout_for_board("internal_servo_i2c_pwm", None).is_some());
        assert!(layout_for_board("bus_servo_only", None).is_some());
        assert!(layout_for_board(
            "",
            Some("firmwares/rp2040_servo2040")
        )
        .is_some());
        // Shared crate path must not override UART-only board_class layout.
        assert_eq!(
            layout_for_board("bus_servo_only", Some("firmwares/rp2040_servo2040"))
                .unwrap()
                .resolve("UART0:1"),
            Some(HardwareIdentity::UartBus {
                uart: 0,
                device_id: Some(1)
            })
        );
        assert_eq!(
            layout_for_board("bus_servo_only", Some("firmwares/rp2040_servo2040"))
                .unwrap()
                .resolve("Servo1"),
            None
        );
    }
