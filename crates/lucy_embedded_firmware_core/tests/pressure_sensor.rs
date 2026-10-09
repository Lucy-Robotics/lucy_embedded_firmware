use lucy_embedded_firmware_core::drivers::{
    PressureSensorConfig, PressureSensorDriver, PressureSensorModbusAdapter,
};
use lucy_embedded_firmware_core::modbus::{ModbusAdapter, RegisterTable, RegisterView};


    #[test]
    fn pressure_block_size_is_two() {
        let mut adapter = PressureSensorModbusAdapter {
            base_register: 4,
            cmd_reg_off: 0,
            value_reg_off: 1,
            driver: PressureSensorDriver::new(PressureSensorConfig {
                min_value: 0,
                max_value: 4095,
            }),
        };
        assert_eq!(adapter.get_nb_register(), 2);
        assert_eq!(adapter.get_base_register(), 4);

        let rt = RegisterTable::default();
        rt.registers[4].set(1);
        let rv = RegisterView {
            table: &rt,
            base_register: 4,
            nb_register: 2,
        };
        adapter.tick(&rv);
        assert_eq!(rt.registers[4].get(), 0);
        assert_eq!(rt.registers[5].get(), 0);
    }

    #[test]
    fn placeholder_clamps_stored_value() {
        let mut driver = PressureSensorDriver {
            config: PressureSensorConfig {
                min_value: 100,
                max_value: 200,
            },
            last_value: 999,
        };
        assert_eq!(driver.read_placeholder(), 200);
        driver.last_value = 50;
        assert_eq!(driver.read_placeholder(), 100);
    }
