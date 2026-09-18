use crate::uart::UartChannel;
use crate::{
    modbus::{ModbusAdapter, RegisterView},
    utils::millirad_to_pulse,
};

fn compute_checksum(payload: &[u8]) -> u8 {
    let sum: u8 = payload.iter().fold(0u8, |acc, &x| acc.wrapping_add(x));
    !sum
}

/// Bus-servo configuration. Angle fields are **milliradians** (`rad × 1000`).
pub struct BusServoConfig {
    pub min_pulse: u16,
    pub max_pulse: u16,
    pub min_angle: u16,
    pub max_angle: u16,
    pub default_angle: u16,
}

pub struct BusServoDriver<U> {
    pub config: BusServoConfig,
    pub channel: U,
}

impl<U: UartChannel> BusServoDriver<U> {
    pub fn enable_led(&mut self, id: u8, enable: bool) {
        const REG: u8 = 0x2F;
        const INST_WRITE: u8 = 0x03;
        let length = 4u8;
        let value = if enable { 1u8 } else { 0u8 };

        let mut frame = [
            0xFF,
            0xFF,
            id,
            length,
            INST_WRITE,
            REG,
            value,
            0x00,
        ];

        let payload_to_sum = &frame[2..frame.len() - 1];
        let checksum = compute_checksum(payload_to_sum);
        frame[frame.len() - 1] = checksum;

        let _ = self.channel.write(&frame);
    }

    pub fn set_servo_mode(&mut self, id: u8) {
        const REG_TORQUE_ENABLE: u8 = 0x21;
        const INST_WRITE: u8 = 0x03;
        let length = 4u8;
        let value = 0x00;

        let mut frame = [
            0xFF,
            0xFF,
            id,
            length,
            INST_WRITE,
            REG_TORQUE_ENABLE,
            value,
            0x00,
        ];

        let payload_to_sum = &frame[2..frame.len() - 1];
        let checksum = compute_checksum(payload_to_sum);
        frame[frame.len() - 1] = checksum;

        let _ = self.channel.write(&frame);
    }

    pub fn enable_torque(&mut self, id: u8, enable: bool) {
        const REG_TORQUE_ENABLE: u8 = 0x28;
        const INST_WRITE: u8 = 0x03;
        let length = 4u8;
        let value = if enable { 1u8 } else { 0u8 };

        let mut frame = [
            0xFF,
            0xFF,
            id,
            length,
            INST_WRITE,
            REG_TORQUE_ENABLE,
            value,
            0x00,
        ];

        let payload_to_sum = &frame[2..frame.len() - 1];
        let checksum = compute_checksum(payload_to_sum);
        frame[frame.len() - 1] = checksum;

        let _ = self.channel.write(&frame);
    }

    /// `angle_millirad` is radians × 1000.
    pub fn move_angle(&mut self, id: u8, angle_millirad: u16) {
        let pulse = millirad_to_pulse(
            angle_millirad,
            self.config.min_angle,
            self.config.max_angle,
            self.config.min_pulse,
            self.config.max_pulse,
        );

        const REG_TARGET_POSITION: u8 = 0x2A;
        const INST_WRITE: u8 = 0x03;
        let length = 9u8;

        let time: u16 = 0;
        let speed: u16 = 1000;

        let [pos_l, pos_h] = pulse.to_le_bytes();
        let [time_l, time_h] = time.to_le_bytes();
        let [spd_l, spd_h] = speed.to_le_bytes();

        let mut frame = [
            0xFF,
            0xFF,
            id,
            length,
            INST_WRITE,
            REG_TARGET_POSITION,
            pos_l,
            pos_h,
            time_l,
            time_h,
            spd_l,
            spd_h,
            0x00,
        ];
        let payload_to_sum = &frame[2..frame.len() - 1];
        let checksum = compute_checksum(payload_to_sum);
        frame[frame.len() - 1] = checksum;

        let _ = self.channel.write(&frame);
    }
}

pub struct BusServoModbusAdapter<'a, U> {
    pub base_register: u16,
    pub cmd_reg_off: u16,
    pub id_reg_off: u16,
    pub angle_reg_off: u16,
    pub driver: &'a mut BusServoDriver<U>,
}

impl<'a, U: UartChannel> ModbusAdapter for BusServoModbusAdapter<'a, U> {
    fn tick(&mut self, rv: &RegisterView) {
        let cmd = rv.read_register(self.cmd_reg_off);
        if cmd == 0 {
            return;
        }
        let id = rv.read_register(self.id_reg_off) as u8;
        rv.write_register(self.cmd_reg_off, 0);
        match cmd {
            1 => {
                let angle = rv.read_register(self.angle_reg_off);
                self.driver.move_angle(id, angle);
            }
            2 => {
                self.driver.set_servo_mode(id);
            }
            3 => {
                self.driver.enable_torque(id, true);
            }
            4 => {
                self.driver.enable_led(id, false);
            }
            5 => {
                self.driver.enable_torque(id, false);
            }
            _ => {}
        }
    }

    fn get_nb_register(&self) -> u16 {
        3
    }

    fn get_base_register(&self) -> u16 {
        self.base_register
    }
}
