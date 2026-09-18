/// Generated board instance consts (virtual pins, configs).
#[allow(unused_imports, dead_code)]
mod _generated {
    include!(concat!(env!("OUT_DIR"), "/config.rs"));
}
pub use _generated::*;

use rp2040_hal::{
    uart::{Writer, Reader, UartDevice, ValidUartPinout},
    uart::{DataBits, StopBits, UartConfig, UartPeripheral, State},
    pac,
    fugit::RateExtU32,
    fugit::MicrosDuration,
    clocks::init_clocks_and_plls,
    gpio::{bank0, Pin, Pins, FunctionPio0, FunctionUart, FunctionPwm, FunctionI2C, FunctionNull, PullUp, PullDown},
    i2c::I2C,
    pwm::{Slices, A, B, Pwm0, Pwm3, Pwm4, Pwm5, Pwm6, Slice, FreeRunning, Channel},
    pio::PIOExt,
    sio::Sio,
    timer::Timer,
    watchdog::Watchdog,
    Clock
};

use embedded_hal::{
    delay::DelayNs,
    digital::OutputPin,
    pwm::SetDutyCycle,
    i2c::I2c,
};

use lucy_embedded_firmware_core::{
    pwm::{PwmChannel},
    uart::UartChannel,
    drivers::{
        pwm_servo::{PwmServoDriver, PwmServoConfig, PwmServoModbusAdapter},
        bus_servo::{BusServoDriver, BusServoConfig, BusServoModbusAdapter},
    },
    modbus::{ModbusError, ModbusAdapter, RegisterView, RegisterTable, Slave, parse_modbus_frame, route_modbus_request}
};

use crate::channel::Rp2040PwmChannel;

type Gpio6 = Pin<bank0::Gpio6, FunctionNull, PullDown>;
type Gpio7 = Pin<bank0::Gpio7, FunctionNull, PullDown>;
type Gpio8 = Pin<bank0::Gpio8, FunctionNull, PullDown>;
type Gpio9 = Pin<bank0::Gpio9, FunctionNull, PullDown>;
type Gpio10 = Pin<bank0::Gpio10, FunctionNull, PullDown>;
type Gpio11 = Pin<bank0::Gpio11, FunctionNull, PullDown>;

type Servo7 = Rp2040PwmChannel<Channel<Slice<Pwm3, FreeRunning>, A>>;
type Servo8 = Rp2040PwmChannel<Channel<Slice<Pwm3, FreeRunning>, B>>;
type Servo9 = Rp2040PwmChannel<Channel<Slice<Pwm4, FreeRunning>, A>>;
type Servo10 = Rp2040PwmChannel<Channel<Slice<Pwm4, FreeRunning>, B>>;
type Servo11 = Rp2040PwmChannel<Channel<Slice<Pwm5, FreeRunning>, A>>;
type Servo12 = Rp2040PwmChannel<Channel<Slice<Pwm5, FreeRunning>, B>>;

pub struct Robot {
    pub servo1: PwmServoModbusAdapter<3142, Servo7>,
    pub servo2: PwmServoModbusAdapter<3142, Servo8>, 
    pub servo3: PwmServoModbusAdapter<3142, Servo9>, 
    pub servo4: PwmServoModbusAdapter<3142, Servo10>, 
    pub servo5: PwmServoModbusAdapter<3142, Servo11>, 
    pub servo6: PwmServoModbusAdapter<3142, Servo12>,
}

impl Robot {
    pub fn tick(&mut self, rt: &RegisterTable) {
        let rv = RegisterView {
            table: rt,
            base_register: self.servo1.get_base_register(),
            nb_register: self.servo1.get_nb_register(),
        };
        self.servo1.tick(&rv);

        let rv = RegisterView {
            table: rt,
            base_register: self.servo2.get_base_register(),
            nb_register: self.servo2.get_nb_register(),
        };
        self.servo2.tick(&rv);

        let rv = RegisterView {
            table: rt,
            base_register: self.servo3.get_base_register(),
            nb_register: self.servo3.get_nb_register(),
        };
        self.servo3.tick(&rv);

        let rv = RegisterView {
            table: rt,
            base_register: self.servo4.get_base_register(),
            nb_register: self.servo4.get_nb_register(),
        };
        self.servo4.tick(&rv);

        let rv = RegisterView {
            table: rt,
            base_register: self.servo5.get_base_register(),
            nb_register: self.servo5.get_nb_register(),
        };
        self.servo5.tick(&rv);

        let rv = RegisterView {
            table: rt,
            base_register: self.servo6.get_base_register(),
            nb_register: self.servo6.get_nb_register(),
        };
        self.servo6.tick(&rv);
    }
}

pub fn config(
    rt: &mut RegisterTable, slices: Slices,
    gpio6: Gpio6, gpio7: Gpio7, gpio8: Gpio8, gpio9: Gpio9, gpio10: Gpio10, gpio11: Gpio11,
) -> Robot {
    // SERVO 1 7 WRIST
    let mut pwm = slices.pwm3;
    pwm.set_div_int(125);
    pwm.set_top(20000 - 1);
    pwm.channel_a.output_to(gpio6);
    pwm.channel_b.output_to(gpio7);
    pwm.enable();

    let mut channel_pwm = Rp2040PwmChannel {
        channel: pwm.channel_a,
    };

    let mut driver_config = PwmServoConfig {
        min_pulse: 500,
        max_pulse: 2500,
        min_angle: 0,
        max_angle: 5236,
        default_angle: 1571
    };

    let mut driver = PwmServoDriver {
        config: driver_config,
        channel: channel_pwm
    };

    let mut adapter1 = PwmServoModbusAdapter {
        base_register: 0x00,
        cmd_reg_off: 0,
        angle_reg_off: 1,
        driver: driver
    };

    // SERVO 2
    let mut channel_pwm2 = Rp2040PwmChannel {
        channel: pwm.channel_b,
    };

    let mut driver_config2 = PwmServoConfig {
        min_pulse: 1000,
        max_pulse: 2000,
        min_angle: 0,
        max_angle: 5236,
        default_angle: 1571
    };

    let mut driver2 = PwmServoDriver {
        config: driver_config2,
        channel: channel_pwm2
    };

    let mut adapter2 = PwmServoModbusAdapter {
        base_register: 0x00,
        cmd_reg_off: 0,
        angle_reg_off: 1,
        driver: driver2
    };

    // SERVO 3
    let mut pwm = slices.pwm4;
    pwm.set_div_int(125);
    pwm.set_top(20000 - 1);
    pwm.channel_a.output_to(gpio8);
    pwm.channel_b.output_to(gpio9);
    pwm.enable();
    let mut channel_pwm = Rp2040PwmChannel {
        channel: pwm.channel_a,
    };

    let driver_config = PwmServoConfig {
        min_pulse: 600,
        max_pulse: 2500,
        min_angle: 0,
        max_angle: 3142,
        default_angle: 1571
    };

    let driver = PwmServoDriver {
        config: driver_config,
        channel: channel_pwm
    };

    let mut adapter3 = PwmServoModbusAdapter {
        base_register: 0x00,
        cmd_reg_off: 0,
        angle_reg_off: 1,
        driver: driver
    };

    // SERVO 4
      let mut channel_pwm = Rp2040PwmChannel {
        channel: pwm.channel_b,
    };

    let driver_config = PwmServoConfig {
        min_pulse: 600,
        max_pulse: 2500,
        min_angle: 0,
        max_angle: 3142,
        default_angle: 1571
    };

    let driver = PwmServoDriver {
        config: driver_config,
        channel: channel_pwm
    };

    let mut adapter4 = PwmServoModbusAdapter {
        base_register: 0x00,
        cmd_reg_off: 0,
        angle_reg_off: 1,
        driver: driver
    };

    // SERVO 5
    let mut pwm = slices.pwm5;
    pwm.set_div_int(125);
    pwm.set_top(20000 - 1);
    pwm.channel_a.output_to(gpio10);
    pwm.channel_b.output_to(gpio11);
    pwm.enable();
    let mut channel_pwm = Rp2040PwmChannel {
        channel: pwm.channel_a,
    };

    let driver_config = PwmServoConfig {
        min_pulse: 600,
        max_pulse: 2500,
        min_angle: 0,
        max_angle: 3142,
        default_angle: 1571
    };

    let driver = PwmServoDriver {
        config: driver_config,
        channel: channel_pwm
    };

    let mut adapter5 = PwmServoModbusAdapter {
        base_register: 0x00,
        cmd_reg_off: 0,
        angle_reg_off: 1,
        driver: driver
    };

    // SERVO 6
    let mut channel_pwm = Rp2040PwmChannel {
        channel: pwm.channel_b,
    };

    let driver_config = PwmServoConfig {
        min_pulse: 600,
        max_pulse: 2500,
        min_angle: 0,
        max_angle: 3142,
        default_angle: 1571
    };

    let driver = PwmServoDriver {
        config: driver_config,
        channel: channel_pwm
    };

    let mut adapter6 = PwmServoModbusAdapter {
        base_register: 0x00,
        cmd_reg_off: 0,
        angle_reg_off: 1,
        driver: driver
    };

    Robot {
        servo1: adapter1,
        servo2: adapter2,
        servo3: adapter3,
        servo4: adapter4,
        servo5: adapter5,
        servo6: adapter6,
    }
}
