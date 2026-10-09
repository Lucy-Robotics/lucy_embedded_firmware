#![no_std]
#![no_main]

use core::fmt::Write;

struct BufferWriter<'a> {
    buf: &'a mut [u8],
    offset: usize,
}

impl<'a> BufferWriter<'a> {
    fn new(buf: &'a mut [u8]) -> Self {
        Self { buf, offset: 0 }
    }

    fn as_bytes(&self) -> &[u8] {
        &self.buf[..self.offset]
    }
}

impl<'a> Write for BufferWriter<'a> {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        let bytes = s.as_bytes();
        let remaining = self.buf.len() - self.offset;
        if bytes.len() > remaining {
            return Err(core::fmt::Error);
        }
        self.buf[self.offset..self.offset + bytes.len()].copy_from_slice(bytes);
        self.offset += bytes.len();
        Ok(())
    }
}

mod channel;
use channel::Rp2040PwmChannel;
mod config;
use config::{Robot, config};
mod generated_config;
use generated_config::{BUS_SERVO_BASE, BUS_SERVO_BLOCK, BUS_SERVO_SLOTS};

use lucy_embedded_firmware_core::{
    pwm::PwmChannel,
    uart::UartChannel,
    drivers::{
        pwm_servo::{PwmServoDriver, PwmServoConfig, PwmServoModbusAdapter},
        bus_servo::{BusServoDriver, BusServoConfig, BusServoModbusAdapter},
    },
    modbus::{
        ModbusError, ModbusAdapter, RegisterView, RegisterTable, Slave,
        inter_frame_delay_us, parse_modbus_frame, route_modbus_request,
    },
};

use embedded_hal::{
    delay::DelayNs,
    digital::OutputPin,
    pwm::SetDutyCycle,
    i2c::I2c,
};

use rp2040_hal::{
    uart::{Writer, Reader, UartDevice, ValidUartPinout},
    uart::{DataBits, StopBits, UartConfig, UartPeripheral, State},
    pac,
    fugit::RateExtU32,
    fugit::MicrosDuration,
    clocks::init_clocks_and_plls,
    gpio::{bank0, Pins, FunctionPio0, FunctionUart, FunctionPwm, FunctionI2C, PullUp, PullDown},
    i2c::I2C,
    pwm::{Slices, Pwm0, Pwm4, Slice, FreeRunning},
    pio::PIOExt,
    sio::Sio,
    timer::Timer,
    watchdog::Watchdog,
    Clock
};
use smart_leds::{SmartLedsWrite, RGB8};
use ws2812_pio::Ws2812;

use usb_device::{class_prelude::*, prelude::*};
use usbd_serial::SerialPort;

use cortex_m_rt::entry;
use panic_halt as _;


enum UartError {

}

struct Rp2040UartChannel<DIR>
where
    DIR: OutputPin,
{
    uart: UartPeripheral<rp2040_hal::uart::Enabled, pac::UART0, (rp2040_hal::gpio::Pin<bank0::Gpio0, FunctionUart, PullDown>, rp2040_hal::gpio::Pin<bank0::Gpio1, FunctionUart, PullDown>)>,
    dir: DIR
}

impl<DIR> UartChannel for Rp2040UartChannel<DIR>
where
    DIR: OutputPin,
{
    type Error = UartError;

    fn write(&mut self, bytes: &[u8]) -> Result<(), Self::Error> {
        self.dir.set_high();

        self.uart.write_full_blocking(bytes);
        while self.uart.uart_is_busy() {}

        self.dir.set_low();
        Ok(())
    }

    fn read(&mut self, buffer: &mut [u8]) -> Result<usize, Self::Error> {
        Ok(0)
    }
}

#[unsafe(link_section = ".boot2")]
#[unsafe(no_mangle)]
#[used]
pub static BOOT2: [u8; 256] = rp2040_boot2::BOOT_LOADER_GENERIC_03H;


#[entry]
fn main() -> ! {
    let core = cortex_m::Peripherals::take().unwrap();
    let mut pac = pac::Peripherals::take().unwrap();
    let mut watchdog = Watchdog::new(pac.WATCHDOG);
    let sio = Sio::new(pac.SIO);

    let clocks = init_clocks_and_plls(
        12_000_000,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    ).ok().unwrap();
    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().raw());
    let timer = Timer::new(pac.TIMER, &mut pac.RESETS, &clocks);
    let pins = Pins::new(pac.IO_BANK0, pac.PADS_BANK0, sio.gpio_bank0, &mut pac.RESETS);

    let uart_tx = pins.gpio0.into_function::<FunctionUart>();
    let uart_rx = pins.gpio1.into_function::<FunctionUart>();
    let mut dir_pin = pins.gpio2.into_push_pull_output();
    dir_pin.set_low().unwrap();

    let (mut pio, sm0, _, _, _) = pac.PIO0.split(&mut pac.RESETS);

    let mut ws = Ws2812::new(
        pins.gpio18.into_function(),
        &mut pio,
        sm0,
        clocks.peripheral_clock.freq(),
        timer.count_down(),
    );
    let mut leds = [RGB8::default(); 3];

    leds[0] = RGB8 { r: 100, g: 0, b: 0 };
    ws.write(leds.iter().cloned()).unwrap();

    let mut rt = RegisterTable::default();

    let pwm_slices = Slices::new(pac.PWM, &mut pac.RESETS);
    let mut robot = config(&mut rt, pwm_slices,
        pins.gpio6, pins.gpio7, pins.gpio8, pins.gpio9, pins.gpio10, pins.gpio11);

    // LeRobot
    let uart = UartPeripheral::new(
        pac.UART0,
        (uart_tx, uart_rx),
        &mut pac.RESETS
    ).enable(
        UartConfig::new(
            1_000_000.Hz(),
            DataBits::Eight,
            None,
            StopBits::One,
        ),
        clocks.peripheral_clock.freq(),
    ).unwrap();

    let channel_uart = Rp2040UartChannel {
        dir: dir_pin,
        uart: uart
    };

    let mut driver_config = BusServoConfig {
        min_pulse: 0,
        max_pulse: 4096,
        min_angle: 0,
        max_angle: 360,
        default_angle: 90
    };

    let mut driver = BusServoDriver {
        config: driver_config,
        channel: channel_uart
    };

    /* USB */

    let usb_bus = UsbBusAllocator::new(rp2040_hal::usb::UsbBus::new(
        pac.USBCTRL_REGS,
        pac.USBCTRL_DPRAM,
        clocks.usb_clock,
        true,
        &mut pac.RESETS,
    ));

    let mut serial = SerialPort::new(&usb_bus);

    let mut usb_dev = UsbDeviceBuilder::new(&usb_bus, UsbVidPid(0x16c0, 0x27dd))
        .strings(&[StringDescriptors::default()
            .manufacturer("Custom")
            .product("Servo2040 Serial")
            .serial_number("TEST")])
        .unwrap()
        .device_class(usbd_serial::USB_CLASS_CDC)
        .build();

    let slave = Slave {
        address: 0x01,
    };


    let mut rx_buf = [0u8; 256];
    let mut tx_buf = [0u8; 256];
    let mut rx_len = 0;
    let mut rx_active_timer = false;
    let mut last_rx_micros: u64 = 0;
    let frame_gap_us = inter_frame_delay_us(115_200);

    leds[1] = RGB8 { r: 0, g: 10, b: 0 };
    ws.write(leds.iter().cloned()).unwrap();

    let mut angle = 0_u16;
    loop {
        let now = timer.get_counter().ticks();

        if usb_dev.poll(&mut [&mut serial]) {
            let mut tmp_buf = [0u8; 64];

            while let Ok(count) = serial.read(&mut tmp_buf) {
                if count == 0 {
                    break;
                }
                if rx_len + count <= rx_buf.len() {
                    rx_buf[rx_len..rx_len + count].copy_from_slice(&tmp_buf[..count]);
                    rx_len += count;
                    last_rx_micros = now;
                    rx_active_timer = true;
                } else {
                    rx_active_timer = false;
                    rx_len = 0;
                    break;
                }
            }
        }
        if rx_active_timer && (now.wrapping_sub(last_rx_micros) >= frame_gap_us) {
            rx_active_timer = false;
            if rx_len >= 4 {
                let raw_request = parse_modbus_frame(&slave, &rx_buf[..rx_len]);
                match raw_request {
                    Ok(request) => {
                        if let Ok(n) =
                            route_modbus_request(slave.address, &rt, request, &mut tx_buf)
                        {
                            let _ = serial.write(&tx_buf[..n]);
                        }
                    }
                    Err(ModbusError::InvalidAddress) => {
                        // Silence for other slaves (RTU spec).
                    }
                    Err(_) => {
                        // Malformed frames: no response.
                    }
                }
            }
            rx_len = 0;
        }

        for slot in 0..BUS_SERVO_SLOTS {
            let base = BUS_SERVO_BASE + slot * BUS_SERVO_BLOCK;
            let rv = RegisterView {
                table: &rt,
                base_register: base,
                nb_register: BUS_SERVO_BLOCK,
            };
            let mut adapter = BusServoModbusAdapter {
                base_register: base,
                id_reg_off: 0,
                angle_reg_off: 1,
                cmd_reg_off: 2,
                driver: &mut driver,
            };
            adapter.tick(&rv);
        }
    }
}
