//! UART bus-servo RP2040 board (`board_class: bus_servo_only`).
//!
//! YAML selects enabled Feetech IDs via `GENERATED_BUS_DEVICES`. Pinout:
//! UART0 TX=GPIO0, RX=GPIO1, DIR=GPIO2 @ 1 Mbaud. Shared bring-up /
//! banks live in `lucy_embedded_firmware_rp2040_support`.
#![no_std]
#![no_main]

mod board_layout;

include!(concat!(env!("OUT_DIR"), "/config.rs"));

use lucy_embedded_firmware_core::modbus::{RegisterTable, Slave};
use lucy_embedded_firmware_rp2040_support::{
    build_usb_device, resolve_usb_serial, BusBank, BusBankDevice, ModbusCdcState, Rp2040UartChannel,
    MAX_BUS_DEVICES,
};

use cortex_m_rt::entry;
use embedded_hal::digital::OutputPin;
use panic_halt as _;
use rp2040_hal::{
    clocks::init_clocks_and_plls,
    fugit::RateExtU32,
    gpio::{FunctionUart, Pins},
    pac,
    sio::Sio,
    timer::Timer,
    uart::{DataBits, StopBits, UartConfig, UartPeripheral},
    watchdog::Watchdog,
    Clock,
};
use usb_device::class_prelude::*;

#[unsafe(link_section = ".boot2")]
#[unsafe(no_mangle)]
#[used]
pub static BOOT2: [u8; 256] = rp2040_boot2::BOOT_LOADER_GENERIC_03H;

#[entry]
fn main() -> ! {
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
    )
    .ok()
    .unwrap();
    let timer = Timer::new(pac.TIMER, &mut pac.RESETS, &clocks);
    let pins = Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    let uart_tx = pins.gpio0.into_function::<FunctionUart>();
    let uart_rx = pins.gpio1.into_function::<FunctionUart>();
    let mut dir_pin = pins.gpio2.into_push_pull_output();
    let _ = dir_pin.set_low();

    let uart = UartPeripheral::new(pac.UART0, (uart_tx, uart_rx), &mut pac.RESETS)
        .enable(
            UartConfig::new(1_000_000.Hz(), DataBits::Eight, None, StopBits::One),
            clocks.peripheral_clock.freq(),
        )
        .unwrap();

    let channel = Rp2040UartChannel {
        uart,
        dir: dir_pin,
    };

    let mut device_buf = [BusBankDevice {
        device_id: 0,
        base_register: 0,
        config: lucy_embedded_firmware_core::drivers::BusServoConfig {
            min_pulse: 0,
            max_pulse: 4095,
            min_angle: 0,
            max_angle: 6283,
            default_angle: 3142,
        },
    }; MAX_BUS_DEVICES];
    let mut n = 0usize;
    if GENERATED_HAS_BUS {
        for d in GENERATED_BUS_DEVICES.iter().take(MAX_BUS_DEVICES) {
            // Only UART0 is wired on this board firmware.
            if d.uart != 0 {
                continue;
            }
            device_buf[n] = BusBankDevice {
                device_id: d.device_id,
                base_register: d.base_register,
                config: d.config,
            };
            n += 1;
        }
    }
    let mut bus_bank = BusBank::new(channel, &device_buf[..n]);

    let usb_bus = UsbBusAllocator::new(rp2040_hal::usb::UsbBus::new(
        pac.USBCTRL_REGS,
        pac.USBCTRL_DPRAM,
        clocks.usb_clock,
        true,
        &mut pac.RESETS,
    ));
    let usb_serial = resolve_usb_serial(GENERATED_USB_SERIAL_ID);
    let (mut serial, mut picotool, mut usb_dev) =
        build_usb_device(&usb_bus, "Lucy RP2040 Bus Servo", usb_serial);

    let slave = Slave {
        address: GENERATED_SLAVE_ADDRESS,
    };
    let rt = RegisterTable::default();
    bus_bank.seed_id_registers(&rt);

    let mut modbus = ModbusCdcState::new(115_200);

    loop {
        let now = timer.get_counter().ticks();
        modbus.poll_usb(&mut usb_dev, &mut serial, &mut picotool, now);
        modbus.try_route_frame(&slave, &rt, &mut serial, now);
        bus_bank.tick(&rt);
    }
}
