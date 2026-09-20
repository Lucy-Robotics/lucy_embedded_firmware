//! UART bus-servo RP2040 board (`board_class: bus_servo_only`).
//!
//! YAML (`config.yaml` / `LUCY_FIRMWARE_CONFIG`) selects enabled Feetech IDs and
//! Modbus bases via `GENERATED_BUS_DEVICES`. Hardware pinout matches the SO-ARM
//! PoC: UART0 TX=GPIO0, RX=GPIO1, DIR=GPIO2 @ 1 Mbaud.
#![no_std]
#![no_main]

mod board_layout;
mod bus_bank;
mod uart_channel;

include!(concat!(env!("OUT_DIR"), "/config.rs"));

use bus_bank::{BusBank, BusBankDevice};
use lucy_embedded_firmware_core::modbus::{
    inter_frame_delay_us, parse_modbus_frame, route_modbus_request, ModbusError, RegisterTable,
    Slave,
};
use lucy_embedded_firmware_rp2040_support::PicoToolReset;
use uart_channel::Rp2040UartChannel;

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
use usb_device::{class_prelude::*, prelude::*};
use usbd_serial::SerialPort;

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
    }; bus_bank::MAX_BUS_DEVICES];
    let mut n = 0usize;
    for d in GENERATED_BUS_DEVICES.iter().take(bus_bank::MAX_BUS_DEVICES) {
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
    let mut bus_bank = BusBank::new(channel, &device_buf[..n]);

    let usb_bus = UsbBusAllocator::new(rp2040_hal::usb::UsbBus::new(
        pac.USBCTRL_REGS,
        pac.USBCTRL_DPRAM,
        clocks.usb_clock,
        true,
        &mut pac.RESETS,
    ));
    let mut serial = SerialPort::new(&usb_bus);
    let mut picotool = PicoToolReset::new(&usb_bus);
    let usb_serial = {
        let s = GENERATED_USB_SERIAL_ID;
        if s.is_empty() {
            "TEST"
        } else {
            s
        }
    };
    let mut usb_dev = UsbDeviceBuilder::new(&usb_bus, UsbVidPid(0x2e8a, 0x000a))
        .strings(&[StringDescriptors::default()
            .manufacturer("Sentience")
            .product("Lucy RP2040 Bus Servo")
            .serial_number(usb_serial)])
        .unwrap()
        .composite_with_iads()
        .max_packet_size_0(64)
        .unwrap()
        .build();

    let slave = Slave {
        address: GENERATED_SLAVE_ADDRESS,
    };
    let rt = RegisterTable::default();
    bus_bank.seed_id_registers(&rt);

    let mut rx_buf = [0u8; 256];
    let mut tx_buf = [0u8; 256];
    let mut rx_len = 0usize;
    let mut rx_active_timer = false;
    let mut last_rx_micros: u64 = 0;
    let frame_gap_us = inter_frame_delay_us(115_200);

    loop {
        let now = timer.get_counter().ticks();

        if usb_dev.poll(&mut [&mut serial, &mut picotool]) {
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
                match parse_modbus_frame(&slave, &rx_buf[..rx_len]) {
                    Ok(request) => {
                        if let Ok(n) =
                            route_modbus_request(slave.address, &rt, request, &mut tx_buf)
                        {
                            let _ = serial.write(&tx_buf[..n]);
                        }
                    }
                    Err(ModbusError::InvalidAddress) => {}
                    Err(_) => {}
                }
            }
            rx_len = 0;
        }

        bus_bank.tick(&rt);
    }
}
