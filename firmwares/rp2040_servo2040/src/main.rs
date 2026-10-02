#![no_std]
#![no_main]

mod board_layout;
mod config;
mod pwm_bank;

use config::{GENERATED_PWM_DEVICES, GENERATED_SLAVE_ADDRESS, GENERATED_USB_SERIAL_ID};
use lucy_embedded_firmware_rp2040_support::PicoToolReset;
use pwm_bank::PwmBank;

use lucy_embedded_firmware_core::modbus::{
    inter_frame_delay_us, parse_modbus_frame, route_modbus_request, ModbusError, RegisterTable,
    Slave,
};

use rp2040_hal::{
    clocks::init_clocks_and_plls,
    gpio::{DynPinId, FunctionNull, FunctionPio0, Pins, PullDown},
    pac,
    pio::PIOExt,
    sio::Sio,
    timer::Timer,
    watchdog::Watchdog,
    Clock,
};
use smart_leds::{SmartLedsWrite, RGB8};
use usb_device::{class_prelude::*, prelude::*};
use usbd_serial::SerialPort;
use ws2812_pio::Ws2812;

use cortex_m_rt::entry;
use panic_halt as _;

#[unsafe(link_section = ".boot2")]
#[unsafe(no_mangle)]
#[used]
pub static BOOT2: [u8; 256] = rp2040_boot2::BOOT_LOADER_GENERIC_03H;

fn unreset_pwm(resets: &mut pac::RESETS) {
    resets.reset().modify(|_, w| w.pwm().clear_bit());
    while resets.reset_done().read().pwm().bit_is_clear() {}
}

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
    )
    .ok()
    .unwrap();
    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().raw());
    let timer = Timer::new(pac.TIMER, &mut pac.RESETS, &clocks);
    let pins = Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    let (mut pio, sm0, _, _, _) = pac.PIO0.split(&mut pac.RESETS);
    let mut ws = Ws2812::new(
        pins.gpio18.into_function::<FunctionPio0>(),
        &mut pio,
        sm0,
        clocks.peripheral_clock.freq(),
        timer.count_down(),
    );
    let mut leds = [RGB8::default(); 3];
    leds[0] = RGB8 { r: 0, g: 10, b: 0 };
    let _ = ws.write(leds.iter().cloned());

    let mut pin_pool: [Option<PinDynNull>; 18] = [
        Some(pins.gpio0.into_dyn_pin()),
        Some(pins.gpio1.into_dyn_pin()),
        Some(pins.gpio2.into_dyn_pin()),
        Some(pins.gpio3.into_dyn_pin()),
        Some(pins.gpio4.into_dyn_pin()),
        Some(pins.gpio5.into_dyn_pin()),
        Some(pins.gpio6.into_dyn_pin()),
        Some(pins.gpio7.into_dyn_pin()),
        Some(pins.gpio8.into_dyn_pin()),
        Some(pins.gpio9.into_dyn_pin()),
        Some(pins.gpio10.into_dyn_pin()),
        Some(pins.gpio11.into_dyn_pin()),
        Some(pins.gpio12.into_dyn_pin()),
        Some(pins.gpio13.into_dyn_pin()),
        Some(pins.gpio14.into_dyn_pin()),
        Some(pins.gpio15.into_dyn_pin()),
        Some(pins.gpio16.into_dyn_pin()),
        Some(pins.gpio17.into_dyn_pin()),
    ];

    unreset_pwm(&mut pac.RESETS);
    let pwm = pac.PWM;

    let mut device_buf: [(u8, u16, lucy_embedded_firmware_core::drivers::PwmServoConfig); 18] =
        [(
            0,
            0,
            lucy_embedded_firmware_core::drivers::PwmServoConfig {
                min_pulse: 1000,
                max_pulse: 2000,
                min_angle: 0,
                max_angle: 3142,
                default_angle: 1571,
            },
        ); 18];
    let mut n = 0usize;
    for d in GENERATED_PWM_DEVICES.iter().take(18) {
        device_buf[n] = (d.gpio, d.base_register, d.config);
        n += 1;
    }
    let pwm_bank = PwmBank::from_null_pool(&pwm, &mut pin_pool, &device_buf[..n]);

    /* USB CDC + picotool reset (VID 0x2e8a) */
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
            .product("Lucy RP2040")
            .serial_number(usb_serial)])
        .unwrap()
        .composite_with_iads()
        .max_packet_size_0(64)
        .unwrap()
        .build();

    let slave = Slave {
        address: GENERATED_SLAVE_ADDRESS,
    };
    let mut rt = RegisterTable::default();
    let mut rx_buf = [0u8; 256];
    let mut tx_buf = [0u8; 256];
    let mut rx_len = 0usize;
    let mut rx_active_timer = false;
    let mut last_rx_micros: u64 = 0;
    let frame_gap_us = inter_frame_delay_us(115_200);

    let _ = delay;

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

        pwm_bank.tick(&pwm, &rt);
    }
}

type PinDynNull = rp2040_hal::gpio::Pin<DynPinId, FunctionNull, PullDown>;
