//! Pimoroni Servo2040 — one board firmware for all Servo2040 `board_class`es.
//!
//! YAML (`config.yaml` / `LUCY_FIRMWARE_CONFIG`) enables PWM, UART bus, I2C-PWM,
//! and ADC banks via codegen (`GENERATED_HAS_*`). MCU banks / USB / Modbus live
//! in `lucy_embedded_firmware_rp2040_support`; this crate owns pinmux and the
//! WS2812 status LED.
//!
//! UART bus (`GENERATED_HAS_BUS`): UART0 half-duplex Feetech/STS on GPIO0 TX,
//! GPIO1 RX, GPIO2 DIR @ 1 Mbaud (Servo1–3 silk). Do not also enable those pads
//! as PWM. This is the SO-ARM101 **Servo2040 UART** Feetech path (alternate to
//! host `firmwares/linux` USB-serial Feetech). InMoov uses this same package for
//! PWM / I2C PCA9685 / ADC profiles without the host Feetech process.
#![no_std]
#![no_main]

mod board_layout;
mod config;

use config::{
    GENERATED_ADC_DEVICES, GENERATED_BUS_DEVICES, GENERATED_HAS_ADC, GENERATED_HAS_BUS,
    GENERATED_HAS_I2C_PWM, GENERATED_HAS_PWM, GENERATED_I2C_PWM_DEVICES, GENERATED_PWM_DEVICES,
    GENERATED_SLAVE_ADDRESS, GENERATED_USB_SERIAL_ID,
};
use lucy_embedded_firmware_core::modbus::{RegisterTable, Slave};
use lucy_embedded_firmware_rp2040_support::{
    build_usb_device, resolve_usb_serial, unreset_pwm, AdcBank, AdcBankDevice, BusBank,
    BusBankDevice, I2cPwmBank, I2cPwmBankDevice, ModbusCdcState, PwmBank, Rp2040UartChannel,
    MAX_ADC_DEVICES, MAX_BUS_DEVICES, MAX_I2C_PWM_DEVICES,
};

use cortex_m_rt::entry;
use embedded_hal::digital::OutputPin;
use panic_halt as _;
use rp2040_hal::{
    clocks::init_clocks_and_plls,
    fugit::{ExtU32, RateExtU32},
    gpio::{DynPinId, FunctionNull, FunctionPio0, FunctionUart, Pins, PullDown},
    pac,
    pio::PIOExt,
    sio::Sio,
    timer::Timer,
    uart::{DataBits, StopBits, UartConfig, UartPeripheral},
    watchdog::Watchdog,
    Clock,
};
use smart_leds::{SmartLedsWrite, RGB8};
use usb_device::class_prelude::*;
use ws2812_pio::Ws2812;

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
    )
    .ok()
    .unwrap();
    let delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().raw());
    let timer = Timer::new(pac.TIMER, &mut pac.RESETS, &clocks);
    let pins = Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    // Board-only: WS2812 status LED on GPIO18.
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

    // GPIO0–2: UART bus (TX/RX/DIR) when HAS_BUS, else PWM Servo1–3.
    let mut gpio0 = Some(pins.gpio0);
    let mut gpio1 = Some(pins.gpio1);
    let mut gpio2 = Some(pins.gpio2);

    let mut bus_bank = if GENERATED_HAS_BUS {
        let uart_tx = gpio0.take().unwrap().into_function::<FunctionUart>();
        let uart_rx = gpio1.take().unwrap().into_function::<FunctionUart>();
        let mut dir_pin = gpio2.take().unwrap().into_push_pull_output();
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
                min_angle: 0.0,
                max_angle: core::f32::consts::TAU,
                default_angle: core::f32::consts::PI,
            },
        }; MAX_BUS_DEVICES];
        let mut n = 0usize;
        for d in GENERATED_BUS_DEVICES.iter().take(MAX_BUS_DEVICES) {
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
        Some(BusBank::new(channel, &device_buf[..n]))
    } else {
        None
    };

    let mut pin_pool: [Option<PinDynNull>; 18] = [
        gpio0.map(|p| p.into_dyn_pin()),
        gpio1.map(|p| p.into_dyn_pin()),
        gpio2.map(|p| p.into_dyn_pin()),
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

    let pwm_hw = if GENERATED_HAS_PWM {
        unreset_pwm(&mut pac.RESETS);
        Some(pac.PWM)
    } else {
        None
    };

    let mut pwm_device_buf: [(u8, u16, lucy_embedded_firmware_core::drivers::PwmServoConfig); 18] =
        [(
            0,
            0,
            lucy_embedded_firmware_core::drivers::PwmServoConfig {
                min_pulse: 1000,
                max_pulse: 2000,
                min_angle: 0.0,
                max_angle: core::f32::consts::PI,
                default_angle: core::f32::consts::FRAC_PI_2,
            },
        ); 18];
    let mut pwm_n = 0usize;
    if GENERATED_HAS_PWM {
        for d in GENERATED_PWM_DEVICES.iter().take(18) {
            pwm_device_buf[pwm_n] = (d.gpio, d.base_register, d.config);
            pwm_n += 1;
        }
    }
    let pwm_bank = match &pwm_hw {
        Some(pwm) if GENERATED_HAS_PWM => {
            Some(PwmBank::from_null_pool(pwm, &mut pin_pool, &pwm_device_buf[..pwm_n]))
        }
        _ => None,
    };

    let mut i2c_buf = [I2cPwmBankDevice {
        bus: 0,
        device_addr: 0x40,
        channel: 0,
        base_register: 0,
        config: lucy_embedded_firmware_core::drivers::PwmServoConfig {
            min_pulse: 1000,
            max_pulse: 2000,
            min_angle: 0.0,
            max_angle: core::f32::consts::PI,
            default_angle: core::f32::consts::FRAC_PI_2,
        },
    }; MAX_I2C_PWM_DEVICES];
    let mut i2c_n = 0usize;
    if GENERATED_HAS_I2C_PWM {
        for d in GENERATED_I2C_PWM_DEVICES.iter().take(MAX_I2C_PWM_DEVICES) {
            i2c_buf[i2c_n] = I2cPwmBankDevice {
                bus: d.bus,
                device_addr: d.device_addr,
                channel: d.channel,
                base_register: d.base_register,
                config: d.config,
            };
            i2c_n += 1;
        }
    }
    let mut i2c_bank = if GENERATED_HAS_I2C_PWM {
        Some(I2cPwmBank::new(&i2c_buf[..i2c_n]))
    } else {
        None
    };

    let mut adc_buf = [AdcBankDevice {
        channel: 0,
        base_register: 0,
        config: lucy_embedded_firmware_core::drivers::PressureSensorConfig {
            min_value: 0,
            max_value: 4095,
        },
    }; MAX_ADC_DEVICES];
    let mut adc_n = 0usize;
    if GENERATED_HAS_ADC {
        for d in GENERATED_ADC_DEVICES.iter().take(MAX_ADC_DEVICES) {
            adc_buf[adc_n] = AdcBankDevice {
                channel: d.channel,
                base_register: d.base_register,
                config: d.config,
            };
            adc_n += 1;
        }
    }
    let mut adc_bank = if GENERATED_HAS_ADC {
        Some(AdcBank::new(&adc_buf[..adc_n]))
    } else {
        None
    };

    let usb_bus = UsbBusAllocator::new(rp2040_hal::usb::UsbBus::new(
        pac.USBCTRL_REGS,
        pac.USBCTRL_DPRAM,
        clocks.usb_clock,
        true,
        &mut pac.RESETS,
    ));
    let usb_serial = resolve_usb_serial(GENERATED_USB_SERIAL_ID);
    let (mut serial, mut picotool, mut usb_dev) =
        build_usb_device(&usb_bus, "Lucy RP2040", usb_serial);

    let slave = Slave {
        address: GENERATED_SLAVE_ADDRESS,
    };
    let rt = RegisterTable::default();
    if let Some(bank) = bus_bank.as_mut() {
        bank.seed_id_registers(&rt);
    }
    let mut modbus = ModbusCdcState::new(115_200);

    let _ = delay;
    let _ = pin_pool;

    // Generous timeout: UART bus TX can block briefly; pet every loop iteration.
    watchdog.start(2_000_000u32.micros());

    loop {
        watchdog.feed();
        let now = timer.get_counter().ticks();
        modbus.poll_usb(&mut usb_dev, &mut serial, &mut picotool, now);
        modbus.try_route_frame(&slave, &rt, &mut usb_dev, &mut serial, &mut picotool, now);

        if let (Some(bank), Some(pwm)) = (&pwm_bank, &pwm_hw) {
            bank.tick(pwm, &rt);
        }
        if let Some(bank) = bus_bank.as_mut() {
            bank.tick(&rt);
        }
        if let Some(bank) = i2c_bank.as_mut() {
            bank.tick(&rt);
        }
        if let Some(bank) = adc_bank.as_mut() {
            bank.tick(&rt);
        }
    }
}

type PinDynNull = rp2040_hal::gpio::Pin<DynPinId, FunctionNull, PullDown>;
