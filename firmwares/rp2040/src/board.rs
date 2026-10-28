use crate::i2c_bank::I2cBank;
use crate::ws2812::QWs2812;
use core::option::Option;

use rp2040_hal::pio::PIOExt;
use rp2040_hal::{
    gpio,
    pac,
    clocks,
    usb,
    pio,
    pwm,
    I2C,

    Sio,
    Timer,
    Clock,
    Watchdog,
};
use rp2040_hal::fugit::RateExtU32;

use crate::headers::KeptPins;

pub type ServoI2c = I2C<
    pac::I2C0,
    (
        gpio::Pin<gpio::bank0::Gpio20, gpio::FunctionI2C, gpio::PullUp>,
        gpio::Pin<gpio::bank0::Gpio21, gpio::FunctionI2C, gpio::PullUp>,
    ),
>;

pub struct Resources {
    pub watchdog: Watchdog,
    pub usb_bus: Option<usb::UsbBus>,
    pub timer: Timer,
    pub delay: cortex_m::delay::Delay,
    pub ws: Option<QWs2812<pac::PIO0, pio::SM0, gpio::Pin<gpio::bank0::Gpio18, gpio::FunctionPio0, gpio::PullDown>, 6>>,
    pub slices: pwm::Slices,
    pub i2c: Option<I2cBank<ServoI2c>>,
}

pub fn init() -> Resources {
    let core = cortex_m::Peripherals::take().unwrap();
    let mut pac = pac::Peripherals::take().unwrap();

    let mut watchdog = Watchdog::new(pac.WATCHDOG);
    let sio = Sio::new(pac.SIO);
    let pins = gpio::Pins::new(pac.IO_BANK0, pac.PADS_BANK0, sio.gpio_bank0, &mut pac.RESETS);
    let (mut pio, sm0, _, _, _) = pac.PIO0.split(&mut pac.RESETS);

    let clocks = clocks::init_clocks_and_plls(
        12_000_000, pac.XOSC, pac.CLOCKS, pac.PLL_SYS, pac.PLL_USB,
        &mut pac.RESETS, &mut watchdog
    ).ok().unwrap();
    let timer = Timer::new(pac.TIMER, &mut pac.RESETS, &clocks);
    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());

    let mut slices = pwm::Slices::new(pac.PWM, &mut pac.RESETS);
    let kept: KeptPins = crate::headers::attach(pins, &mut slices);
    let ws2812 = QWs2812::new(
        kept.gpio18.into_function(),
        &mut pio,
        sm0, clocks.peripheral_clock.freq(), timer.count_down());
    let i2c = if crate::i2c_bank::rows().is_empty() {
        drop(kept.gpio20);
        drop(kept.gpio21);
        None
    } else {
        let sda = kept.gpio20.into_pull_up_input().into_function();
        let scl = kept.gpio21.into_pull_up_input().into_function();
        let bus = I2C::i2c0(
            pac.I2C0,
            sda,
            scl,
            400.kHz(),
            &mut pac.RESETS,
            clocks.system_clock.freq(),
        );
        Some(I2cBank::new(bus, |us| delay.delay_us(us)))
    };

    let usb_bus = usb::UsbBus::new(
        pac.USBCTRL_REGS,
        pac.USBCTRL_DPRAM,
        clocks.usb_clock,
        true,
        &mut pac.RESETS,
    );

    Resources {
        watchdog,
        usb_bus: Some(usb_bus),
        ws: Some(ws2812),
        timer,
        delay,
        slices,
        i2c,
    }
}
