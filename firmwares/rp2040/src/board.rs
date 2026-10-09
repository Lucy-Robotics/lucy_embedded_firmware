use crate::channel::Rp2040UartChannel;
use crate::channel::Rp2040PwmChannel;
use crate::ws2812::QWs2812;
use core::option::Option;

use rp2040_hal::pio::PIOExt;
use rp2040_hal::{
    gpio,
    uart,
    pac,
    clocks,
    usb,
    pio,
    pwm,

    Sio,
    Timer,
    Clock,
    Watchdog,
};
use rp2040_hal::fugit::RateExtU32;

pub type Uart0Channel = Rp2040UartChannel<
    gpio::Pin<gpio::bank0::Gpio16, gpio::FunctionSioOutput, gpio::PullDown>,
    pac::UART0,
    (gpio::Pin<gpio::bank0::Gpio0, gpio::FunctionUart, gpio::PullDown>, gpio::Pin<gpio::bank0::Gpio1, gpio::FunctionUart, gpio::PullDown>)>;

pub type Servo3Channel = Rp2040PwmChannel<pwm::Channel<pwm::Slice<pwm::Pwm1, pwm::FreeRunning>, pwm::A>>;


pub struct Resources {
    // MANDATORY
    pub watchdog: Watchdog,
    pub usb_bus: Option<usb::UsbBus>,
    pub timer: Timer,
    pub delay: cortex_m::delay::Delay,
    pub ws: Option<QWs2812<pac::PIO0, pio::SM0, gpio::Pin<gpio::bank0::Gpio18, gpio::FunctionPio0, gpio::PullDown>, 6>>,

    // GENERATED
    pub uart0: Uart0Channel,
    pub servo3: Servo3Channel,
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
    let delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());

    let ws2812 = QWs2812::new(
        pins.gpio18.into_function(),
        &mut pio,
        sm0, clocks.peripheral_clock.freq(), timer.count_down());

    let usb_bus = usb::UsbBus::new(
        pac.USBCTRL_REGS,
        pac.USBCTRL_DPRAM,
        clocks.usb_clock,
        true,
        &mut pac.RESETS,
    );

    // Generated

    let tx = pins.gpio0.into_function::<gpio::FunctionUart>();
    let rx = pins.gpio1.into_function::<gpio::FunctionUart>();
    let dir = pins.gpio16.into_push_pull_output_in_state(gpio::PinState::Low);
    let uart0 = uart::UartPeripheral::new(
        pac.UART0,
        (tx, rx),
        &mut pac.RESETS,
    );
    let uart0 = uart0.enable(
        uart::UartConfig::new(1_000_000.Hz(), uart::DataBits::Eight, None, uart::StopBits::One),
        clocks.peripheral_clock.freq(),
    ).unwrap();

    let pwm_slices = pwm::Slices::new(pac.PWM, &mut pac.RESETS);
    let mut pwm = pwm_slices.pwm1;
    pwm.set_div_int(125);
    pwm.set_top(20000 - 1);
    pwm.channel_a.output_to(pins.gpio2.into_function::<gpio::FunctionPwm>());
    pwm.enable();
    let channel = pwm.channel_a;

    Resources {
        watchdog,
        usb_bus: Some(usb_bus),
        ws: Some(ws2812),
        timer,
        delay,

        uart0: Rp2040UartChannel {
            dir,
            uart: uart0,
            timer,
        },
        servo3: Rp2040PwmChannel {
            channel
        },
    }
}
