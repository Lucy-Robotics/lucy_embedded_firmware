use rp2040_hal::{
    gpio,
    pac,
    pio,
    timer,
};
use rp2040_hal::fugit;
use smart_leds::{RGB8, SmartLedsWrite};

pub struct QWs2812<P, SM, I, const N: usize>
where
    I: gpio::AnyPin<Function = P::PinFunction>,
    P: pio::PIOExt,
    SM: pio::StateMachineIndex,
{
    pub ws: ws2812_pio::Ws2812<P, SM, timer::CountDown, I>,
    current_colors: [RGB8; N],
}

impl<P, SM, I, const N: usize> QWs2812<P, SM, I, N>
where
    I: gpio::AnyPin<Function = P::PinFunction>,
    P: pio::PIOExt,
    SM: pio::StateMachineIndex,
{
    pub fn new(pin: I, pio: &mut pio::PIO<P>, sm: pio::UninitStateMachine<(P, SM)>, freq: fugit::HertzU32, timer: timer::CountDown) -> Self {
        let ws = ws2812_pio::Ws2812::new(pin, pio, sm, freq, timer);
        QWs2812 { ws, current_colors: [RGB8::default(); N] }
    }

    pub fn set_color(&mut self, index: usize, color: RGB8) {
        self.current_colors[index] = color;
        self.ws.write(self.current_colors.iter().cloned()).unwrap();
    }
}
