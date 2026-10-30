#![no_std]
#![no_main]

use lucy_embedded_firmware_core::joint::TorqueStatus;
use lucy_embedded_firmware_core::robot::Robot;

use rp2040_hal::fugit::MicrosDurationU64;
use rp2040_hal::fugit::ExtU32;

use usb_device::{class_prelude::*, prelude::*};

mod picotool_reset;
use picotool_reset::PicoToolReset;

use smart_leds::RGB8;

use lucy_embedded_firmware_communication::{
    encode, frame_len, Body, Frame, FrameDecoder, JointCommands, JointStates,
};

use cortex_m_rt::entry;
use panic_halt as _;

mod ws2812;
mod channel;
mod board;
use board::init;
mod config;
use config::{get_robot, JOINTS};
use config::robot_tick;

type Serial<'a, B> = usbd_serial::SerialPort<'a, B, [u8; 512], [u8; 512]>;

const FRAME_LEN: usize = frame_len(JOINTS);

struct FrameWriter {
    buf: [u8; FRAME_LEN],
    len: usize,
    sent: usize,
}

impl FrameWriter {
    const fn new() -> Self {
        Self {
            buf: [0u8; FRAME_LEN],
            len: 0,
            sent: 0,
        }
    }

    fn is_idle(&self) -> bool {
        self.sent >= self.len
    }

    fn start(&mut self, frame: &Frame<JOINTS>) {
        if let Ok(len) = encode(frame, &mut self.buf) {
            self.len = len;
            self.sent = 0;
        }
    }

    fn flush<B: usb_device::bus::UsbBus>(&mut self, serial: &mut Serial<'_, B>) {
        if self.is_idle() {
            return;
        }
        if let Ok(n) = serial.write(&self.buf[self.sent..self.len]) {
            self.sent += n;
        }
    }
}

fn read_commands<B: usb_device::bus::UsbBus>(
    serial: &mut Serial<'_, B>,
    decoder: &mut FrameDecoder<'_, JOINTS>,
    commands: &mut JointCommands<JOINTS>,
) -> Option<u32> {
    let mut last_seq = None;
    let mut buf = [0u8; 64];
    loop {
        match serial.read(&mut buf) {
            Ok(0) | Err(_) => break,
            Ok(count) => {
                for &byte in &buf[..count] {
                    if let Some(Ok(Frame { seq, body: Body::Command(c), .. })) = decoder.push_byte(byte) {
                        *commands = c;
                        last_seq = Some(seq);
                    }
                }
            }
        }
    }
    last_seq
}

fn apply_commands<const N: usize>(robot: &mut Robot<N>, commands: &JointCommands<N>) {
    for i in 0..N {
        let position = robot.clamp_position(i, commands.positions[i]);
        let joint = &mut robot.joints_command[i];
        joint.position = position;
        joint.velocity = commands.velocities[i];
        joint.acceleration = commands.accelerations[i];
        joint.torque_enabled = if commands.torque_enabled[i] {
            TorqueStatus::Enabled
        } else {
            TorqueStatus::Disabled
        };
    }
}

fn disable_torque<const N: usize>(robot: &mut Robot<N>) {
    for joint in robot.joints_command.iter_mut() {
        joint.torque_enabled = TorqueStatus::Disabled;
    }
}

fn collect_states<const N: usize>(robot: &Robot<N>) -> JointStates<N> {
    JointStates { positions: robot.joints_state.map(|joint| joint.position) }
}

#[unsafe(link_section = ".boot2")]
#[unsafe(no_mangle)]
#[used]
pub static BOOT2: [u8; 256] = rp2040_boot2::BOOT_LOADER_GENERIC_03H;

const TICK_PERIOD: MicrosDurationU64 = MicrosDurationU64::micros(1000);
const COMMAND_TIMEOUT: MicrosDurationU64 = MicrosDurationU64::millis(200);

#[entry]
fn main() -> ! {
    let mut resources = init();
    let mut robot = get_robot();

    let mut commands = JointCommands::<JOINTS>::default();

    let mut rx_buf = [0u8; FRAME_LEN];
    let mut decoder = FrameDecoder::<JOINTS>::new(&mut rx_buf);
    let mut frame_writer = FrameWriter::new();
    let mut pending_reply: Option<u32> = None;
    let mut last_command_tick: Option<u64> = None;
    let mut tick_count: u64 = 0;
    let mut last_error_tick: u64 = 0;

    let mut ws2812 = resources.ws.take().unwrap();

    let usb_bus = UsbBusAllocator::new(resources.usb_bus.take().unwrap());

    let mut serial = usbd_serial::SerialPort::new_with_store(&usb_bus, [0u8; 512], [0u8; 512]);
    let mut reset = PicoToolReset::new(&usb_bus);
    // Serial comes from DEVICE_SERIAL_NUMBER (env or config.yaml `serial_id` via
    // build.rs). Omit when unset - never bake a hardcoded board string.
    let mut descriptors = StringDescriptors::default()
        .manufacturer("Pimoroni")
        .product("Servo2040 Serial");
    if let Some(serial_number) = option_env!("DEVICE_SERIAL_NUMBER") {
        if !serial_number.is_empty() {
            descriptors = descriptors.serial_number(serial_number);
        }
    }
    let mut usb_dev = UsbDeviceBuilder::new(&usb_bus, UsbVidPid(0x2e8a, 0x0003))
        .strings(&[descriptors])
        .unwrap()
        .device_class(usbd_serial::USB_CLASS_CDC)
        .composite_with_iads()
        .build();

    ws2812.set_color(0, RGB8::new(0, 50, 0));
    resources.watchdog.start(1_000.millis());

    let mut led_on = true;
    let mut led_tick = resources.timer.get_counter().ticks();

    loop {
        resources.watchdog.feed();
        let tick_start = resources.timer.get_counter().ticks();
        usb_dev.poll(&mut [&mut serial, &mut reset]);

        if let Some(seq) = read_commands(&mut serial, &mut decoder, &mut commands) {
            pending_reply = Some(seq);
            last_command_tick = Some(tick_start);
            apply_commands(&mut robot, &commands);
        }

        let timed_out = match last_command_tick {
            Some(t) => tick_start - t > COMMAND_TIMEOUT.ticks(),
            None => true,
        };
        if timed_out {
            disable_torque(&mut robot);
        }

        if !robot_tick(tick_count, &mut robot, &mut resources) {
            last_error_tick = tick_start;
        }
        tick_count = tick_count.wrapping_add(1);

        if frame_writer.is_idle() {
            if let Some(seq) = pending_reply.take() {
                frame_writer.start(&Frame::new(seq, Body::State(collect_states(&robot))));
            }
        }
        frame_writer.flush(&mut serial);

        let elapsed_led_ticks = resources.timer.get_counter().ticks() - led_tick;
        if elapsed_led_ticks > 500_000 {
            led_tick = resources.timer.get_counter().ticks();
            let level = 50 * (led_on as u8);
            let bus_error = last_error_tick != 0 && led_tick - last_error_tick < 1_000_000;
            let color = if bus_error { RGB8::new(level, 0, 0) } else { RGB8::new(0, level, 0) };
            ws2812.set_color(0, color);
            led_on = !led_on;
        }

        let elapsed_ticks = resources.timer.get_counter().ticks() - tick_start;
        if elapsed_ticks < TICK_PERIOD.ticks() {
            let remaining_ticks = TICK_PERIOD.ticks() - elapsed_ticks;
            resources.delay.delay_us(remaining_ticks as u32);
        }
    }
}
