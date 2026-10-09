#![no_std]
#![no_main]

use portable_atomic::AtomicU64;

use lucy_embedded_firmware_core::data::{CommandBlock, StateBlock};

use rp2040_hal::fugit::MicrosDurationU64;
use rp2040_hal::fugit::ExtU32;

use usb_device::{class_prelude::*, prelude::*};
use usbd_serial::SerialPort;

mod picotool_reset;
use picotool_reset::PicoToolReset;

use smart_leds::{SmartLedsWrite, RGB8};

use lucy_embedded_firmware_communication::{encode_frame, framed_len, FrameDecoder};

use cortex_m_rt::entry;
use panic_halt as _;

mod ws2812;
mod channel;
mod board;
use board::init;
mod config;
use config::get_robot;
use config::robot_tick;

const STATE_BYTES: usize = size_of::<StateBlock<32>>();
const COMMAND_BYTES: usize = size_of::<CommandBlock<32>>();

fn struct_as_bytes<T>(value: &T) -> &[u8] {
    unsafe {
        core::slice::from_raw_parts(
            value as *const T as *const u8,
            size_of::<T>(),
        )
    }
}

fn struct_as_bytes_mut<T>(value: &mut T) -> &mut [u8] {
    unsafe {
        core::slice::from_raw_parts_mut(
            value as *mut T as *mut u8,
            size_of::<T>(),
        )
    }
}

#[unsafe(link_section = ".boot2")]
#[unsafe(no_mangle)]
#[used]
pub static BOOT2: [u8; 256] = rp2040_boot2::BOOT_LOADER_GENERIC_03H;

const TICK_PERIOD: MicrosDurationU64 = MicrosDurationU64::micros(1000);

#[entry]
fn main() -> ! {
    let mut resources = init();
    let mut robot = get_robot();

    let mut commands = CommandBlock::<32> {
        command_seq: AtomicU64::new(0),
        hw_commands: [0.0; 32],
        hw_velocities: [0.0; 32],
        hw_accelerations: [0.0; 32],
        hw_torque_enabled: [0.0; 32],
    };

    let mut states = StateBlock::<32> {
        state_seq: AtomicU64::new(0),
        hw_positions: [0.0; 32],
    };

    let mut decoder = FrameDecoder::<COMMAND_BYTES>::new();

    let mut ws2812 = resources.ws.take().unwrap();

    let usb_bus = UsbBusAllocator::new(resources.usb_bus.take().unwrap());
    let mut serial = usbd_serial::SerialPort::new(&usb_bus);
    let mut reset = PicoToolReset::new(&usb_bus);
    let mut usb_dev = UsbDeviceBuilder::new(&usb_bus, UsbVidPid(0x2e8a, 0x0003))
        .strings(&[StringDescriptors::default()
            .manufacturer("Pimoroni")
            .product("Servo2040 Serial")
            .serial_number("E0C9125B0D9B")])
        .unwrap()
        .device_class(usbd_serial::USB_CLASS_CDC)
        .composite_with_iads()
        .build();

    ws2812.set_color(0, RGB8::new(0, 50, 0));
    resources.watchdog.start(1_000.millis());

    loop {
        resources.watchdog.feed();
        let tick_start = resources.timer.get_counter().ticks();
        usb_dev.poll(&mut [&mut serial, &mut reset]);

        let mut tmp_buf = [0u8; 64];
        while let Ok(count) = serial.read(&mut tmp_buf) {
            if count == 0 {
                break;
            }

            for &byte in &tmp_buf[..count] {
                if let Some(payload) = decoder.push_byte(byte) {
                    struct_as_bytes_mut(&mut commands).copy_from_slice(payload);
                }
            }
        }
        robot.sync_write(&commands);
        robot.sync_read(&mut states);

        let mut frame = [0u8; framed_len(STATE_BYTES)];
        encode_frame(struct_as_bytes(&states), &mut frame);

        let _ = serial.write(&frame);
        robot_tick(
            states.state_seq.load(portable_atomic::Ordering::Relaxed),
            &mut robot,
            &mut resources,
        );

        let elapsed_ticks = resources.timer.get_counter().ticks() - tick_start;
        if elapsed_ticks < TICK_PERIOD.ticks() {
            let remaining_ticks = TICK_PERIOD.ticks() - elapsed_ticks;
        }
    }
}
