use lucy_embedded_firmware_feetech::feetech::FeetechBusConfig;
use lucy_embedded_firmware_feetech::feetech::FeetechBusDriver;
use lucy_embedded_firmware_feetech::feetech::FeetechServoConfig;

use crate::fake_bus::FakeBus;

const fn servo(id: u8) -> FeetechServoConfig {
    FeetechServoConfig {
        id,
        amplitude: core::f32::consts::TAU,
        min_pulse: 0,
        max_pulse: 4096,
        torque: 2.94,
        gear_ratio: 1.0,
        stall_current: 2.7,
        no_load_current: 0.18,
        current_lsb: 0.0065,
    }
}

static CONFIG: FeetechBusConfig<2> = FeetechBusConfig {
    servos: [servo(1), servo(2)],
};

pub(crate) fn driver() -> FeetechBusDriver<FakeBus, 2> {
    FeetechBusDriver::new(&CONFIG)
}

pub(crate) fn no_delay() -> impl FnMut() {
    || {}
}
