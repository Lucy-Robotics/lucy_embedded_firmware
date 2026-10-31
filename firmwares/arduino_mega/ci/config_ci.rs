pub const PCA_ADDRESSES: [u8; 1] = [0x40];

pub const MEGA_PINS: [u8; 1] = [8];

macro_rules! mega_servo_pins {
    ($pins:ident) => {
        [$pins.d8.into_output().downgrade()]
    };
}

pub const JOINTS_CONFIG: [CommandConfig; 2] = [
    CommandConfig { default: 0.0, limit_min: 0.0, limit_max: 3.14159 },
    CommandConfig { default: 0.0, limit_min: 0.0, limit_max: 3.14159 },
];

pub const DRIVEN: [(usize, u8); 2] = [(0, 0), (1, 16)];
