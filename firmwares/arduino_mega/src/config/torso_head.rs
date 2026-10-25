//! `mega_torso_head` (inmoov_i2_urdf `config/hardware/active.yaml`): one PCA9685.

use lucy_embedded_firmware_core::joint::CommandConfig;

use super::full_range;

use core::f64::consts::PI;

const DEG: f64 = PI / 180.0;

/// The Nervo board's PCA9685.
pub const PCA_ADDRESSES: [u8; 1] = [0x41];

/// PCA output per HI slot (`virtual_pin`). The jaw stays on channel 6, where it was wired first.
pub const OUTPUTS: [u8; 12] = [0, 1, 2, 3, 4, 5, 7, 6, 8, 9, 10, 11];

pub const JOINTS_CONFIG: [CommandConfig; 12] = [
    full_range(0.0),     // left_shoulder_y
    full_range(0.0),     // right_shoulder_y
    full_range(2.35619), // torso_z
    full_range(1.5708),  // torso_y
    full_range(1.5708),  // head_rothead
    full_range(1.5708),  // neck roll
    full_range(1.5708),  // head_neck
    // Jaw: rest at 25 deg, mechanical range 25..120 deg.
    CommandConfig { default: 25.0 * DEG, limit_min: 25.0 * DEG, limit_max: 120.0 * DEG },
    full_range(1.5708),  // right eye vertical
    full_range(1.5708),  // right eye horizontal
    full_range(1.5708),  // left eye vertical
    full_range(1.5708),  // left eye horizontal
];
