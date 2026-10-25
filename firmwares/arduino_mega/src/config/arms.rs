//! `mega_arms` (inmoov_i2_urdf `config/hardware/active.yaml`): two PCA9685s chained on I2C.

use lucy_embedded_firmware_core::joint::CommandConfig;

use super::full_range;

/// First chip: left arm. Second chip (A0 bridged): right arm.
pub const PCA_ADDRESSES: [u8; 2] = [0x40, 0x41];

/// PCA output per HI slot (`virtual_pin`): left arm on outputs 0..8, right arm on 16..24.
pub const OUTPUTS: [u8; 18] = [
    0, 1, 2, 3, 4, 5, 6, 7, 8,
    16, 17, 18, 19, 20, 21, 22, 23, 24,
];

pub const JOINTS_CONFIG: [CommandConfig; 18] = [
    full_range(1.5708),  // left shoulder_z
    full_range(0.0),     // left shoulder_x
    full_range(0.0),     // left elbow_x
    full_range(2.61799), // left wrist_z
    full_range(0.0),     // left thumb
    full_range(0.0),     // left index
    full_range(0.0),     // left majeure
    full_range(0.0),     // left ring
    full_range(0.0),     // left pinky
    full_range(1.5708),  // right shoulder_z
    full_range(0.0),     // right shoulder_x
    full_range(0.0),     // right elbow_x
    full_range(2.61799), // right wrist_z
    full_range(0.0),     // right thumb
    full_range(0.0),     // right index
    full_range(0.0),     // right majeure
    full_range(0.0),     // right ring
    full_range(0.0),     // right pinky
];
