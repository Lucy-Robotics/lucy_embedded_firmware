#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct JointConfig {
    pub default: f64,
    pub limit_min: f64,
    pub limit_max: f64,
}

#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub enum TorqueStatus {
    #[default]
    Disabled,
    Enabled,
}

impl From<f64> for TorqueStatus {
    #[inline]
    fn from(value: f64) -> Self {
        if value >= 0.5 && !value.is_nan() {
            TorqueStatus::Enabled
        } else {
            TorqueStatus::Disabled
        }
    }
}

#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct JointCommand {
    pub position: f64,
    pub velocity: f64,
    pub acceleration: f64,
    pub torque_enabled: TorqueStatus,
}

#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct JointState {
    pub position: f64,
    pub temperature: f64,
    pub torque: f64,
}
