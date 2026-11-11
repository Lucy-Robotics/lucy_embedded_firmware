#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct CommandConfig {
    pub default: f32,
    pub limit_min: f32,
    pub limit_max: f32,
}

#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub enum TorqueStatus {
    #[default]
    Disabled,
    Enabled,
}

impl From<f32> for TorqueStatus {
    #[inline]
    fn from(value: f32) -> Self {
        if value >= 0.5 && !value.is_nan() {
            TorqueStatus::Enabled
        } else {
            TorqueStatus::Disabled
        }
    }
}

#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct Command {
    pub position: f32,
    pub velocity: f32,
    pub acceleration: f32,
    pub torque_enabled: TorqueStatus,
}

#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct State {
    pub position: f32,
    pub temperature: f32,
    pub effort: f32,
    pub current: f32,
    pub load: f32,
    pub voltage: f32,
}
