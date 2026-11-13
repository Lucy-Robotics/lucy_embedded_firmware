use lucy_embedded_firmware_core::guard::Guard;
use lucy_embedded_firmware_core::joint::{Command, CommandConfig, State, TorqueStatus};

pub struct PositionLimitGuard<const M: usize> {
    limits: &'static [CommandConfig; M],
}

impl<const M: usize> PositionLimitGuard<M> {
    pub const fn new(limits: &'static [CommandConfig; M]) -> Self {
        Self { limits }
    }
}

impl<const M: usize> Guard for PositionLimitGuard<M> {
    fn apply(&mut self, _state: &[State], command: &mut [Command]) {
        for (limit, command) in self.limits.iter().zip(command.iter_mut()) {
            if !command.position.is_finite() {
                command.torque_enabled = TorqueStatus::Disabled;
                continue;
            }
            command.position = command.position.clamp(limit.limit_min, limit.limit_max);
        }
    }
}
