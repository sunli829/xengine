use bitflags::_core::time::Duration;
use std::time::Instant;

pub struct Timer {
    start: Instant,
}

impl Timer {
    pub fn new() -> Timer {
        Timer {
            start: Instant::now(),
        }
    }

    pub fn reset(&mut self) {
        self.start = Instant::now();
    }

    pub fn get_duration(&self) -> Duration {
        Instant::now() - self.start
    }
}
