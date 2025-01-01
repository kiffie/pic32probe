//! System timer using the core timer
//!
// Copyright (C) 2022, 2023 Stephan <kiffie@mailbox.org>
// SPDX-License-Identifier: GPL-2.0-or-later

use crate::Error;
use crate::SystemTimerOps;

use fugit::MicrosDurationU64;
use std::marker::PhantomData;

pub type Instant = fugit::Instant<u64, 1, 1_000_000>;
pub type Duration = MicrosDurationU64;

pub struct SystemTimer {
    _private_dummy: PhantomData<()>,
}

impl SystemTimer {
    /// Create a new system timer instance.
    ///
    #[allow(clippy::new_without_default)]
    pub fn new() -> SystemTimer {
        SystemTimer {
            _private_dummy: PhantomData,
        }
    }
}

impl SystemTimerOps for SystemTimer {
    type Duration = Duration;
    type Instant = Instant;

    /// Get current time
    fn now(&self) -> Self::Instant {
        let micros = std::time::SystemTime::now()
            .duration_since(std::time::SystemTime::UNIX_EPOCH)
            .unwrap()
            .as_micros();
        Instant::from_ticks(micros as u64)
    }

    /// This does nothing in the std implementation.
    fn schedule_irq(&mut self, _when: Self::Instant) -> Result<(), Error> {
        Ok(())
    }

    fn wait_until(&mut self, when: Self::Instant) {
        if let Some(duration) = when.checked_duration_since(self.now()) {
            let micros = duration.to_micros();
            std::thread::sleep(std::time::Duration::from_micros(micros));
        }
    }
}

#[test]
fn test_std_impl() {
    use super::SystemTimer;

    let mut st = SystemTimer::new();
    let start_time = std::time::Instant::now();
    st.wait(Duration::secs(1));
    let end_time = std::time::Instant::now();
    let run_duration = end_time.duration_since(start_time).as_secs_f64();
    println!("run duration: {run_duration}");
    assert!(run_duration >= 1.0 && run_duration < 1.001);
}
