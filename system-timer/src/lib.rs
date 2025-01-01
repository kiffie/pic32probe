//! Target independent system timer
// Copyright (C) 2022 Stephan <kiffie@mailbox.org>
// SPDX-License-Identifier: GPL-2.0-or-later

#![cfg_attr(not(feature = "std"), no_std)]

use cfg_if::cfg_if;
use core::ops::Add;
pub use fugit;

#[cfg(not(feature = "device-selected"))]
compile_error!("This crate requires one device feature to be enabled");

#[cfg(feature = "pic32")]
mod impl_pic32;

#[cfg(feature = "pic32")]
pub use impl_pic32::{Duration, Instant, SystemTimer};

#[cfg(feature = "rp2040")]
mod impl_rp2040;

#[cfg(feature = "rp2040")]
pub use impl_rp2040::{Duration, Instant, SystemTimer};

#[cfg(feature = "std")]
mod impl_std;

#[cfg(feature = "std")]
pub use impl_std::{Duration, Instant, SystemTimer};

#[derive(Debug)]
pub enum Error {
    InstantTooEarly,
    InstantTooLate,
    CannotReschedule,
    InternalError,
}

pub trait SystemTimerOps {
    type Instant;
    type Duration;

    /// Get current time
    fn now(&self) -> Self::Instant;

    /// Schedule the next IRQ to be triggered at `when` at the latest.
    ///
    /// Fails if `when` is too early, i.e. too close to the current time or even
    /// behind the current time. Depending on the implementation, this function
    /// may have no effect, e.g., when an IRQ has already been scheduled at an
    /// earlier Instant.
    fn schedule_irq(&mut self, when: Self::Instant) -> Result<(), Error>;

    /// Delay until the clock reaches an Instant
    fn delay_until(&self, when: Self::Instant)
    where
        Self::Instant: PartialOrd,
    {
        while self.now() < when {}
    }

    /// Delay execution by Duration d
    fn delay(&self, d: Self::Duration)
    where
        Self::Instant: PartialOrd,
        Self::Instant: Add<Self::Duration, Output = Self::Instant>,
    {
        let when: Self::Instant = self.now() + d;
        self.delay_until(when);
    }

    /// Wait until the clock reaches an Instant by scheduling IRQs and putting
    /// the processor in a wait mode.
    fn wait_until(&mut self, when: Self::Instant);

    /// Wait for a Duration d by scheduling IRQs and putting the processor in a
    /// wait mode.
    fn wait(&mut self, d: Self::Duration)
    where
        Self::Instant: Add<Self::Duration, Output = Self::Instant>,
    {
        let when: Self::Instant = self.now() + d;
        self.wait_until(when);
    }
}

cfg_if! {
    if #[cfg(feature = "embedded_hal_02")] {

        use embedded_hal_02::{
            blocking::delay::{DelayMs, DelayUs},
            timer::{CountDown, Periodic},
        };
        use fugit::{TimerDurationU64, TimerInstantU64};
        use void::Void;

        macro_rules! impl_delay_ms {
            ($t:ty) => {
                impl<const FREQ_HZ: u32> DelayMs<$t> for SystemTimer
                where
                    SystemTimer: SystemTimerOps<
                        Instant = TimerInstantU64<FREQ_HZ>,
                        Duration = TimerDurationU64<FREQ_HZ>,
                    >,
                {
                    fn delay_ms(&mut self, ms: $t) {
                        self.delay(<SystemTimer as SystemTimerOps>::Duration::millis(ms as u64));
                    }
                }
            };
        }

        impl_delay_ms!(i32);
        impl_delay_ms!(u32);
        impl_delay_ms!(u16);
        impl_delay_ms!(u8);

        macro_rules! impl_delay_us {
            ($t:ty) => {
                impl<const FREQ_HZ: u32> DelayUs<$t> for SystemTimer
                where
                    SystemTimer: SystemTimerOps<
                        Instant = TimerInstantU64<FREQ_HZ>,
                        Duration = TimerDurationU64<FREQ_HZ>,
                    >,
                {
                    fn delay_us(&mut self, us: $t) {
                        self.delay(<SystemTimer as SystemTimerOps>::Duration::micros(us as u64));
                    }
                }
            };
        }

        impl_delay_us!(i32);
        impl_delay_us!(u32);
        impl_delay_us!(u16);
        impl_delay_us!(u8);

        pub struct Timer {
            elapse: Option<Instant>,
        }

        impl Timer {
            #[allow(clippy::new_without_default)]
            pub fn new() -> Timer {
                Timer { elapse: None }
            }
        }

        impl CountDown for Timer {
            type Time = Duration;

            fn start<T>(&mut self, count: T)
            where
                T: Into<Self::Time>,
            {
                self.elapse = Some(SystemTimer::new().now() + count.into());
            }

            fn wait(&mut self) -> nb::Result<(), Void> {
                let elapse = self.elapse.unwrap();
                if SystemTimer::new().now() >= elapse {
                    Ok(())
                } else {
                    Err(nb::Error::WouldBlock)
                }
            }
        }

        impl Periodic for Timer {}
    }
}
