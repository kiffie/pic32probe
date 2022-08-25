//! System timer using the core timer
//!
// Copyright (C) 2022 Stephan <kiffie@mailbox.org>
// SPDX-License-Identifier: GPL-2.0-or-later

pub use core::time::Duration;

use core::cell::RefCell;
use core::ops::{Add, AddAssign};

use pic32_hal::coretimer::Timer;
use pic32_hal::int::{Int, InterruptSource, IPL1};
use pic32_hal::pac::{INT, interrupt};
use pic32_hal::time::Hertz;

use mips_mcu::interrupt::free as irq_free;
use mips_mcu::interrupt::Mutex;

use log::trace;
use crate::Error;

const MIN_IRQ_SCHEDULE_TICKS: u64 = 1000;

pub struct SystemTimer {
    timer: Timer,
    ticks_hi: u32,
    ticks_lo_last: u32,
    nanos_per_tick: u32,
}

static SYSTEM_TIMER: Mutex<RefCell<Option<SystemTimer>>> = Mutex::new(RefCell::new(None));

#[derive(Clone, Copy, PartialEq, PartialOrd, Debug)]
pub struct Instant {
    ticks: u64,
}

impl SystemTimer {
    /// Initialize the system timer. This function must be called before other
    /// functions can be used. Otherwise, the other functions will panic.
    pub fn init(sysclock: Hertz, int: &Int, timer: Timer) {
        // core timer runs with the half CPU clock frequency
        let timer_clock = sysclock.0 / 2;
        let nanos_per_tick = 1_000_000_000 / timer_clock;
        irq_free(|cs| {
            let mut st = SYSTEM_TIMER.borrow(*cs).borrow_mut();
            if st.is_some() {
                panic!("System timer already initialized.");
            }
            let ticks_hi = 0;
            let ticks_lo_last = 0;
            *st = Some(SystemTimer {
                timer,
                ticks_hi,
                ticks_lo_last,
                nanos_per_tick,
            });
        });
        int.set_ipl(interrupt::Interrupt::CORE_TIMER, IPL1);
        int.ei(InterruptSource::CORE_TIMER);
        // trigger IRQ to finalize the initialization
        int.set_if(InterruptSource::CORE_TIMER);
    }

    /// Return Timer instance resetting the System Timer to the uninitialized
    /// state. Panics if not initialized.
    pub fn free() -> Timer {
        unsafe {
            (*INT::ptr()).iec0clr.write(|w| w.ctie().bit(true));
        }
        irq_free(|cs| SYSTEM_TIMER.borrow(*cs).borrow_mut().take().unwrap().timer)
    }

    fn nanos_per_tick() -> u32 {
        irq_free(|cs| {
            let stb = SYSTEM_TIMER.borrow(*cs).borrow();
            let st = stb.as_ref().unwrap();
            st.nanos_per_tick
        })
    }

    fn ticks_to_next_irq() -> u32 {
        irq_free(|cs| {
            let stb = SYSTEM_TIMER.borrow(*cs).borrow();
            let st = stb.as_ref().unwrap();
            st.timer.read_compare().wrapping_sub(st.timer.read_count())
        })
    }

    /// Schedule the next IRQ to be triggered at `when` if `when` is earlier
    /// than the currently scheduled IRQ. Fails if `when` is too early, i.e.
    /// too close to the current time or even behind the current time.
    pub fn schedule_irq(when: Instant) -> Result<(), Error> {
        let now = Instant::now().ticks;
        if now > when.ticks {
            return Err(Error::InstantTooEarly);
        }
        let delta = when.ticks - now;
        if when.ticks - now < MIN_IRQ_SCHEDULE_TICKS {
            return Err(Error::InstantTooEarly);
        }
        trace!(
            "now = {}, delta = {}, ticks_to_next_irq() = {}",
            now,
            delta,
            Self::ticks_to_next_irq()
        );
        if delta < Self::ticks_to_next_irq() as u64 {
            irq_free(|cs| {
                let stb = SYSTEM_TIMER.borrow(*cs).borrow();
                let st = stb.as_ref().unwrap();
                st.timer.write_compare(when.ticks as u32);
            });
        }
        Ok(())
    }
}

impl Instant {
    /// Get current time
    pub fn now() -> Instant {
        let (hi, lo) = irq_free(|cs| {
            let stb = SYSTEM_TIMER.borrow(*cs).borrow();
            let st = stb.as_ref().unwrap();
            (st.ticks_hi, st.timer.read_count())
            //(st.as_ref().unwrap().ticks_hi, st.as_ref().unwrap().timer.read_count())
        });
        Instant {
            ticks: (hi as u64) << 32 | lo as u64,
        }
    }
}

impl Add<Duration> for Instant {
    type Output = Self;

    fn add(self, rhs: Duration) -> Self {
        let nanos_per_tick = SystemTimer::nanos_per_tick();
        let duration_ticks = (rhs.as_nanos() / nanos_per_tick as u128) as u64;
        Instant {
            ticks: self.ticks + duration_ticks,
        }
    }
}

impl AddAssign<Duration> for Instant {
    fn add_assign(&mut self, rhs: Duration) {
        let nanos_per_tick = SystemTimer::nanos_per_tick();
        let duration_ticks = (rhs.as_nanos() / nanos_per_tick as u128) as u64;
        self.ticks += duration_ticks;
    }
}

// ISR: CORE_TIMER
#[interrupt]
fn CORE_TIMER() {
    irq_free(|cs| {
        let mut stb = SYSTEM_TIMER.borrow(*cs).borrow_mut();
        let mut st = stb.as_mut().unwrap();

        // update hi word of system timer
        let low = st.timer.read_count();
        if low < st.ticks_lo_last {
            // timer overflow
            st.ticks_hi += 1; // cannot overflow because increases too slowly
        }
        st.ticks_lo_last = low;

        // set compare register to schedule next IRQ
        st.timer.write_compare(low.wrapping_add(u32::MAX / 2));
    });

    unsafe {
        (*INT::ptr()).ifs0clr.write(|w| w.ctif().bit(true));
    }
}
