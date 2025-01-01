//! System timer using the core timer
//!
// Copyright (C) 2022, 2023 Stephan <kiffie@mailbox.org>
// SPDX-License-Identifier: GPL-2.0-or-later

use crate::Error;
use crate::SystemTimerOps;

use konst::{primitive::parse_u32, unwrap_ctx};

use core::cell::RefCell;
use core::marker::PhantomData;
use critical_section::Mutex;
use fugit::{TimerDurationU64, TimerInstantU64};
use mips_mcu::interrupt::wait;
use pic32_hal::coretimer::Timer;
use pic32_hal::int::{Int, InterruptSource, IPL1};
use pic32_hal::pac::{interrupt, INT};

const TIMER_FREQ: u32 = unwrap_ctx!(parse_u32(env!("SYS_CLOCK"))) / 2;
const MIN_IRQ_SCHEDULE_TICKS: u64 = 1000;

pub type Instant = TimerInstantU64<TIMER_FREQ>;
pub type Duration = TimerDurationU64<TIMER_FREQ>;

pub struct SystemTimer {
    _private_dummy: PhantomData<()>,
}

struct SystemTimerInternal {
    timer: Timer,
    ticks_hi: u32,
    ticks_lo_last: u32,
}

static SYSTEM_TIMER: Mutex<RefCell<Option<SystemTimerInternal>>> = Mutex::new(RefCell::new(None));

impl SystemTimer {
    /// Initialize the system timer
    ///
    /// This function must be called before other functions can be used.
    /// Otherwise, the other functions will panic.
    pub fn init(int: &Int, timer: Timer) {
        // core timer runs with the half CPU clock frequency
        critical_section::with(|cs| {
            let mut st = SYSTEM_TIMER.borrow_ref_mut(cs);
            if st.is_some() {
                panic!("System timer already initialized.");
            }
            *st = Some(SystemTimerInternal {
                timer,
                ticks_hi: 0,
                ticks_lo_last: 0,
            });
        });
        int.set_ipl(interrupt::Interrupt::CORE_TIMER, IPL1);
        int.ei(InterruptSource::CORE_TIMER);
        // trigger IRQ to finalize the initialization
        int.set_if(InterruptSource::CORE_TIMER);
    }

    /// Create a new system timer instance.
    ///
    /// Panics if not initialized.
    pub fn new() -> SystemTimer {
        if critical_section::with(|cs| SYSTEM_TIMER.borrow_ref(cs).is_none()) {
            panic!("Call to SystemTimer::new() without prior initialization");
        }
        SystemTimer {
            _private_dummy: PhantomData,
        }
    }

    /// Return Timer instance resetting the System Timer to the uninitialized
    /// state.
    ///
    /// Panics if not initialized.
    pub fn free() -> Timer {
        unsafe {
            (*INT::ptr()).iec0clr.write(|w| w.ctie().bit(true));
        }
        critical_section::with(|cs| SYSTEM_TIMER.borrow_ref_mut(cs).take().unwrap().timer)
    }
}

impl SystemTimerInternal {
    /// Get the counter and Check if it has wrapped and the update high word of
    /// the timer if needed
    ///
    /// Needs to be called sufficiently often so that counter wraps are not
    /// missed.
    #[inline(always)]
    fn get_low_and_update_high(&mut self) -> u32 {
        // update hi word of system timer
        let low = self.timer.read_count();
        if low < self.ticks_lo_last {
            self.ticks_hi += 1; // ticks_hi cannot overflow because increases too slowly
        }
        self.ticks_lo_last = low;
        low
    }

    fn ticks_to_next_irq(&self) -> u32 {
        self.timer
            .read_compare()
            .wrapping_sub(self.timer.read_count())
    }
}

impl SystemTimerOps for SystemTimer {
    type Duration = Duration;
    type Instant = Instant;

    /// Get current time
    fn now(&self) -> Self::Instant {
        let (hi, lo) = critical_section::with(|cs| {
            let mut stb = SYSTEM_TIMER.borrow_ref_mut(cs);
            let st = stb.as_mut().unwrap();
            (st.ticks_hi, st.get_low_and_update_high())
        });
        Instant::from_ticks((hi as u64) << 32 | lo as u64)
    }

    /// Schedule the next IRQ to be triggered at `when` if `when` is earlier
    /// than the currently scheduled IRQ. Fails if `when` is too early, i.e.
    /// too close to the current time or even behind the current time.
    fn schedule_irq(&mut self, when: Self::Instant) -> Result<(), Error> {
        let now = self.now().ticks();
        if now > when.ticks() {
            return Err(Error::InstantTooEarly);
        }
        let delta = when.ticks() - now;
        if delta < MIN_IRQ_SCHEDULE_TICKS {
            return Err(Error::InstantTooEarly);
        }
        critical_section::with(|cs| {
            let stb = SYSTEM_TIMER.borrow_ref(cs);
            let st = stb.as_ref().unwrap();
            let ticks_to_irq = st.ticks_to_next_irq() as u64;
            if delta < ticks_to_irq {
                if ticks_to_irq - delta > MIN_IRQ_SCHEDULE_TICKS {
                    st.timer.write_compare(when.ticks() as u32);
                    Ok(())
                } else {
                    Err(Error::CannotReschedule)
                }
            } else {
                Ok(())
            }
        })
    }

    fn wait_until(&mut self, when: Self::Instant) {
        while self.now() < when {
            if self.schedule_irq(when).is_ok() {
                wait();
            }
        }
    }
}

// ISR: CORE_TIMER
#[interrupt]
fn CORE_TIMER() {
    critical_section::with(|cs| {
        let mut stb = SYSTEM_TIMER.borrow_ref_mut(cs);
        let st = stb.as_mut().unwrap();

        let low = st.get_low_and_update_high();

        // set compare register to schedule next IRQ
        st.timer.write_compare(low.wrapping_add(u32::MAX / 2));
    });

    unsafe {
        (*INT::ptr()).ifs0clr.write(|w| w.ctif().bit(true));
    }
}
