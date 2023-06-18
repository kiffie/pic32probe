//! System timer using the core timer
//!
// Copyright (C) 2022 Stephan <kiffie@mailbox.org>
// SPDX-License-Identifier: GPL-2.0-or-later

pub use core::time::Duration;

use core::cell::RefCell;
use core::ops::{Add, AddAssign};

//use rp2040_hal::pac::TIMER;
use rp2040_hal::pac::Interrupt;
use rp2040_hal::timer::{Alarm, Alarm0, Timer, ScheduleAlarmError};
use rp2040_hal::pac::interrupt;

use fugit::ExtU32;

use cortex_m::asm;
use cortex_m::interrupt::free as irq_free;
use cortex_m::interrupt::Mutex;
use cortex_m::peripheral::NVIC;

use log::debug;
use crate::Error;

const NANOS_PER_TICK: u64 = 1000;
const MIN_IRQ_SCHEDULE_TICKS: u64 = 1000;

pub struct SystemTimer {
    timer: Timer,
    alarm: Alarm0,
}

static SYSTEM_TIMER: Mutex<RefCell<Option<SystemTimer>>> = Mutex::new(RefCell::new(None));

#[derive(Clone, Copy, PartialEq, PartialOrd, Debug)]
pub struct Instant {
    ticks: u64,
}

impl SystemTimer {
    /// Initialize the system timer. This function must be called before other
    /// functions can be used. Otherwise, the other functions will panic.
    /// Panics if Alarm0 is not available. Furthermore, the NVIC is accessed to
    /// enable `TIMER_IRQ_0`.
    pub unsafe fn init(mut timer: Timer) {
        let alarm = timer.alarm_0().unwrap(); 
        irq_free(|cs| {
            let mut st = SYSTEM_TIMER.borrow(cs).borrow_mut();
            if st.is_some() {
                panic!("System timer already initialized.");
            }
            *st = Some(SystemTimer { timer, alarm });
        });
        NVIC::unmask(Interrupt::TIMER_IRQ_0);
    }

    /// Return Timer and Alarm0 instance resetting the System Timer to the uninitialized
    /// state. Panics if not initialized.
    pub fn free() -> (Timer, Alarm0) {
        irq_free(|cs| {
            let mut st = SYSTEM_TIMER.borrow(cs).borrow_mut().take().unwrap();
            st.alarm.disable_interrupt();
            (st.timer, st.alarm)
        })
    }

    /// Wait until the current time corresponds at least to `instant`.
    /// Performs a loop including an architecture specific wait instruction .
    pub fn wait_until(instant: Instant) {
        while Instant::now() < instant {
            if Self::schedule_irq(instant).is_ok() {
                asm::wfe();
            }
        }
    }

    //// Wait using a loop including an architecture specific wait instruction
    pub fn wait(delay: Duration) {
        let when = Instant::now() + delay;
        Self::wait_until(when);
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
        debug!(
            "now = {}, delta = {}",
            now,
            delta
        );
        irq_free(|cs| {
            let mut stb = SYSTEM_TIMER.borrow(cs).borrow_mut();
            let st = stb.as_mut().unwrap();
            st.alarm.enable_interrupt();
            st.alarm.schedule((delta as u32).micros())
                .map_err(|e| match e {
                    ScheduleAlarmError::AlarmTooLate => Error::InstantTooLate,
                })
        })
    }
}

impl Instant {
    /// Get current time
    pub fn now() -> Instant {
        irq_free(|cs| {
            let stb = SYSTEM_TIMER.borrow(cs).borrow();
            let st = stb.as_ref().unwrap();
            Instant { ticks: st.timer.get_counter().ticks() }
        })
    }
}

impl Add<Duration> for Instant {
    type Output = Self;

    fn add(self, rhs: Duration) -> Self {
        let duration_ticks = (rhs.as_nanos() / NANOS_PER_TICK as u128) as u64;
        Instant {
            ticks: self.ticks + duration_ticks,
        }
    }
}

impl AddAssign<Duration> for Instant {
    fn add_assign(&mut self, rhs: Duration) {
        let duration_ticks = (rhs.as_nanos() / NANOS_PER_TICK as u128) as u64;
        self.ticks += duration_ticks;
    }
}

#[interrupt]
fn TIMER_IRQ_0() {

    //debug!("!!!ALARM0 IRQ");
    irq_free(|cs| {
        let mut stb = SYSTEM_TIMER.borrow(cs).borrow_mut();
        let st = stb.as_mut().unwrap();
        st.alarm.clear_interrupt();
    });

}
