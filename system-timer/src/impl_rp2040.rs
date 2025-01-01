//! System timer using the core timer
//!
// Copyright (C) 2023 Stephan <kiffie@mailbox.org>
// SPDX-License-Identifier: GPL-2.0-or-later

use crate::Error;
use crate::SystemTimerOps;

use core::cell::RefCell;
use core::marker::PhantomData;
use cortex_m::asm;
use cortex_m::peripheral::NVIC;
use critical_section::Mutex;
use fugit::ExtU32;
use fugit::{TimerDurationU64, TimerInstantU64};
use log::debug;
use rp2040_hal::pac::{interrupt, Interrupt};
use rp2040_hal::timer::{Alarm, Alarm0, ScheduleAlarmError, Timer};

const TIMER_FREQ: u32 = 1_000_000;
const MIN_IRQ_SCHEDULE_TICKS: u64 = 1000;

pub type Instant = TimerInstantU64<TIMER_FREQ>;
pub type Duration = TimerDurationU64<TIMER_FREQ>;

pub struct SystemTimer {
    _private_dummy: PhantomData<()>,
}

pub struct SystemTimerInternal {
    timer: Timer,
    alarm: Alarm0,
}

static SYSTEM_TIMER: Mutex<RefCell<Option<SystemTimerInternal>>> = Mutex::new(RefCell::new(None));

impl SystemTimer {
    /// Initialize the system timer. This function must be called before other
    /// functions can be used. Otherwise, the other functions will panic.
    /// Panics if Alarm0 is not available. Furthermore, the NVIC is accessed to
    /// enable `TIMER_IRQ_0`.
    pub fn init(mut timer: Timer) {
        let alarm = timer.alarm_0().unwrap();
        critical_section::with(|cs| {
            let mut st = SYSTEM_TIMER.borrow_ref_mut(cs);
            if st.is_some() {
                panic!("System timer already initialized.");
            }
            *st = Some(SystemTimerInternal { timer, alarm });
        });
        unsafe {
            NVIC::unmask(Interrupt::TIMER_IRQ_0);
        }
    }

    /// Create a new system timer instance.
    ///
    /// Panics if not initialized.
    #[allow(clippy::new_without_default)]
    pub fn new() -> SystemTimer {
        critical_section::with(|cs| {
            if SYSTEM_TIMER.borrow_ref(cs).is_none() {
                panic!("Call to SystemTimer::new() without prior initialization");
            }
        });
        SystemTimer {
            _private_dummy: PhantomData,
        }
    }

    /// Return Timer and Alarm0 instance resetting the System Timer to the uninitialized
    /// state. Panics if not initialized.
    pub fn free() -> (Timer, Alarm0) {
        critical_section::with(|cs| {
            let mut st = SYSTEM_TIMER.borrow_ref_mut(cs).take().unwrap();
            st.alarm.disable_interrupt();
            (st.timer, st.alarm)
        })
    }
}

impl SystemTimerOps for SystemTimer {
    type Duration = Duration;
    type Instant = Instant;

    fn now(&self) -> Self::Instant {
        critical_section::with(|cs| {
            let stb = SYSTEM_TIMER.borrow_ref(cs);
            let st = stb.as_ref().unwrap();
            Instant::from_ticks(st.timer.get_counter().ticks())
        })
    }

    fn schedule_irq(&mut self, when: Self::Instant) -> Result<(), Error> {
        let now = self.now().ticks();
        if now > when.ticks() {
            return Err(Error::InstantTooEarly);
        }
        let delta = when.ticks() - now;
        if when.ticks() - now < MIN_IRQ_SCHEDULE_TICKS {
            return Err(Error::InstantTooEarly);
        }
        debug!("now = {}, delta = {}", now, delta);
        critical_section::with(|cs| {
            let mut stb = SYSTEM_TIMER.borrow_ref_mut(cs);
            let st = stb.as_mut().unwrap();
            st.alarm.enable_interrupt();
            st.alarm
                .schedule((delta as u32).micros())
                .map_err(|e| match e {
                    ScheduleAlarmError::AlarmTooLate => Error::InstantTooLate,
                })
        })
    }

    fn wait_until(&mut self, when: Self::Instant) {
        while self.now() < when {
            if self.schedule_irq(when).is_ok() {
                asm::wfe();
            }
        }
    }
}

#[interrupt]
fn TIMER_IRQ_0() {
    //debug!("!!!ALARM0 IRQ");
    critical_section::with(|cs| {
        let mut stb = SYSTEM_TIMER.borrow_ref_mut(cs);
        let st = stb.as_mut().unwrap();
        st.alarm.clear_interrupt();
    });
}
