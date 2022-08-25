//! Target independent system timer
// Copyright (C) 2022 Stephan <kiffie@mailbox.org>
// SPDX-License-Identifier: GPL-2.0-or-later

#![no_std]
use embedded_hal::timer::{CountDown, Periodic};
//use embedded_time::duration::Duration;
use void::Void;

#[cfg(not(feature = "device-selected"))]
compile_error!("This crate requires one device feature to be enabled");

#[cfg(feature = "pic32")]
mod impl_pic32;

#[cfg(feature = "pic32")]
pub use impl_pic32::{Instant, SystemTimer};

#[cfg(feature = "rp2040")]
mod impl_rp2040;

#[cfg(feature = "rp2040")]
pub use impl_rp2040::{Instant, SystemTimer};

pub use core::time::Duration;

#[derive(Debug)]
pub enum Error {
    InstantTooEarly,
    InternalError,
}

pub struct Timer {
    elapse: Option<Instant>
}

impl Timer {
    pub fn new() -> Timer  {
        Timer { elapse: None }
    }
}

impl CountDown for Timer {
    type Time = Duration;

    fn start<T>(&mut self, count: T)
    where
        T: Into<Self::Time>
    {
        self.elapse = Some(Instant::now() + count.into());
    }

    fn wait(&mut self) -> nb::Result<(), Void> {

        let elapse = self.elapse.unwrap();
        if Instant::now() >= elapse {
            Ok(())
        } else {
            Err(nb::Error::WouldBlock)
        }
    }
}

impl Periodic for Timer {}
