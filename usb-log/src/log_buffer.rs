//! Log buffer
//!
// Copyright (C) 2022 Stephan <kiffie@mailbox.org>
// SPDX-License-Identifier: GPL-2.0-or-later

use core::cell::RefCell;
use core::fmt::Write;
use log::{Level, Metadata, Record};

struct LogBufferInner<const N: usize> {
    wr: usize,
    rd: usize,
    buf: [u8; N],
}

impl<const N: usize> LogBufferInner<N> {

    const fn new() -> LogBufferInner<N> {
        LogBufferInner {
            wr: 0,
            rd: 0,
            buf: [0; N],
        }
    }

    /// Write a byte
    /// 
    /// Returns an error if buffer is full
    fn write(&mut self, byte: u8) -> Result<(), ()> {
        if Self::inc_mod_n(self.wr) != self.rd {
            let w: usize = self.wr;
            self.buf[w] = byte;
            self.wr = Self::inc_mod_n(self.wr);
            Ok(())
        } else {
            Err(())
        }
    }

    /// Read a byte
    /// 
    /// Returns None if LogBuffer is empty
    pub fn read(&mut self) -> Option<u8> {
        if self.wr != self.rd {
            let byte = self.buf[self.rd];
            self.rd = Self::inc_mod_n(self.rd);
            Some(byte)
        } else {
            None
        }
    }

    fn inc_mod_n(val: usize) -> usize {
        if val + 1 < N {
            val + 1
        } else {
            0
        }
    }
}

pub struct LogBuffer<const N: usize> {
    inner: RefCell<LogBufferInner<N>>,
}

impl<const N: usize> LogBuffer<N> {

    pub const fn new() -> LogBuffer<N> {
        LogBuffer { inner: RefCell::new(LogBufferInner::new()) }
    }

    /// Read a byte
    /// 
    /// Returns None if LogBuffer is empty
    pub fn read(&self) -> Option<u8> {
        let mut inner = self.inner.borrow_mut();
        inner.read()
    }

}

impl<const N: usize> Write for LogBufferInner<N> {

    /// Write a string slice
    /// 
    /// If the buffer is full then the respective characters of the string slice are discarded
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        for byte in s.bytes() {
            if self.write(byte).is_err() {
                break;
            }
        }
        Ok(())
    }
}

unsafe impl<const N: usize> Sync for LogBuffer<N> {}
unsafe impl<const N: usize> Send for LogBuffer<N> {}

impl<const N: usize> log::Log for LogBuffer<N> {
    fn enabled(&self, metadata: &Metadata) -> bool {
        metadata.level() <= Level::Debug
    }

    fn log(&self, record: &Record) {
        let mut inner = self.inner.borrow_mut();
        if self.enabled(record.metadata()) {
            let _ = writeln!(
                inner,
                "[{}:{}] {}",
                record.file().unwrap_or("???"),
                record.line().unwrap_or(0),
                record.args()
            );
        }
    }

    fn flush(&self) {}
}

// #[panic_handler]
// fn panic(panic_info: &PanicInfo<'_>) -> ! {
//     if let Some(s) = panic_info.message() {
//         error!("Panic: {:?}", s);
//     } else {
//         error!("Panic");
//     }
//     if let Some(l) = panic_info.location() {
//         error!("location: [{}:{}]", l.file(), l.line());
//     }
//     error!("entering endless loop.");
//     loop {}
// }
