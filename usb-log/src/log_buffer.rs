//! Log buffer
//!
// Copyright (C) 2022 Stephan <kiffie@mailbox.org>
// SPDX-License-Identifier: GPL-2.0-or-later

use core::cell::RefCell;
use core::fmt::Write;
use critical_section::Mutex;
use log::{Metadata, Record};

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

    /// Returns true if LogBuffer is empty
    pub fn is_empty(&self) -> bool {
        self.wr == self.rd
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
    inner: Mutex<RefCell<LogBufferInner<N>>>,
}

impl<const N: usize> LogBuffer<N> {
    #[allow(clippy::new_without_default)]
    pub const fn new() -> LogBuffer<N> {
        LogBuffer {
            inner: Mutex::new(RefCell::new(LogBufferInner::new())),
        }
    }

    /// Read a byte
    ///
    /// Returns None if LogBuffer is empty
    pub fn read(&self) -> Option<u8> {
        critical_section::with(|cs| {
            let mut inner = self.inner.borrow(cs).borrow_mut();
            inner.read()
        })
    }

    /// Returns true if LogBuffer is empty
    pub fn is_empty(&self) -> bool {
        critical_section::with(|cs| self.inner.borrow(cs).borrow().is_empty())
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

impl<const N: usize> log::Log for LogBuffer<N> {
    fn enabled(&self, _metadata: &Metadata) -> bool {
        true
    }

    fn log(&self, record: &Record) {
        const MAX_FILE_LEN: usize = 32;
        critical_section::with(|cs| {
            let mut inner = self.inner.borrow(cs).borrow_mut();
            if self.enabled(record.metadata()) {
                if record.target() == "PANIC" {
                    writeln!(inner, "[PANIC] {}", record.args()).ok();
                } else {
                    let (prefix, file) = if let Some(f) = record.file_static() {
                        if f.len() <= MAX_FILE_LEN {
                            ("", f)
                        } else {
                            ("...", &f[f.len() - MAX_FILE_LEN..])
                        }
                    } else {
                        ("???", "")
                    };
                    writeln!(
                        inner,
                        "[{}{}:{}] {}",
                        prefix,
                        file,
                        record.line().unwrap_or(0),
                        record.args()
                    )
                    .ok();
                }
            }
        });
    }

    fn flush(&self) {}
}

#[cfg(feature = "panic-handler")]
use core::panic::PanicInfo;
#[cfg(feature = "panic-handler")]
use log::error;

#[cfg(feature = "panic-handler")]
#[panic_handler]
fn panic(panic_info: &PanicInfo<'_>) -> ! {
    if let Some(l) = panic_info.location() {
        error!(target: "PANIC", "at {}:{}", l.file(), l.line());
    }
    error!(target: "PANIC", "{}", panic_info.message());
    error!(target: "PANIC", "entering endless loop.");
    loop {}
}
