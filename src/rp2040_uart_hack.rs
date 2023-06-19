//! Hack to set the baudrate of an UART handled by the HAL without disabling the UART

use fugit::HertzU32;
use rp2040_hal::pac::{Peripherals, UART0, UART1};
use rp2040_hal::uart::{Enabled, UartDevice, UartPeripheral, ValidUartPinout};

// Error type for UART operations.
#[derive(Debug)]
pub enum Error {
    /// Bad argument : when things overflow, ...
    BadArgument,
}

pub trait UartConfigExt {
    fn set_baudrate(&mut self, baudrate: HertzU32, frequency: HertzU32) -> Result<HertzU32, Error>;
}

impl<P: ValidUartPinout<UART0>> UartConfigExt for UartPeripheral<Enabled, UART0, P> {
    fn set_baudrate(&mut self, baudrate: HertzU32, frequency: HertzU32) -> Result<HertzU32, Error> {
        let mut uart0 = unsafe { Peripherals::steal().UART0 };
        configure_baudrate(&mut uart0, baudrate, frequency)
    }
}

impl<P: ValidUartPinout<UART1>> UartConfigExt for UartPeripheral<Enabled, UART1, P> {
    fn set_baudrate(&mut self, baudrate: HertzU32, frequency: HertzU32) -> Result<HertzU32, Error> {
        let mut uart1 = unsafe { Peripherals::steal().UART1 };
        configure_baudrate(&mut uart1, baudrate, frequency)
    }
}

/// The PL011 (PrimeCell UART) supports a fractional baud rate divider
/// From the wanted baudrate, we calculate the divider's two parts: integer and fractional parts.
/// Code inspired from the C SDK.
fn calculate_baudrate_dividers(
    wanted_baudrate: HertzU32,
    frequency: HertzU32,
) -> Result<(u16, u16), Error> {
    // See Chapter 4, Section 2 §7.1 from the datasheet for an explanation of how baudrate is
    // calculated
    let baudrate_div = frequency
        .to_Hz()
        .checked_mul(8)
        .and_then(|r| r.checked_div(wanted_baudrate.to_Hz()))
        .ok_or(Error::BadArgument)?;

    Ok(match (baudrate_div >> 7, ((baudrate_div & 0x7F) + 1) / 2) {
        (0, _) => (1, 0),

        (int_part, _) if int_part >= 65535 => (65535, 0),

        (int_part, frac_part) => (int_part as u16, frac_part as u16),
    })
}

/// Baudrate configuration. Code loosely inspired from the C SDK.
fn configure_baudrate<U: UartDevice>(
    device: &mut U,
    wanted_baudrate: HertzU32,
    frequency: HertzU32,
) -> Result<HertzU32, Error> {
    let (baud_div_int, baud_div_frac) = calculate_baudrate_dividers(wanted_baudrate, frequency)?;

    // First we load the integer part of the divider.
    device.uartibrd.write(|w| unsafe {
        w.baud_divint().bits(baud_div_int);
        w
    });

    // Then we load the fractional part of the divider.
    device.uartfbrd.write(|w| unsafe {
        w.baud_divfrac().bits(baud_div_frac as u8);
        w
    });

    // PL011 needs a (dummy) line control register write to latch in the
    // divisors. We don't want to actually change LCR contents here.
    device.uartlcr_h.modify(|_, w| w);

    Ok(HertzU32::from_raw(
        (4 * frequency.to_Hz()) / (64 * baud_div_int as u32 + baud_div_frac as u32),
    ))
}
