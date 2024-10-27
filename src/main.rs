//! Pic32Probe --- ICSP (2-wire) Programming Dongle for PIC32 MCUs
//
// Copyright (C) 2022 Stephan <kiffie@mailbox.org>
// SPDX-License-Identifier: GPL-2.0-or-later
//
#![no_std]
#![no_main]

use cfg_if::cfg_if;
use core::cmp::min;
use cortex_m::interrupt;
use cortex_m_rt::entry;
use log::{debug, error, info, LevelFilter};
use panic_persist as _;

use hal::{
    clocks::{init_clocks_and_plls, Clock},
    gpio::{FunctionPio0, FunctionUart},
    pac,
    sio::Sio,
    uart::{DataBits, StopBits, UartConfig, UartPeripheral},
    usb::UsbBus,
    watchdog::Watchdog,
    Timer,
};
use rp2040_hal as hal;

use fugit::RateExtU32;
use usb_device::class_prelude::UsbBusAllocator;
use usbd_serial::SerialPort;

use system_timer::SystemTimer;
use usb_log::{log_buffer::LogBuffer, usb_log_channel::UsbLogChannel};

use usb_device::prelude::*;

mod probe;

mod rp2040_uart_hack;
use rp2040_uart_hack::UartConfigExt;

mod adapter;
use adapter::Pic32Adapter;

mod usb;
use usb::ProbeClass;

#[link_section = ".boot2"]
#[used]
pub static BOOT_LOADER: [u8; 256] = rp2040_boot2::BOOT_LOADER_W25Q080;

static LOGGER: LogBuffer<1024> = LogBuffer::new();

#[entry]
fn main() -> ! {
    //rtt_init_default!();
    //let up_write = channels.up.0;
    unsafe {
        log::set_logger_racy(&LOGGER).unwrap();
        log::set_max_level_racy(LevelFilter::Debug);
    }

    info!("PIC32probe v{}", env!("CARGO_PKG_VERSION"));
    info!("Build: {}", env!("BUILD_DATETIME"));
    if let Some(panic_message) = panic_persist::get_panic_message_utf8() {
        error!("Device panicked: {panic_message}");
    }
    let mut pac = pac::Peripherals::take().unwrap();
    let mut watchdog = Watchdog::new(pac.WATCHDOG);
    let sio = Sio::new(pac.SIO);

    // External high-speed crystal on the pico board is 12Mhz
    let external_xtal_freq_hz = 12_000_000u32;
    let clocks = init_clocks_and_plls(
        external_xtal_freq_hz,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    // global IRQ enable
    unsafe {
        interrupt::enable();
    }

    // System timer
    let timer = Timer::new(pac.TIMER, &mut pac.RESETS, &clocks);
    unsafe {
        SystemTimer::init(timer);
    }

    let pins = hal::gpio::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    cfg_if! {
        if #[cfg(feature = "adafruit_itsy")] {
            info!("Target board: Adafruit ItsyBitsy RP2040");
            let _pgec = pins.gpio3.into_mode::<FunctionPio0>();
            let _pged = pins.gpio2.into_mode::<FunctionPio0>();
            let _mclr = pins.gpio6.into_mode::<FunctionPio0>();

            // enable weak pull-up for MCLR pin
            let pad = unsafe { pac::Peripherals::steal() };
            pad.PADS_BANK0.gpio[6].write(|w| w.pde().bit(false).pue().bit(true));
            debug!("pad GPIO2: {:08x}", pad.PADS_BANK0.gpio[2].read().bits());
            debug!("pad GPIO3: {:08x}", pad.PADS_BANK0.gpio[3].read().bits());
            debug!("pad GPIO6: {:08x}", pad.PADS_BANK0.gpio[6].read().bits());

            let probe = probe::Rp2040Comm::new(pac.PIO0, 3, 2, 6, &mut pac.RESETS);
            let mut adapter = Pic32Adapter::new(probe);

            let mut led_pin = pins.gpio11.into_push_pull_output();
            led_pin.set_high().unwrap();
            //let button_pin = pins.gpio12.into_pull_up_input();

            // UART
            let mut uart = UartPeripheral::new(pac.UART0, &mut pac.RESETS)
                .enable(
                    uart::common_configs::_115200_8_N_1,
                    clocks.peripheral_clock.freq(),
                )
                .unwrap();
            let _txd_pin = pins.gpio20.into_mode::<FunctionUart>();
            let mut rxd_pin = pins.gpio5.into_mode::<FunctionUart>();
        } else if #[cfg(feature = "adafruit_qt")] {
            info!("Target board: Adafruit QT Py RP2040");
            let _pgec = pins.gpio6.into_function::<FunctionPio0>();
            let _pged = pins.gpio4.into_function::<FunctionPio0>();
            let _mclr = pins.gpio3.into_function::<FunctionPio0>();


            // enable weak pull-up for MCLR pin
            let pad = unsafe { pac::Peripherals::steal() };
            pad.PADS_BANK0.gpio[3].write(|w| w.pde().bit(false).pue().bit(true));
            debug!("pad GPIO3: {:08x}", pad.PADS_BANK0.gpio[3].read().bits());
            debug!("pad GPIO4: {:08x}", pad.PADS_BANK0.gpio[4].read().bits());
            debug!("pad GPIO6: {:08x}", pad.PADS_BANK0.gpio[6].read().bits());

            let probe = probe::Rp2040Comm::new(pac.PIO0, 6, 4, 3, &mut pac.RESETS);
            let mut adapter = Pic32Adapter::new(probe);

            //let button_pin = pins.gpio21.into_pull_up_input();

            // UART
            let uart_pins = (
                pins.gpio20.into_function::<FunctionUart>(), // TX
                pins.gpio5.into_function::<FunctionUart>(), // RX
            );
            let mut uart = UartPeripheral::new(pac.UART1, uart_pins, &mut pac.RESETS)
                .enable(
                    UartConfig::new(115200u32.Hz(), DataBits::Eight, None, StopBits::One),
                    clocks.peripheral_clock.freq(),
                )
                .unwrap();
        } else {
            compile_error!("no board selected");
        }
    }
    let usb_bus = UsbBusAllocator::new(UsbBus::new(
        pac.USBCTRL_REGS,
        pac.USBCTRL_DPRAM,
        clocks.usb_clock,
        true,
        &mut pac.RESETS,
    ));

    let mut probe_class = ProbeClass::new(&usb_bus);
    let mut serial_class = SerialPort::new(&usb_bus);
    let mut log_channel = UsbLogChannel::new(&usb_bus, "kiffielog", &LOGGER);

    let mut usb_dev = UsbDeviceBuilder::new(&usb_bus, UsbVidPid(0x16c0, 0x05dc))
        .max_packet_size_0(64)
        .manufacturer("Kiffie Labs https://github.com/kiffie")
        .product("Pic32Probe")
        .build();

    let mut response = [0u8; 1024];
    let mut len = 0;
    let mut tndx = 0;
    let mut serial_rx = [0; 1];
    let mut serial_has_rx = false;
    let mut serial_rxbuf = [0; 32];
    let mut baudrate = 115200;
    loop {
        usb_dev.poll(&mut [&mut probe_class, &mut serial_class, &mut log_channel]);

        if len == 0 {
            if let Some(data) = probe_class.receive() {
                len = adapter.process(data, &mut response);
            }
        } else {
            let l = min(len - tndx, 64);
            if probe_class.transmit(&response[tndx..(tndx + l)]).is_ok() {
                tndx += l;
                if tndx == len && l < 64 {
                    len = 0;
                    tndx = 0;
                }
            }
        }

        if !serial_has_rx {
            match serial_class.read(&mut serial_rx) {
                Ok(len) => {
                    assert!(len == 1);
                    serial_has_rx = uart.write_raw(&serial_rx).is_err();
                }
                Err(UsbError::WouldBlock) => {}
                Err(e) => {
                    error!("Serial read failed: {e:?}");
                }
            }
        }

        if let Ok(len) = uart.read_raw(&mut serial_rxbuf) {
            serial_class.write(&serial_rxbuf[..len]).ok();
        }

        if serial_class.line_coding().data_rate() != baudrate {
            baudrate = serial_class.line_coding().data_rate();
            match uart.set_baudrate(baudrate.Hz(), clocks.peripheral_clock.freq()) {
                Err(_) => info!("Baudrate {baudrate} out of range; not changed"),
                Ok(real) => info!(" Baudrate changed: requested = {baudrate}, real = {real}"),
            }
        }

        // probe.iscp_connect_key_sequence();
        // probe.run_test_idle();
        // let id_code = probe.id_code();
        // debug!("id_code = {id_code:08x}");
    }

    //panic!("This statement should never be reached.");
}
