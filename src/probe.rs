//! ISCP implementation
//!
// Copyright (C) 2022 Stephan <kiffie@mailbox.org>
// SPDX-License-Identifier: GPL-2.0-or-later

use core::time::Duration;
use log::debug;
use rp2040_hal::pac::RESETS;
use rp2040_hal::pio::{
    InstalledProgram, PIOBuilder, PIOExt, PinDir, PinState, Running, Rx, ShiftDirection,
    StateMachine, Stopped, Tx, UninitStateMachine, SM0,
};
use system_timer::{Instant, SystemTimer};

const PRACC_TIMEOUT: Duration = Duration::from_millis(1000);

#[derive(Clone, Copy, Debug)]
#[non_exhaustive]
pub enum Error {
    // /// Invalid arguments, e.g. slice size mismatch
    // InvalidArgument,
    /// Timeout during Fast Data transfer
    FastDataTimeout,
}

type Result<T> = core::result::Result<T, Error>;

pub trait Pic32Comm {
    /// Connect to target device
    /// Perform a reset and debug interface initialization sequence
    fn iscp_connect_key_sequence(&mut self, icsp_clock_hz: u32);

    /// Disconnect from target device
    fn disconnect(&mut self);

    /// Goto Run-Test/Idle state
    fn run_test_idle(&mut self);

    /// Write into the 5 bits TAP command register
    fn send_command(&mut self, command: JtagCommand);

    /// 8-bit data register transfer
    fn xfer_data_u8(&mut self, data: u8) -> u8;

    /// 32-bit data register transfer
    fn xfer_data_u32(&mut self, data: u32) -> u32;

    /// MIPS EJTAG 32-bit Fast Data transfer
    fn xfer_fastdata(&mut self, data: u32) -> Result<u32>;

    /// transfer an instruction to the core
    ///
    /// The core must be executing from dmseg memory and the EJTAG TAP must be selected.
    fn xfer_instruction(&mut self, instruction: u32) -> Result<()> {
        self.send_command(JtagCommand::EtapControl);
        let timeout = Instant::now() + PRACC_TIMEOUT;
        loop {
            let control = self.xfer_data_u32(0x0004c000);
            if control & ejtag_control::PRACC != 0 {
                break;
            }
            if Instant::now() >= timeout {
                debug!("timeout, control = {control:08x}");
                return Err(Error::FastDataTimeout);
            }
        }
        self.send_command(JtagCommand::EtapData);
        self.xfer_data_u32(instruction);
        self.send_command(JtagCommand::EtapControl);
        self.xfer_data_u32(0x0000c000);
        Ok(())
    }
}

#[repr(u8)]
#[derive(Clone, Copy, Debug)]
pub enum JtagCommand {
    IdCode = 0x01,
    MtapSwMtap = 0x04,
    MtapSwEtap = 0x05,
    MtapCommand = 0x07,
    EtapAddress = 0x08,
    EtapData = 0x09,
    EtapControl = 0x0a,
    EtapAll = 0x0b,
    EtapEjtagBoot = 0x0c,
    EtapNormalBoot = 0x0d,
    EtapFastData = 0x0e,
    Bypass = 0x1f,
}

impl From<JtagCommand> for u8 {
    fn from(c: JtagCommand) -> u8 {
        c as u8
    }
}

/// Microchip TAP (MTAP) commands
#[repr(u8)]
#[derive(Clone, Copy, Debug)]
pub enum MtapCommand {
    /// NOP / Return Status
    Status = 0x00,

    /// Assert device reset
    AssertReset = 0xD1,

    /// Remove device reset
    DeassertReset = 0xD0,

    /// Flash chip erase
    Erase = 0xFC,

    /// Enable access from CPU to flash
    FlashEnable = 0xFE,

    /// Disable access from CPU to flash
    FlashDisable = 0xFD,

    /// Reread the configuration settings and initialize accordingly
    ReadConfig = 0xff,
}

impl From<MtapCommand> for u8 {
    fn from(c: MtapCommand) -> Self {
        c as u8
    }
}

/// MTAP Status bits
#[allow(dead_code)]
pub mod mtap_status {
    /// Device is NOT code-protected
    pub const CPS: u8 = 0x80;

    /// Error occured during NVM operation
    pub const NVMERR: u8 = 0x20;

    /// Configuration has been read and Code-Protect State bit is valid
    pub const CFGRDY: u8 = 0x08;

    /// Flash Controller is Busy (erase is in progress)
    pub const FCBUSY: u8 = 0x04;

    /// Flash access is enabled
    pub const FAEN: u8 = 0x02;

    /// Device reset is active
    pub const DEVRST: u8 = 0x01;
}

/// EJTAG Control register bits
#[allow(dead_code)]
pub mod ejtag_control {
    /// Reset occured
    pub const ROCC: u32 = 1 << 31;
    /// Size of pending access
    pub const PSZ_MASK: u32 = 3 << 29;
    /// Byte
    pub const PSZ_BYTE: u32 = 0 << 29;
    /// Half-word
    pub const PSZ_HALFWORD: u32 = 1 << 29;
    /// Word
    pub const PSZ_WORD: u32 = 2 << 29;
    /// Triple, double-word
    pub const PSZ_TRIPLE: u32 = 3 << 29;
    /// VPE disabled
    pub const VPED: u32 = 1 << 23;
    /// Processor in low-power mode
    pub const DOZE: u32 = 1 << 22;
    /// System bus clock stopped
    pub const HALT: u32 = 1 << 21;
    /// Peripheral reset applied
    pub const PERRST: u32 = 1 << 20;
    /// Processor Access Read and Write (0: Read, 1: Write)
    pub const PRNW: u32 = 1 << 19;
    /// Pending processor access
    pub const PRACC: u32 = 1 << 18;
    /// Relocatable debug exception vector
    pub const RDVEC: u32 = 1 << 17;
    /// Processor reset applied
    pub const PRRST: u32 = 1 << 16;
    /// Probe will service processor accesses
    pub const PROBEN: u32 = 1 << 15;
    /// Debug vector at ff200200
    pub const PROBTRAP: u32 = 1 << 14;
    /// Debug interrupt exception
    pub const EJTAGBRK: u32 = 1 << 12;
    /// Debug mode
    pub const DM: u32 = 1 << 3;
}

const RESET_DELAY: Duration = Duration::from_millis(1);
const XFER_DELAY: Duration = Duration::from_micros(5);

pub struct Rp2040Comm<P: PIOExt> {
    pgec: u8,
    pged: u8,
    mclr: u8,
    sm: Option<IcspStateMachine<P>>,
}

impl<P: PIOExt> Rp2040Comm<P> {
    pub fn new(piox: P, pgec: u8, pged: u8, mclr: u8, resets: &mut RESETS) -> Rp2040Comm<P> {
        let (mut pio, sm0, _, _, _) = piox.split(resets);

        // PIO program for ICSP
        // side set pin: PGEC (Clock)
        // set/out     : PGED (Data)
        let mut a = pio::Assembler::<32>::new_with_side_set(pio::SideSet::new(false, 1, false));
        let mut wrap_target = a.label();
        let mut wrap_source = a.label();

        a.bind(&mut wrap_target);

        // output TDI
        a.out_with_side_set(pio::OutDestination::PINS, 1, 1);
        a.nop_with_side_set(0);

        // output TMS
        a.out_with_side_set(pio::OutDestination::PINS, 1, 1);
        a.nop_with_side_set(0);

        // unused clock cycle
        a.set_with_side_set(pio::SetDestination::PINDIRS, 0, 1);
        a.nop_with_side_set(0);

        // Input TDO
        a.in_with_side_set(pio::InSource::PINS, 1, 1);
        a.set_with_side_set(pio::SetDestination::PINDIRS, 1, 0);

        a.bind(&mut wrap_source);

        let program = a.assemble_with_wrap(wrap_source, wrap_target);
        let installed = pio.install(&program).unwrap();

        let mut comm = Rp2040Comm {
            pgec,
            pged,
            mclr,
            sm: None,
        };
        let (stopped, rx, tx, wrap_target) = comm.init_sm(sm0, installed, 1_000_000);
        comm.sm = Some(IcspStateMachine::new(stopped, rx, tx, wrap_target));
        comm
    }

    #[allow(clippy::type_complexity)]
    fn init_sm(
        &self,
        usm: UninitStateMachine<(P, SM0)>,
        prog: InstalledProgram<P>,
        icsp_clock_hz: u32,
    ) -> (
        StateMachine<(P, SM0), Stopped>,
        Rx<(P, SM0)>,
        Tx<(P, SM0)>,
        u8,
    ) {
        assert!(self.sm.is_none());
        // The clock of the PIO state machine should be 2 * icsp_clock_hz
        // because two cycles are needed to generate one ICSP clock. The divider
        // is rounded up to avoid exceeding the requested clock frequency.
        let sys_clock = 125_000_000;
        let clock_div = 1 + (sys_clock - 1) / icsp_clock_hz / 2; // round up
        debug!(
            "ICSP clock: requested = {}, real = {}, div = {}",
            icsp_clock_hz,
            sys_clock / clock_div / 2,
            clock_div
        );
        let wrap_target = prog.wrap_target();

        let (mut stopped, rx, tx) = PIOBuilder::from_program(prog)
            .in_pin_base(self.pged)
            .out_pins(self.pged, 1)
            .set_pins(self.pged, 1)
            .side_set_pin_base(self.pgec)
            .clock_divisor(clock_div as f32)
            .autopull(true)
            .pull_threshold(2)
            .autopush(false)
            .in_shift_direction(ShiftDirection::Right)
            .out_shift_direction(ShiftDirection::Right)
            .build(usm);
        stopped.set_pindirs([
            (self.pgec, PinDir::Input),
            (self.pged, PinDir::Input),
            (self.mclr, PinDir::Input),
        ]);
        (stopped, rx, tx, wrap_target)
    }

    #[allow(clippy::type_complexity)]
    fn reinit_sm(
        &mut self,
        icsp_clock_hz: u32,
    ) -> (
        StateMachine<(P, SM0), Stopped>,
        Rx<(P, SM0)>,
        Tx<(P, SM0)>,
        u8,
    ) {
        let (sm, rx, tx, _) = self.sm.take().unwrap().release();
        let (usm, prog) = sm.uninit(rx, tx);
        self.init_sm(usm, prog, icsp_clock_hz)
    }

    // pub fn icsp_test_pattern(&mut self) {

    //     for pattern in [0b000, 0b010, 0b100, 0b110] {
    //         self.icsp_tx.write_blocking(pattern);
    //     }
    //     self.icsp_tx.write_blocking(0b001);
    //     let bits = self.icsp_rx.read_blocking();
    //     debug!("ICSP test pattern result: {:08x}", bits);

    // }
}

impl<P: PIOExt> Pic32Comm for Rp2040Comm<P> {
    fn iscp_connect_key_sequence(&mut self, icsp_clock_hz: u32) {
        let (mut stopped, rx, tx, wrap_target) = self.reinit_sm(icsp_clock_hz);
        stopped.set_pins([(self.pgec, PinState::Low), (self.mclr, PinState::High)]);
        stopped.set_pindirs([
            (self.pgec, PinDir::Output),
            (self.pged, PinDir::Output),
            (self.mclr, PinDir::Output),
        ]);
        SystemTimer::wait(RESET_DELAY);
        stopped.set_pins([(self.mclr, PinState::Low)]);
        SystemTimer::wait(RESET_DELAY);
        let mut key_sequence = 0x4d434850u32;
        for _ in 0..=31 {
            let bit = if key_sequence & (1 << 31) != 0 {
                PinState::High
            } else {
                PinState::Low
            };
            key_sequence <<= 1;
            stopped.set_pins([(self.pgec, PinState::Low), (self.pged, bit)]);
            SystemTimer::wait(XFER_DELAY);
            stopped.set_pins([(self.pgec, PinState::High)]);
            SystemTimer::wait(XFER_DELAY);
        }
        stopped.set_pins([(self.pgec, PinState::Low)]);
        SystemTimer::wait(RESET_DELAY);
        stopped.set_pins([(self.mclr, PinState::High)]);
        SystemTimer::wait(RESET_DELAY);
        self.sm = Some(IcspStateMachine::new(stopped, rx, tx, wrap_target));
    }

    fn disconnect(&mut self) {
        //let (sm, rx, mut tx) = self.sm.take().unwrap().release();
        let mut sm = self.sm.take().unwrap();
        // go to test logic reset state
        sm.icsp_4phase(true, false);
        sm.icsp_4phase(true, false);
        sm.icsp_4phase(true, false);
        sm.icsp_4phase(true, false);
        sm.icsp_4phase(true, false);
        SystemTimer::wait(RESET_DELAY);

        let (mut stopped, rx, tx, wrap_target) = sm.release();
        stopped.set_pins([(self.mclr, PinState::Low)]);
        SystemTimer::wait(XFER_DELAY);
        stopped.set_pins([(self.pgec, PinState::High)]);
        SystemTimer::wait(XFER_DELAY);
        stopped.set_pins([(self.pgec, PinState::Low)]);
        SystemTimer::wait(RESET_DELAY);
        stopped.set_pins([(self.mclr, PinState::High)]);
        SystemTimer::wait(RESET_DELAY);
        stopped.set_pindirs([
            (self.pgec, PinDir::Input),
            (self.pged, PinDir::Input),
            (self.mclr, PinDir::Input),
        ]);
        self.sm = Some(IcspStateMachine::new(stopped, rx, tx, wrap_target));
    }

    fn run_test_idle(&mut self) {
        let sm = self.sm.as_mut().unwrap();

        sm.icsp_4phase(true, false);
        sm.icsp_4phase(true, false);
        sm.icsp_4phase(true, false);
        sm.icsp_4phase(true, false);
        sm.icsp_4phase(true, false);
        sm.icsp_4phase(false, false);
    }

    fn send_command(&mut self, command: JtagCommand) {
        let sm = self.sm.as_mut().unwrap();
        sm.icsp_4phase(true, false);
        sm.icsp_4phase(true, false);
        sm.icsp_4phase(false, false);
        sm.icsp_4phase(false, false);

        for i in 0..=4 {
            let tdi = command as u32 & (1 << i) != 0;
            // set TMS for last 4-phase transaction
            let tms = i == 4;
            sm.icsp_4phase(tms, tdi);
        }
        sm.icsp_4phase(true, false);
        sm.icsp_4phase(false, false);
    }

    fn xfer_data_u8(&mut self, data: u8) -> u8 {
        let sm = self.sm.as_mut().unwrap();
        sm.icsp_4phase(true, false);
        sm.icsp_4phase(false, false);
        sm.icsp_4phase(false, false);

        let mut byte = 0;
        for i in 0..=7 {
            let tdi = data & (1 << i) != 0;
            // set TMS for last 4-phase transaction and move read the data
            if i == 7 {
                byte = sm.read_tdo_u8();
                sm.icsp_4phase(true, tdi);
            } else {
                sm.icsp_4phase(false, tdi);
            }
        }
        sm.icsp_4phase(true, false);
        sm.icsp_4phase(false, false);
        byte
    }

    fn xfer_data_u32(&mut self, data: u32) -> u32 {
        let sm = self.sm.as_mut().unwrap();
        sm.icsp_4phase(true, false);
        sm.icsp_4phase(false, false);
        sm.icsp_4phase(false, false);

        let mut word = 0;
        for i in 0..=31 {
            let tdi = data & (1 << i) != 0;
            // set TMS for last 4-phase transaction and move read data
            if i == 31 {
                word = sm.read_tdo_u32();
                sm.icsp_4phase(true, tdi);
            } else {
                sm.icsp_4phase(false, tdi);
            }
        }
        sm.icsp_4phase(true, false);
        sm.icsp_4phase(false, false);
        word
    }

    fn xfer_fastdata(&mut self, data: u32) -> Result<u32> {
        let sm = self.sm.as_mut().unwrap();
        let timeout = Instant::now() + PRACC_TIMEOUT;
        loop {
            sm.icsp_4phase(true, false);
            sm.icsp_4phase(false, false);
            sm.icsp_4phase(false, false); // receive PrAcc
            let pracc = sm.read_tdo_bool(); // read PrAcc
            sm.icsp_4phase(false, false); // transmit PrAcc = 0, receive LSB
            let mut word = 0;
            for i in 0..=31 {
                let tdi = data & (1 << i) != 0;
                // set TMS for last 4-phase transaction and move read data into FIFO
                if i == 31 {
                    word = sm.read_tdo_u32();
                    sm.icsp_4phase(true, tdi);
                } else {
                    sm.icsp_4phase(false, tdi);
                }
            }
            sm.icsp_4phase(true, false);
            sm.icsp_4phase(false, false);
            if pracc {
                return Ok(word);
            }
            if Instant::now() >= timeout {
                return Err(Error::FastDataTimeout);
            }
        }
    }
}

struct IcspStateMachine<P: PIOExt> {
    sm: StateMachine<(P, SM0), Running>,
    rx: Rx<(P, SM0)>,
    tx: Tx<(P, SM0)>,
    wrap_target: u8,
}

impl<P: PIOExt> IcspStateMachine<P> {
    fn new(
        sm: StateMachine<(P, SM0), Stopped>,
        rx: Rx<(P, SM0)>,
        tx: Tx<(P, SM0)>,
        wrap_target: u8,
    ) -> IcspStateMachine<P> {
        IcspStateMachine {
            sm: sm.start(),
            rx,
            tx,
            wrap_target,
        }
    }

    #[allow(clippy::type_complexity)]
    fn release(
        self,
    ) -> (
        StateMachine<(P, SM0), Stopped>,
        Rx<(P, SM0)>,
        Tx<(P, SM0)>,
        u8,
    ) {
        (self.sm.stop(), self.rx, self.tx, self.wrap_target)
    }

    /// Perform one 4phase transaction. The resulting TDO bit will be shifted into the ISR.
    fn icsp_4phase(&mut self, tms: bool, tdi: bool) {
        let value = ((tms as u32) << 1) | tdi as u32;
        while !self.tx.write(value) {}
    }

    /// Read TDO bits of the previous 4-phase transactions
    ///
    /// `n_bits` must be <= 32
    fn read_tdo_u32(&mut self) -> u32 {
        // wait until all transactions are completed
        while !self.tx.is_empty() || self.sm.instruction_address() != self.wrap_target as u32 {}
        // push ISR to the RX FIFO
        self.sm.exec_instruction(
            pio::Instruction {
                operands: pio::InstructionOperands::PUSH {
                    if_full: false,
                    block: false,
                },
                delay: 0,
                side_set: Some(1),
            }
            .encode(pio::SideSet::new(false, 1, false)),
        );
        // fetch one word from the RX FIFO
        loop {
            if let Some(word) = self.rx.read() {
                break word;
            }
        }
    }

    fn read_tdo_u8(&mut self) -> u8 {
        (self.read_tdo_u32() >> 24) as u8
    }

    fn read_tdo_bool(&mut self) -> bool {
        self.read_tdo_u32() & (1 << 31) != 0
    }
}
