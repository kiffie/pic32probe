//! Adapter for Flashing a PIC32 Device
//! Has a message based interface e.g. for USB bulk or HID communication
//!
//! # Protocol
//!
//! ## Message format
//!
//! ```txt
//! +----------------------------------------+
//! |  request code / status code            |
//! *----------------------------------------+
//! |  length without header in bytes        |
//! +----------------------------------------+
//! |     ...                                |
//! ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//! ```
//!
//! Messages must start at USB packet boundaries. A message can span multiple
//! USB packets. Transport directly over bulk endpoints or over HID is possible.
//!
//! ## Request Codes
//!
//! ## Status Codes
//!
//! 0: Ok
//! 1: Invalid Message or Argument
//!
// Copyright (C) 2022 Stephan <kiffie@mailbox.org>
// SPDX-License-Identifier: GPL-2.0-or-later

use crate::probe::{self, ejtag_control, mtap_status, JtagCommand, MtapCommand, Pic32Comm};

use byteorder::{ByteOrder, LittleEndian as LE};
use log::debug;
use system_timer::{Duration, Instant, SystemTimer};

#[repr(u16)]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum RequestCode {
    /// Connect to target and reset it
    ConnectReset = 0x0001,

    /// Disconnect from target
    Disconnect = 0x0002,

    /// Read JTAG IDCODE register
    GetIdCode = 0x0003,

    /// Read Word assuming that the core is accepting classic MIPS32 instructions
    ReadWordClassic = 0x0004,

    /// Read Word assuming that the core is accepting MicroMips instructions
    ReadWordMicromips = 0x0005,

    /// Chip erase assuming that the core is accepting classic MIPS32
    /// instructions
    EraseChip = 0x0006,

    /// Load programming executive assuming that the core is accepting classic
    /// MIPS32 instructions
    LoadExecutiveClassic = 0x0010,

    /// Load programming executive assuming that the core is accepting classic
    /// MIPS32 instructions
    LoadExecutiveMicromips = 0x0011,

    /// Perform an programming executive transaction. The length of the response
    /// is determined automatically.
    ExecutiveTransaction = 0x0012,
}

impl From<RequestCode> for u16 {
    fn from(r: RequestCode) -> Self {
        r as u16
    }
}

#[repr(u16)]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum ResponseCode {
    Ok = 0x0000,
    InvalidArgument = 0x0001,
    CodeProtected = 0x0002,
    Timeout = 0x0003,
    ExecutiveNotLoaded = 0x0004,
    /// Internal buffer overrun, e.g. due to too large PE response
    BufferOverrun = 0x0005,
}

impl From<ResponseCode> for u16 {
    fn from(r: ResponseCode) -> Self {
        r as u16
    }
}

impl From<probe::Error> for ResponseCode {
    fn from(e: probe::Error) -> Self {
        match e {
            probe::Error::FastDataTimeout => ResponseCode::Timeout,
            //_ => ResponseCode::InvalidArgument,
        }
    }
}

#[derive(PartialEq, Eq)]
enum Mode {
    Unknown,
    SerialExecution,
    ProgrammingExecutive,
}

pub struct Pic32Adapter<C: Pic32Comm> {
    comm: C,
    mode: Mode,
}

impl<C: Pic32Comm> Pic32Adapter<C> {
    pub fn new(comm: C) -> Pic32Adapter<C> {
        let mode = Mode::Unknown;
        Pic32Adapter { comm, mode }
    }

    /// Enter serial execution mode if not already entered
    /// Returns a ResponseCode in case of failure due to code protection
    fn serial_execution(&mut self) -> Result<(), ResponseCode> {
        if self.mode != Mode::SerialExecution {
            self.comm.send_command(JtagCommand::MtapSwMtap);
            self.comm.send_command(JtagCommand::MtapCommand);
            let status = self.comm.xfer_data_u8(MtapCommand::Status.into());
            if status & mtap_status::CPS == 0 {
                return Err(ResponseCode::CodeProtected);
            }
            self.comm.xfer_data_u8(MtapCommand::AssertReset.into());
            self.comm.send_command(JtagCommand::MtapSwEtap);
            self.comm.send_command(JtagCommand::EtapEjtagBoot);
            self.comm.send_command(JtagCommand::MtapSwMtap);
            self.comm.send_command(JtagCommand::MtapCommand);
            self.comm.xfer_data_u8(MtapCommand::DeassertReset.into());
            self.comm.xfer_data_u8(MtapCommand::FlashEnable.into());
            self.comm.send_command(JtagCommand::MtapSwEtap);
            self.mode = Mode::SerialExecution;
        }
        Ok(())
    }

    /// Read word by serial command execution (classic MIPS32r2 ISA)
    /// Returns Err is case of timeout or code protection
    fn read_word_classic(&mut self, address: u32) -> Result<u32, ResponseCode> {
        let addr_lo = address & 0x0000ffff;
        let addr_hi = address >> 16;
        self.serial_execution()?;
        //self.comm.send_command(JtagCommand::MtapSwEtap);
        //self.comm.run_test_idle();
        self.comm.xfer_instruction(0x3c13ff20)?; // lui s3, 0xFF20
        self.comm.xfer_instruction(0x3c080000 | addr_hi)?; // lui t0, addr_hi
        self.comm.xfer_instruction(0x35080000 | addr_lo)?; // ori t0, addr_lo
        self.comm.xfer_instruction(0x8d090000)?; // lw t1, 0(t0)
        self.comm.xfer_instruction(0xae690000)?; // sw t1, 0(s3)
        self.comm.xfer_instruction(0)?; // nop
        self.comm.send_command(JtagCommand::EtapFastData);
        let word = self.comm.xfer_fastdata(0)?;
        Ok(word)
    }

    /// Read word by serial command execution (MicroMIPS)
    /// Returns Err is case of timeout or code protection
    fn read_word_micromips(&mut self, address: u32) -> Result<u32, ResponseCode> {
        let addr_lo = address << 16;
        let addr_hi = address & 0xffff0000;
        self.serial_execution()?;
        self.comm.xfer_instruction(0xff2041b3)?; // lui s3, 0xFF20
        self.comm.xfer_instruction(0x000041a8 | addr_hi)?; // lui t0, addr_hi
        self.comm.xfer_instruction(0x00005108 | addr_lo)?; // ori t0, addr_lo
        self.comm.xfer_instruction(0x0000fd28)?; // lw t1, 0(t0)
        self.comm.xfer_instruction(0x0000f933)?; // sw t1, 0(s3)
        self.comm.xfer_instruction(0x00000000)?; // nop
        self.comm.send_command(JtagCommand::EtapFastData);
        let word = self.comm.xfer_fastdata(0)?;
        Ok(word)
    }

    /// get response from programming executive
    ///
    /// Returns an error in case of timeout.
    /// Returns Ok(None) in case of read access without completing that read access
    fn get_pe_response(&mut self) -> Result<Option<u32>, ResponseCode> {
        assert!(self.mode == Mode::ProgrammingExecutive);

        // Wait for processor access
        self.comm.send_command(JtagCommand::EtapControl);
        let timeout = Instant::now() + Duration::from_millis(1000);
        loop {
            let control = self.comm.xfer_data_u32(
                ejtag_control::PRACC | ejtag_control::PROBEN | ejtag_control::PROBTRAP,
            );
            if control & ejtag_control::PRACC != 0 {
                if control & ejtag_control::PRNW == 0 {
                    return Ok(None);
                }
                break;
            }
            if Instant::now() >= timeout {
                return Err(ResponseCode::Timeout);
            }
        }

        // Select data register and receive PE response
        self.comm.send_command(JtagCommand::EtapData);
        let response = self.comm.xfer_data_u32(0);

        // Clear PrAcc bit to finalize processor access
        self.comm.send_command(JtagCommand::EtapControl);
        self.comm
            .xfer_data_u32(ejtag_control::PROBEN | ejtag_control::PROBTRAP);

        Ok(Some(response))
    }

    /// Load and start programming executive `pe` (classic MIPS32 ISA)
    ///
    /// The instruction words of the programming executive must be stored in
    /// `pe` in little endian byte order and `pe.len() % 4 == 0` must hold.
    /// Returns the 16 bit version number of the programming executive.
    fn load_programming_executive_classic(&mut self, pe: &[u8]) -> Result<u16, ResponseCode> {
        self.serial_execution()?;
        if pe.len() % 4 != 0 {
            return Err(ResponseCode::InvalidArgument);
        }
        debug!("loading classic PE, length = {}", pe.len());

        /* Step 1. */
        self.comm.xfer_instruction(0x3c04bf88)?; // lui a0, 0xbf88
        self.comm.xfer_instruction(0x34842000)?; // ori a0, 0x2000 - address of BMXCON
        self.comm.xfer_instruction(0x3c05001f)?; // lui a1, 0x1f
        self.comm.xfer_instruction(0x34a50040)?; // ori a1, 0x40   - a1 has 001f0040
        self.comm.xfer_instruction(0xac850000)?; // sw  a1, 0(a0)  - BMXCON initialized

        /* Step 2. */
        self.comm.xfer_instruction(0x34050800)?; // li  a1, 0x800  - a1 has 00000800
        self.comm.xfer_instruction(0xac850010)?; // sw  a1, 16(a0) - BMXDKPBA initialized

        /* Step 3. */
        self.comm.xfer_instruction(0x8c850040)?; // lw  a1, 64(a0) - load BMXDMSZ
        self.comm.xfer_instruction(0xac850020)?; // sw  a1, 32(a0) - BMXDUDBA initialized
        self.comm.xfer_instruction(0xac850030)?; // sw  a1, 48(a0) - BMXDUPBA initialized

        /* Step 4. */
        self.comm.xfer_instruction(0x3c04a000)?; // lui a0, 0xa000
        self.comm.xfer_instruction(0x34840800)?; // ori a0, 0x800  - a0 has a0000800

        /* Download the PE loader. */
        for instruction in PIC32_PE_LOADER {
            /* Step 5. */
            let opcode1 = 0x3c060000 | (instruction >> 16); // PE loader high
            let opcode2 = 0x34c60000 | (instruction & 0xffff); // PE loader low

            self.comm.xfer_instruction(opcode1)?; // lui a2, PE_loader_hi++
            self.comm.xfer_instruction(opcode2)?; // ori a2, PE_loader_lo++
            self.comm.xfer_instruction(0xac860000)?; // sw  a2, 0(a0)
            self.comm.xfer_instruction(0x24840004)?; // addiu a0, 4
        }

        /* Jump to PE loader (step 6). */
        self.comm.xfer_instruction(0x3c19a000)?; // lui t9, 0xa000
        self.comm.xfer_instruction(0x37390800)?; // ori t9, 0x800  - t9 has a0000800
        self.comm.xfer_instruction(0x03200008)?; // jr  t9
        self.comm.xfer_instruction(0x00000000)?; // nop

        // Send parameters for the loader (step 7-A).
        self.comm.send_command(JtagCommand::EtapFastData);
        self.comm.xfer_fastdata(0xa000_0900)?; // PE_ADDRESS
        self.comm.xfer_fastdata(pe.len() as u32 / 4)?; // PE_SIZE (32 bit words)

        /* Download the PE itself (step 7-B). */
        let mut pe_iter = pe.iter();
        'outer: loop {
            let mut word = 0;
            for shift in [0, 8, 16, 24] {
                if let Some(byte) = pe_iter.next() {
                    word |= (*byte as u32) << shift;
                } else {
                    break 'outer;
                }
            }
            self.comm.xfer_fastdata(word)?;
        }

        /* Download the PE instructions. */
        self.comm.xfer_fastdata(0)?; /* Step 8 - jump to PE. */
        self.comm.xfer_fastdata(0xdead0000)?;

        self.mode = Mode::ProgrammingExecutive;
        const PE_EXEC_VERSION: u32 = 0x7;
        self.comm.xfer_fastdata(PE_EXEC_VERSION << 16)?;
        let version = self.get_pe_response()?.unwrap_or(0) as u16;
        debug!("PE version = {version:08x}");
        while self.get_pe_response()?.is_some() {
            debug!("skipping response word");
        }
        Ok(version)
    }

    /// Load and start programming executive `pe` (Micromips)
    ///
    /// The instruction words of the programming executive must be stored in
    /// `pe` in little endian byte order and `pe.len() % 4 == 0` must hold.
    /// Returns the 16 bit version number of the programming executive.
    fn load_programming_executive_micromips(&mut self, pe: &[u8]) -> Result<u16, ResponseCode> {
        self.serial_execution()?;
        if pe.len() % 4 != 0 {
            return Err(ResponseCode::InvalidArgument);
        }
        debug!("loading MicroMIPS PE, length = {}", pe.len());

        /* Step 1 */
        self.comm.xfer_instruction(0xa00041a4)?; // lui a0, 0xa000
        self.comm.xfer_instruction(0x02005084)?; // ori a0, 0x0800 - address of BMXCON

        /* Download the PE loader (Step2) */
        for opcode in PIC32_PEMM_LOADER {
            /* Step 5. */
            let opcode1 = 0x41a6 | (opcode & 0xffff0000); /* hi */
            let opcode2 = 0x50c6 | (opcode << 16); /* lo */
            self.comm.xfer_instruction(opcode1)?; // lui a2, PE_loader_hi++
            self.comm.xfer_instruction(opcode2)?; // ori a2, PE_loader_lo++
            self.comm.xfer_instruction(0x6e42eb40)?; // sw  a2, 0(a0)
                                                     // addiu a0, 4
        }

        /* Jump to PE loader (step 3) */
        self.comm.xfer_instruction(0xa00041b9)?;
        self.comm.xfer_instruction(0x02015339)?;
        self.comm.xfer_instruction(0x0c004599)?;
        self.comm.xfer_instruction(0x0c000c00)?;
        self.comm.xfer_instruction(0x0c000c00)?;

        /* Load PE (step 4) */
        /* Switch from serial to fast execution mode. */
        self.comm.send_command(JtagCommand::EtapFastData);

        // Send parameters for the loader (step 7-A).
        self.comm.xfer_fastdata(0xa0000300)?; // PE_ADDRESS
        self.comm.xfer_fastdata(pe.len() as u32 / 4)?; // PE_SIZE (32 bit words)

        /* Download the PE itself (step 7-B). */
        let mut pe_iter = pe.iter();
        'outer: loop {
            let mut word = 0;
            for shift in [0, 8, 16, 24] {
                if let Some(byte) = pe_iter.next() {
                    word |= (*byte as u32) << shift;
                } else {
                    break 'outer;
                }
            }
            self.comm.xfer_fastdata(word)?;
        }

        /* Download the PE instructions. */
        self.comm.xfer_fastdata(0)?; /* Step 5 - jump to PE. */
        self.comm.xfer_fastdata(0xdead0000)?;

        self.mode = Mode::ProgrammingExecutive;
        const PE_EXEC_VERSION: u32 = 0x7;
        self.comm.xfer_fastdata(PE_EXEC_VERSION << 16)?;
        let version = self.get_pe_response()?.unwrap_or(0) as u16;
        debug!("PE version = {version:08x}");
        while self.get_pe_response()?.is_some() {
            debug!("skipping response word");
        }
        Ok(version)
    }

    /// Perform a Programming Executive (PE) transaction
    ///
    /// The end of a PE response is detected when the PE performs a read access
    /// to the dmseg. Returns the length of the PE response in bytes. The
    /// response buffer must be large enough to hold the PE response.
    fn pe_transaction(
        &mut self,
        command: &[u8],
        response: &mut [u8],
    ) -> Result<usize, ResponseCode> {
        if command.len() % 4 != 0 || response.len() % 4 != 0 {
            return Err(ResponseCode::InvalidArgument);
        }
        if self.mode != Mode::ProgrammingExecutive {
            return Err(ResponseCode::ExecutiveNotLoaded);
        }
        debug!("PE transaction: in: {} bytes", command.len());
        //debug!("PE in: {:?}", command);
        self.comm.run_test_idle();
        self.comm.send_command(JtagCommand::EtapFastData);
        let mut cmd_iter = command.iter();
        'download: loop {
            let mut word = 0;
            for shift in [0, 8, 16, 24] {
                if let Some(byte) = cmd_iter.next() {
                    word |= (*byte as u32) << shift;
                } else {
                    break 'download;
                }
            }
            self.comm.xfer_fastdata(word)?;
        }
        // PIC32MM00xxGPL quirk: switch to Microchip TAP and then back to MIPS TAP,
        // cf. PIC32MM Families Flash Programming Specification, section 18.3.
        // Only needed for programming certain configuration words but we do after every
        // PE transaction.
        self.comm.send_command(JtagCommand::MtapSwMtap);
        SystemTimer::wait(Duration::from_micros(400));
        self.comm.send_command(JtagCommand::MtapSwEtap);

        let mut resp_iter = response.iter_mut();
        let mut resp_len = 0;
        let mut overrun = false;
        while let Some(word) = self.get_pe_response()? {
            for shift in [0, 8, 16, 24] {
                let byte = (word >> shift) as u8;
                if let Some(resp_byte) = resp_iter.next() {
                    *resp_byte = byte;
                    resp_len += 1;
                } else {
                    overrun = true;
                }
            }
        }
        if overrun {
            return Err(ResponseCode::BufferOverrun);
        }
        Ok(resp_len)
    }

    /// process a request message and create a response message
    /// Returns the length of the response message
    /// Panics if response is too short
    pub fn process(&mut self, request: &[u8], response: &mut [u8]) -> usize {
        debug!("process: request, len = {}", request.len());

        fn response_header(response: &mut [u8], code: ResponseCode, len: u16) -> usize {
            LE::write_u16(&mut response[0..], code.into());
            LE::write_u16(&mut response[2..], len);
            4
        }
        // first sanity check of length
        if request.len() < 4 || request.len() - 4 > u16::MAX as usize {
            return response_header(&mut response[0..], ResponseCode::InvalidArgument, 0);
        }
        let request_code = LE::read_u16(&request[0..]);
        let request_len = LE::read_u16(&request[2..]);
        debug!("code = {request_code:04x}, length = {request_len:04x}");
        // second sanity check of length
        if request_len as usize != request.len() - 4 {
            return response_header(&mut response[0..], ResponseCode::InvalidArgument, 0);
        }
        if request_code == RequestCode::ConnectReset.into() {
            if request_len != 4 {
                return response_header(&mut response[0..], ResponseCode::InvalidArgument, 0);
            }
            let bitrate = LE::read_u32(&request[4..]);
            debug!("ConnectReset(bitrate = {bitrate})");
            self.comm.iscp_connect_key_sequence(if bitrate != 0 {
                bitrate
            } else {
                10_000_000 // default: 10 MBit/s
            });
            LE::write_u16(&mut response[0..], ResponseCode::Ok.into());
            LE::write_u16(&mut response[2..], 0);
            self.mode = Mode::Unknown;
            4
        } else if request_code == RequestCode::GetIdCode.into() {
            if request_len != 0 {
                return response_header(&mut response[0..], ResponseCode::InvalidArgument, 0);
            }
            debug!("GetIdCode");
            self.comm.run_test_idle();
            self.comm.send_command(JtagCommand::MtapSwMtap);
            self.comm.run_test_idle();
            self.comm.send_command(JtagCommand::IdCode);
            let id_code = self.comm.xfer_data_u32(0);
            LE::write_u16(&mut response[0..], ResponseCode::Ok.into());
            LE::write_u16(&mut response[2..], 4);
            LE::write_u32(&mut response[4..], id_code);
            8
        } else if request_code == RequestCode::ReadWordClassic.into()
            || request_code == RequestCode::ReadWordMicromips.into()
        {
            if request_len != 4 || request.len() != 8 {
                return response_header(&mut response[0..], ResponseCode::InvalidArgument, 0);
            }
            let address = LE::read_u32(&request[4..]);
            let result = if request_code == RequestCode::ReadWordClassic.into() {
                self.read_word_classic(address)
            } else {
                debug!("read_word_micromips({address:08x})");
                self.read_word_micromips(address)
            };
            match result {
                Ok(word) => {
                    response_header(&mut response[0..], ResponseCode::Ok, 4);
                    LE::write_u32(&mut response[4..], word);
                    8
                }
                Err(code) => response_header(&mut response[0..], code, 0),
            }
        } else if request_code == RequestCode::EraseChip.into() {
            if request_len != 0 || request.len() != 4 {
                return response_header(&mut response[0..], ResponseCode::InvalidArgument, 0);
            }
            self.comm.send_command(JtagCommand::MtapSwMtap);
            self.comm.send_command(JtagCommand::MtapCommand);
            self.comm.xfer_data_u8(MtapCommand::Erase.into());
            self.comm.xfer_data_u8(MtapCommand::DeassertReset.into());
            let mut ctr = 1000;
            loop {
                SystemTimer::wait(Duration::from_millis(10));
                let status = self.comm.xfer_data_u8(MtapCommand::Status.into());
                if status & mtap_status::FCBUSY == 0 && status & mtap_status::CFGRDY != 0 {
                    break;
                }
                ctr -= 1;
                if ctr == 0 {
                    return response_header(&mut response[0..], ResponseCode::Timeout, 0);
                }
            }
            response_header(&mut response[0..], ResponseCode::Ok, 0)
        } else if request_code == RequestCode::LoadExecutiveClassic.into() {
            match self.load_programming_executive_classic(&request[4..]) {
                Ok(version) => {
                    response_header(&mut response[0..], ResponseCode::Ok, 2);
                    LE::write_u16(&mut response[4..], version);
                    6
                }
                Err(code) => response_header(&mut response[0..], code, 0),
            }
        } else if request_code == RequestCode::LoadExecutiveMicromips.into() {
            match self.load_programming_executive_micromips(&request[4..]) {
                Ok(version) => {
                    response_header(&mut response[0..], ResponseCode::Ok, 2);
                    LE::write_u16(&mut response[4..], version);
                    6
                }
                Err(code) => response_header(&mut response[0..], code, 0),
            }
        } else if request_code == RequestCode::ExecutiveTransaction.into() {
            match self.pe_transaction(&request[4..], &mut response[4..]) {
                Ok(len) => {
                    response_header(response, ResponseCode::Ok, len as u16);
                    4 + len
                }
                Err(code) => response_header(response, code, 0),
            }
        } else if request_code == RequestCode::Disconnect.into() {
            if request_len != 0 {
                return response_header(&mut response[0..], ResponseCode::InvalidArgument, 0);
            }
            debug!("Disconnect");
            self.comm.disconnect();
            self.mode = Mode::Unknown;
            LE::write_u16(&mut response[0..], ResponseCode::Ok.into());
            LE::write_u16(&mut response[2..], 0);
            4
        } else {
            response_header(&mut response[0..], ResponseCode::InvalidArgument, 0)
        }
    }
}

static PIC32_PE_LOADER: [u32; 21] = [
    0x3c07dead, 0x3c06ff20, 0x3c05ff20, 0x8cc40000, 0x8cc30000, 0x1067000b, 0x00000000, 0x1060fffb,
    0x00000000, 0x8ca20000, 0x2463ffff, 0xac820000, 0x24840004, 0x1460fffb, 0x00000000, 0x1000fff3,
    0x00000000, 0x3c02a000, 0x34420900, 0x00400008, 0x00000000,
];

static PIC32_PEMM_LOADER: [u32; 14] = [
    0xdead41a7,
    0xff2041a6,
    0xff2041a5,
    0x69e06a60,
    0x000c94e3,
    0x8dfa0c00,
    0x69500c00,	// not shown in the OpCode column instruction
    0xe9406dbe,
    0xadfb6e42,
    0xcff20c00,
    0x0c000c00,
    0xa00041a2,
    0x03015042, // Changed from 0x0901 in documentation to 0x0301 in .hex
    0x0c004582,
];
