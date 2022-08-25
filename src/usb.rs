//! Pic32Probe USB Connectivity
//!
// Copyright (C) 2022 Stephan <kiffie@mailbox.org>
// SPDX-License-Identifier: GPL-2.0-or-later

use heapless::Vec;
use log::debug;
use usb_device::{class_prelude::*, Result};

/// request buffer size
const REQUEST_BUF_CAPACITY: usize = 10240;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum RxState {
    Idle,
    Receiving,
    ReceiveComplete,
}

/// USB Class for the Pic32Probe
pub struct ProbeClass<'a, B: UsbBus> {
    iface: InterfaceNumber,
    iface_string: StringIndex,
    ep_in: EndpointIn<'a, B>,
    ep_out: EndpointOut<'a, B>,
    request: Vec<u8, REQUEST_BUF_CAPACITY>,
    rx_state: RxState,
}

impl<'a, B: UsbBus> ProbeClass<'a, B> {
    pub fn new(alloc: &'a UsbBusAllocator<B>) -> ProbeClass<'a, B> {
        let iface = alloc.interface();
        let iface_string = alloc.string();
        let ep_in = alloc.bulk(64);
        let ep_out = alloc.bulk(64);
        let request = Vec::new();
        let rx_state = RxState::Idle;
        ProbeClass {
            iface,
            iface_string,
            ep_in,
            ep_out,
            request,
            rx_state,
        }
    }

    pub fn receive(&mut self) -> Option<&[u8]> {
        if self.rx_state == RxState::ReceiveComplete {
            self.rx_state = RxState::Idle;
            Some(self.request.as_slice())
        } else {
            None
        }
    }

    pub fn transmit(&mut self, data: &[u8]) -> Result<usize> {
        debug!("transmit {} octets", data.len());
        self.ep_in.write(data)
    }
}

impl<B: UsbBus> UsbClass<B> for ProbeClass<'_, B> {
    fn get_configuration_descriptors(
        &self,
        writer: &mut DescriptorWriter,
    ) -> usb_device::Result<()> {
        writer.interface_alt(self.iface, 0, 0xff, 0, 0, Some(self.iface_string))?;
        writer.endpoint(&self.ep_in)?;
        writer.endpoint(&self.ep_out)
    }

    fn get_string(&self, index: StringIndex, _lang_id: u16) -> Option<&str> {
        if index == self.iface_string {
            Some("pic32probe")
        } else {
            None
        }
    }

    fn endpoint_out(&mut self, addr: EndpointAddress) {
        if addr != self.ep_out.address() {
            return;
        }
        let mut buf = [0u8; 64];
        let len = self.ep_out.read(&mut buf).unwrap_or(0);
        //debug!("read {} octets: {:?}", len, &buf[..len]);
        if self.rx_state == RxState::Idle {
            self.request.clear();
        }
        // truncate messages that are too long to fit into the heapless::Vec
        self.request.extend_from_slice(&buf[..len]).ok();
        self.rx_state = if len == 64 {
            RxState::Receiving
        } else {
            RxState::ReceiveComplete
        };
    }
}
