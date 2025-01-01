//! USB Log Reader
//!
//! Looks for device having an interface named 'kiffielog' by default and having
//! an USB bulk IN EP. Then copies all bytes from the endpoint to stdout.
//!

use clap::Parser;
use rusb::{Context, Device, Direction, UsbContext};
use std::io::Write;
use std::process::exit;
use std::time::Duration;

const TIMEOUT: Duration = Duration::from_millis(100);

#[derive(Parser)]
#[command(about = "Reads USB log channel")]
struct Args {
    /// Log interface name
    #[clap(short = 'i', long = "interface-name", default_value = "kiffielog")]
    interface: String,
}

fn check_device(dev: &Device<Context>, label: &str) -> rusb::Result<Option<(u8, u8)>> {
    let handle = match dev.open() {
        Ok(h) => h,
        Err(_) => return Ok(None),
    };

    let lang = handle.read_languages(TIMEOUT)?[0];
    let config_desc = dev.active_config_descriptor()?;
    for iface in config_desc.interfaces() {
        for iface_desc in iface.descriptors() {
            if let Ok(iface_string) = handle.read_interface_string(lang, &iface_desc, TIMEOUT) {
                if iface_string != label {
                    continue;
                }
                for ep_desc in iface_desc.endpoint_descriptors() {
                    if ep_desc.direction() == Direction::In {
                        return Ok(Some((iface_desc.interface_number(), ep_desc.address())));
                    }
                }
            }
        }
    }
    Ok(None)
}

/// Returns Vector of (Device, interface_no, endpoint_addr)
fn find_endpoints(context: &Context, label: &str) -> rusb::Result<Vec<(Device<Context>, u8, u8)>> {
    let mut res = vec![];
    let dev_list = context.devices().unwrap();
    let mut dev_iter = dev_list.iter();
    loop {
        let found_device;
        let devo = dev_iter.next();
        if let Some(ref dev) = devo {
            found_device = check_device(dev, label)?;
        } else {
            break;
        }
        if let Some((iface_id, ep_addr)) = found_device {
            res.push((devo.unwrap(), iface_id, ep_addr));
        }
    }
    Ok(res)
}

fn main() {
    let args: Args = Args::parse();
    let context = Context::new().unwrap();
    let found = find_endpoints(&context, args.interface.as_str()).unwrap();
    if found.is_empty() {
        eprintln!("no log channel interface found");
        exit(1);
    }
    if found.len() > 1 {
        println!("Warning: there are multiple log channel interfaces.");
    }
    let (dev, iface_id, ep_addr) = &found[0];
    let dev_desc = dev.device_descriptor().unwrap();
    let vid = dev_desc.vendor_id();
    let pid = dev_desc.product_id();
    let handle = dev.open().unwrap();
    handle.claim_interface(*iface_id).unwrap();

    let mut stdout = std::io::stdout();
    let bus = dev.bus_number();
    let addr = dev.address();
    println!("Reading USB log channel from device {vid:04x}:{pid:04x} on bus {bus} at address {addr}, EP 0x{ep_addr:02x}");
    loop {
        let mut buf = [0; 1024];
        match handle.read_bulk(*ep_addr, &mut buf, TIMEOUT) {
            Ok(len) => {
                stdout.write_all(&buf[..len]).unwrap();
            }
            Err(rusb::Error::Timeout) => (),
            Err(e) => {
                eprintln!("Error in Reading from USB: {e}");
                exit(1);
            }
        }
    }
}
