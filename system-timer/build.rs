//! Build script

use std::env;

fn main() {
    // set a default value for SYS_CLOCK for PIC32 if not defined
    let default_sys_clock = 40_000_000;
    if env::var("CARGO_FEATURE_PIC32").is_ok() && env::var("SYS_CLOCK").is_err() {
        println!("cargo:warning=SYS_CLOCK not set using default value {default_sys_clock}");
        println!("cargo:rustc-env=SYS_CLOCK={default_sys_clock}");
    }
}
