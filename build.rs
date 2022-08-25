//! Build script

use chrono::Local;

fn main() {
    println!("cargo:rustc-env=BUILD_DATETIME={}", Local::now());
}
