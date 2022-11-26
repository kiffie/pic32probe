# Pic32Probe

<img src="https://raw.githubusercontent.com/kiffie/pic32probe/master/pinout-qtpy-rp2040.jpg" alt="pinout for RP2040 breakout board" width="400">

This is an ICSP (2-wire) dongle for flashing PIC32 microcontrollers implemented on an Raspberry Pi RP2040 microcontroller. Use a simple RP2040 breakout board to flash PIC32 devices. The target Vdd must be 3.3 V.

An USB UART is implemented to connect to a (debug) UART of the target.

Not tested under Windows.

## How to build the dongle software

Install the toolchain and `elf2uf2`.

```sh
rustup self update
rustup update stable
rustup target add thumbv6m-none-eabi
cargo install elf2uf2-rs --locked
```

Then build the RP2040 image like so.

```sh
cargo build
elf2uf2-rs target/thumbv6m-none-eabi/debug/pic32probe
```

The image `target/thumbv6m-none-eabi/debug/pic32probe.uf2` can then be flashed via the RP2040 USB bootloader.

## How to build the host software

The host software is a fork of Serge Vakulenko's pic32prog. It can be found [here](https://github.com/kiffie/pic32prog-kvh). You need to checkout the branch `pic32probe`.

Simply type `make` to build it.

Should the hidapi library not compile, try the following

```sh
cd hidapi
vi configure.ac # remove the duplicate AC_CONFIG_MACRO_DIR([m4]) at line 23
./bootstrap
cd ..
make
```
