#!/usr/bin/sh

ELF=target/thumbv6m-none-eabi/debug/pic32probe

cargo build || exit

elf2uf2-rs $ELF
picotool load -x $ELF.uf2
sleep 2
usb-logread
