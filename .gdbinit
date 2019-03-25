set architecture arm

target remote :3333

break rust_begin_unwind

set print asm-demangle on

monitor arm semihosting enable

file target/thumbv7m-none-eabi/release/stm32-black-pill-rust

load
