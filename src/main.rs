//! Blink the LED (connected to Pin PC 13) on and off with 1 second interval.

#![allow(unused_imports)]
#![no_main] //  Don't use the Rust standard bootstrap. We will provide our own.
#![no_std] //  Don't use the Rust standard library. We are building a binary that can run on its own.

use stm32f1xx_hal as hal; //  Hardware Abstraction Layer (HAL) for STM32 Blue Pill.

use stm32f1xx_hal::stm32f1::stm32f103::interrupt;

use crate::hal::stm32 as f103;

use cortex_m_rt::{entry, exception};

use nb::block;

use core::fmt::Write;

use cortex_m::Peripherals as CortexPeripherals;

use panic_semihosting;

use crate::f103::Peripherals;
use crate::hal::delay::Delay;
use crate::hal::prelude::*; //  Define HAL traits.
use crate::hal::qei::Qei;
use crate::hal::serial::Serial;
use crate::hal::time::Hertz;

use cortex_m::asm;
use cortex_m_semihosting::hio; //  For displaying messages on the debug console. //  Clocks, flash memory, GPIO for the STM32 Blue Pill.

use cortex_m_rt::ExceptionFrame; //  Stack frame for exception handling.

use qei::QeiManager;

use librobot::navigation::{Motor, PIDParameters};
use librobot::units::MilliMeter;

use numtoa::NumToA;

// Black Pill starts execution at function main().
// interrupt!(TIM3, tim3);

#[entry]
fn main() -> ! {
    let bluepill = Peripherals::take().unwrap();
    let cortex = CortexPeripherals::take().unwrap();
    let _debug_out = hio::hstdout().unwrap();

    let mut dbg = bluepill.DBG;

    // Config des horloges
    let mut rcc = bluepill.RCC.constrain();
    let mut flash = bluepill.FLASH.constrain();
    let mut afio = bluepill.AFIO.constrain(&mut rcc.apb2);

    let clocks = rcc
        .cfgr /*
        .use_hse(8.mhz())
        .sysclk(72.mhz())
        .pclk1(36.mhz())
        .pclk2(72.mhz())*/
        .freeze(&mut flash.acr);

    // Config du GPIO
    let mut gpiob = bluepill.GPIOB.split(&mut rcc.apb2);

    loop {}
}

//  For any hard faults, show a message on the debug console and stop.

#[interrupt]
fn TIM3() {
    static mut HIGH_CAPTURE: bool = false;
    static mut HIGH_ARR: bool = false;
    unsafe {
        let capture = (*f103::TIM3::ptr()).sr.read().cc1if();

        if capture.bit_is_set() {
            if *HIGH_CAPTURE {
                (*f103::GPIOB::ptr()).bsrr.write(|w| w.br13().set_bit());
            } else {
                (*f103::GPIOB::ptr()).bsrr.write(|w| w.bs13().set_bit());
            }
            (*f103::TIM3::ptr()).sr.write(|w| w.cc1if().clear_bit());
            *HIGH_CAPTURE = !(*HIGH_CAPTURE);
        } else {
            if *HIGH_ARR {
                (*f103::GPIOB::ptr()).bsrr.write(|w| w.br14().set_bit());
            } else {
                (*f103::GPIOB::ptr()).bsrr.write(|w| w.bs14().set_bit());
            }
            *HIGH_ARR = !(*HIGH_ARR);
        }
        (*f103::TIM3::ptr()).sr.write(|w| w.uif().clear_bit());
    }
}

#[exception]
fn HardFault(ef: &ExceptionFrame) -> ! {
    asm::bkpt();
    panic!("Hard fault: {:#?}", ef);
}

//  For any unhandled interrupts, show a message on the debug console and stop.

#[exception]
fn DefaultHandler(irqn: i16) {
    asm::bkpt();
    panic!("Unhandled exception (IRQn = {})", irqn);
}
