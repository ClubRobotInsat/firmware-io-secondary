//! Blink the LED (connected to Pin PC 13) on and off with 1 second interval.

#![allow(unused_imports)]
#![no_main] //  Don't use the Rust standard bootstrap. We will provide our own.
#![no_std] //  Don't use the Rust standard library. We are building a binary that can run on its own.
#[macro_use]
extern crate cortex_m; //  Low-level functions for ARM Cortex-M3 processor in STM32 Blue Pill.
extern crate arrayvec;
extern crate cortex_m_rt; //  Startup and runtime functions for ARM Cortex-M3.
extern crate cortex_m_semihosting; //  Debug console functions for ARM Cortex-M3.
extern crate embedded_hal;
extern crate librobot;
extern crate nb;
extern crate numtoa;
extern crate panic_semihosting; //  Panic reporting functions, which transmit to the debug console.
extern crate pid_control;
extern crate qei;

extern crate stm32f1xx_hal as bluepill_hal; //  Hardware Abstraction Layer (HAL) for STM32 Blue Pill.

use stm32f1xx_hal::stm32f1::stm32f103::interrupt;

use crate::bluepill_hal::stm32 as f103;

use cortex_m_rt::{entry, exception};

use core::fmt::Write;

use cortex_m::Peripherals as CortexPeripherals;

use crate::bluepill_hal::delay::Delay;
use crate::bluepill_hal::prelude::*; //  Define HAL traits.
use crate::bluepill_hal::pwm_input::*;
use crate::bluepill_hal::qei::Qei;
use crate::bluepill_hal::serial::Serial;
use crate::bluepill_hal::time::Hertz;
use crate::f103::Peripherals;

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

    //cortex.NVIC.enable(stm32f1xx_hal::stm32::Interrupt::TIM3);

    // Config des horloges
    let mut rcc = bluepill.RCC.constrain();
    let mut flash = bluepill.FLASH.constrain();
    let mut afio = bluepill.AFIO.constrain(&mut rcc.apb2);

    let clocks = rcc
        .cfgr
        .use_hse(8.mhz())
        .sysclk(72.mhz())
        .pclk1(36.mhz())
        .pclk2(72.mhz())
        .freeze(&mut flash.acr);

    let _delay = Delay::new(cortex.SYST, clocks);
    // Config du GPIO
    let mut gpiob = bluepill.GPIOB.split(&mut rcc.apb2);
    let _gpioa = bluepill.GPIOA.split(&mut rcc.apb2);

    let pb10 = gpiob.pb10.into_alternate_push_pull(&mut gpiob.crh);
    let pb11 = gpiob.pb11.into_floating_input(&mut gpiob.crh);
    let _ = gpiob.pb13.into_push_pull_output(&mut gpiob.crh);
    let _ = gpiob.pb14.into_push_pull_output(&mut gpiob.crh);

    let pb4 = gpiob.pb4.into_alternate_open_drain(&mut gpiob.crl);
    let pb5 = gpiob.pb5.into_alternate_open_drain(&mut gpiob.crl);

    let timer = bluepill.TIM3.pwm_input(
        (pb4, pb5),
        &mut rcc.apb1,
        &mut afio.mapr,
        &mut dbg,
        &clocks,
        Configuration::Frequency(10.khz()),
    );

    let serial = Serial::usart3(
        bluepill.USART3,
        (pb10, pb11),
        &mut afio.mapr,
        115_200.bps(),
        clocks,
        &mut rcc.apb1,
    );

    let (mut ser, _) = serial.split();

    let mut buffer_freq = [0u8; 64];
    let mut buffer_duty = [0u8; 64];

    let str1 = b"Freq : ";
    let sep = b" | ";
    let str2 = b"Duty : ";
    let end = b"\n";

    loop {
        let (duty, period) = timer.read_duty(ReadMode::Instant).unwrap();
        let freq = timer.read_frequency(ReadMode::Instant, &clocks).unwrap().0;
        let percent = 100.0 * (duty as f32 / period as f32);
        let duty_indices = (percent as u16).numtoa(10, &mut buffer_duty);
        let freq_indices = freq.numtoa(10, &mut buffer_freq);

        for b in str1
            .iter()
            .chain(freq_indices.iter())
            .chain(sep.iter())
            .chain(str2.iter())
            .chain(duty_indices.iter())
            .chain(end.iter())
        {
            ser.write(*b).expect("Failed to send data");
        }
    }
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
