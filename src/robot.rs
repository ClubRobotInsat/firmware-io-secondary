use crate::f103;
use crate::f103::interrupt;
use crate::f103::Peripherals;
use crate::f103::SPI1;
use crate::hal::delay::Delay;
use crate::hal::gpio::{
    gpioa::*, gpiob::*, gpioc::*, Alternate, Floating, Input, Output, PushPull,
};
use crate::hal::prelude::*;

use crate::hal::spi::*;
use crate::hal::timer::Timer;
use crate::CortexPeripherals;
use cortex_m_rt::exception;
use cortex_m_rt::ExceptionFrame;
use stm32f1xx_hal::gpio::PullDown; //  Stack frame for exception handling.

type SpiPins = (
    PA5<Alternate<PushPull>>,
    PA6<Input<Floating>>,
    PA7<Alternate<PushPull>>,
);

pub struct Robot<K, P> {
    pub spi_eth: Spi<K, P>,
    pub delay: Delay,
    pub led_feedback: PC14<Output<PushPull>>,
    pub led_communication: PC15<Output<PushPull>>,
    pub pumps: PB0<Output<PushPull>>,
    pub valves: [PBx<Output<PushPull>>; 8],
    pub cs: PB13<Output<PushPull>>,
    pub tirette: PA0<Input<PullDown>>,
}

pub fn init_peripherals(chip: Peripherals, mut cortex: CortexPeripherals) -> Robot<SPI1, SpiPins> {
    //  Get the clocks from the STM32 Reset and Clock Control (RCC) and freeze the Flash Access Control Register (ACR).
    let _dbg = chip.DBG;
    // Config des horloges
    let mut rcc = chip.RCC.constrain();
    let mut flash = chip.FLASH.constrain();
    let mut afio = chip.AFIO.constrain(&mut rcc.apb2);

    cortex.DCB.enable_trace();
    cortex.DWT.enable_cycle_counter();

    let clocks = rcc
        .cfgr
        .use_hse(8.mhz())
        .sysclk(72.mhz())
        .pclk1(36.mhz())
        .pclk2(72.mhz())
        .freeze(&mut flash.acr);

    //  Configuration des GPIOs
    let mut gpioa = chip.GPIOA.split(&mut rcc.apb2);
    let mut gpiob = chip.GPIOB.split(&mut rcc.apb2);
    let mut gpioc = chip.GPIOC.split(&mut rcc.apb2);

    // Configuration des PINS

    // Slave select, on le fixe à un état bas (on n'en a pas besoin, une seule communication)
    let mut cs = gpiob.pb13.into_push_pull_output(&mut gpiob.crh);
    cs.set_low();

    let sclk = gpioa.pa5.into_alternate_push_pull(&mut gpioa.crl);
    let miso = gpioa.pa6.into_floating_input(&mut gpioa.crl);
    let mosi = gpioa.pa7.into_alternate_push_pull(&mut gpioa.crl);

    let vannes = [
        gpiob.pb3.into_push_pull_output(&mut gpiob.crl).downgrade(),
        gpiob.pb4.into_push_pull_output(&mut gpiob.crl).downgrade(),
        gpiob.pb5.into_push_pull_output(&mut gpiob.crl).downgrade(),
        gpiob.pb6.into_push_pull_output(&mut gpiob.crl).downgrade(),
        gpiob.pb8.into_push_pull_output(&mut gpiob.crh).downgrade(),
        gpiob.pb12.into_push_pull_output(&mut gpiob.crh).downgrade(),
        gpiob.pb14.into_push_pull_output(&mut gpiob.crh).downgrade(),
        gpiob.pb15.into_push_pull_output(&mut gpiob.crh).downgrade(),
    ];

    let pumps = gpiob.pb0.into_push_pull_output(&mut gpiob.crl);

    {
        // Hardfault LED
        let mut pin = gpiob.pb7.into_push_pull_output(&mut gpiob.crl);
        pin.set_low();
        // Blinking led
        let mut led = gpioc.pc13.into_push_pull_output(&mut gpioc.crh);
        led.set_low();
    }
    let led_feedback = gpioc.pc14.into_push_pull_output(&mut gpioc.crh);
    let led_communication = gpioc.pc15.into_push_pull_output(&mut gpioc.crh);

    let tirette = gpioa.pa0.into_pull_down_input(&mut gpioa.crl);

    let spi = Spi::spi1(
        chip.SPI1,
        (sclk, miso, mosi),
        &mut afio.mapr,
        Mode {
            polarity: Polarity::IdleLow,
            phase: Phase::CaptureOnFirstTransition,
        },
        1.mhz(),
        clocks,
        &mut rcc.apb2,
    );

    // Clignotement de la led
    let _ = Timer::tim3(chip.TIM3, 5.hz(), clocks, &mut rcc.apb1);

    //  Create a delay timer from the RCC clocks.
    let delay = Delay::new(cortex.SYST, clocks);

    Robot {
        spi_eth: spi,
        delay,
        led_feedback,
        led_communication,
        pumps,
        valves: vannes,
        cs,
        tirette,
    }
}

#[interrupt]
fn TIM3() {
    static mut TOOGLE: bool = false;
    unsafe {
        if *TOOGLE {
            (*f103::GPIOC::ptr()).bsrr.write(|w| w.br13().set_bit());
        } else {
            (*f103::GPIOC::ptr()).bsrr.write(|w| w.bs13().set_bit());
        }
        *TOOGLE = !(*TOOGLE);
        (*f103::TIM3::ptr()).sr.write(|w| w.uif().clear_bit());
    }
}

#[exception]
fn HardFault(ef: &ExceptionFrame) -> ! {
    unsafe {
        (*f103::GPIOB::ptr()).bsrr.write(|w| w.br7().set_bit());
    }
    panic!("Hard fault: {:#?}", ef);
}

//  For any unhandled interrupts, show a message on the debug console and stop.

#[exception]
fn DefaultHandler(irqn: i16) {
    unsafe {
        (*f103::GPIOB::ptr()).bsrr.write(|w| w.br7().set_bit());
    }
    panic!("Unhandled exception (IRQn = {})", irqn);
}
