#![no_main]
#![no_std]

use crate::f103::Peripherals;
use crate::hal::stm32 as f103;
use cortex_m::Peripherals as CortexPeripherals;
use cortex_m_rt::entry;
#[allow(unused_imports)]
use panic_semihosting;
use stm32f1xx_hal as hal;
use w5500::*;
mod robot;
use crate::robot::init_peripherals;
use librobot::transmission::{
    eth::{init_eth, SOCKET_UDP},
    io::IO,
    Jsonizable,
};

#[entry]
fn main() -> ! {
    let chip = Peripherals::take().unwrap();
    let cortex = CortexPeripherals::take().unwrap();
    let mut robot = init_peripherals(chip, cortex);
    let mut eth = W5500::new(&mut robot.spi_eth, &mut robot.cs);
    init_eth(
        &mut eth,
        &mut robot.spi_eth,
        &MacAddress::new(0x02, 0x01, 0x02, 0x03, 0x04, 0x05),
        &IpAddress::new(192, 168, 0, 222),
    );

    let mut buffer = [0; 2048];

    loop {
        if let Some((_, _, size)) = eth
            .try_receive_udp(&mut robot.spi_eth, SOCKET_UDP, &mut buffer)
            .unwrap()
        {
            let _id = buffer[0];

            match IO::from_json_slice(&buffer[1..size]) {
                Ok(_e) => {}
                Err(e) => panic!("{:#?}", e),
            }
        }
    }
}
