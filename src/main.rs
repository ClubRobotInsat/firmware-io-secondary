#![no_main]
#![no_std]

use crate::f103::Peripherals;
use crate::hal::stm32 as f103;
use cortex_m::Peripherals as CortexPeripherals;
use cortex_m_rt::entry;
use embedded_hal::digital::{InputPin, OutputPin, StatefulOutputPin};
#[allow(unused_imports)]
use panic_semihosting;
use stm32f1xx_hal as hal;
use w5500::*;
mod robot;
use crate::robot::init_peripherals;
use heapless::consts::U2048;
use librobot::transmission::{
    eth::{init_eth, SOCKET_UDP},
    io::{IOState, TriggerState, IO},
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
        if let Ok(Some((ip, port, size))) =
            eth.try_receive_udp(&mut robot.spi_eth, SOCKET_UDP, &mut buffer)
        {
            let _id = buffer[0];

            match IO::from_json_slice(&buffer[1..size]) {
                Ok(io) => {
                    robot.led_communication.set_low();

                    for (state, valve) in io.valves.iter().zip(robot.valves.iter_mut()) {
                        match state {
                            IOState::On => valve.set_high(),
                            IOState::Off => valve.set_low(),
                        }
                    }

                    match io.pumps {
                        IOState::On => robot.pumps.set_high(),
                        IOState::Off => robot.pumps.set_low(),
                    }

                    let pumps = if robot.pumps.is_set_high() {
                        IOState::On
                    } else {
                        IOState::Off
                    };

                    let mut valves = [IOState::Off; 8];
                    for (state, valve) in robot.valves.iter().zip(valves.iter_mut()) {
                        *valve = if state.is_set_high() {
                            IOState::On
                        } else {
                            IOState::Off
                        };
                    }

                    let tirette = if robot.tirette.is_high() {
                        TriggerState::Waiting
                    } else {
                        TriggerState::Triggered
                    };

                    let state = IO {
                        pumps,
                        pump_intensity: 0,
                        tirette,
                        valves,
                    };

                    if let Ok(data) = state.to_string::<U2048>() {
                        robot.led_feedback.set_low();
                        if let Ok(_) = eth.send_udp(
                            &mut robot.spi_eth,
                            SOCKET_UDP,
                            50,
                            &ip,
                            port,
                            &data.as_bytes(),
                        ) {
                            robot.led_feedback.set_low();
                        } else {
                            robot.led_feedback.set_high();
                        }
                    } else {
                        robot.led_feedback.set_high();
                    }
                }
                Err(e) => {
                    robot.led_communication.set_high();
                    panic!("{:#?}", e)
                }
            }
        }
    }
}
