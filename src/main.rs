#![no_main]
#![no_std]
#![allow(unused_imports)]

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
use crate::hal::device::SPI1;
use crate::hal::spi::Spi;
use crate::robot::init_peripherals;
use crate::robot::Robot;
use crate::robot::SpiPins;
use cortex_m::asm;
use embedded_hal::spi::FullDuplex;
use heapless::consts::U2048;
use librobot::transmission::io::BuzzerState;
use librobot::transmission::io::Pneumatic;
use librobot::transmission::{
    eth::{init_eth, SOCKET_UDP},
    io::{IOState, TriggerState, IO},
    Jsonizable,
};
use pwm_speaker::songs::*;

fn send_tirette_state<T, K>(
    robot: &mut Robot,
    spi: &mut Spi<T, K>,
    eth: &mut W5500,
    buzzer_state: &BuzzerState,
    ip: &IpAddress,
    port: u16,
) where
    Spi<T, K>: FullDuplex<u8>,
{
    let tirette = if robot.tirette.is_low() {
        TriggerState::Waiting
    } else {
        TriggerState::Triggered
    };

    let state = IO {
        buzzer: *buzzer_state,
        tirette,
    };

    if let Ok(data) = state.to_string::<U2048>() {
        let mut new_data: [u8; 2049] = [0u8; 2049];
        new_data[0] = 4;
        for (i, b) in data.as_bytes().iter().enumerate() {
            new_data[i + 1] = *b;
        }
        robot.led_feedback.set_low();
        if let Ok(_) = eth.send_udp(spi, SOCKET_UDP, 51, ip, port, &new_data[0..data.len() + 1]) {
            robot.led_feedback.set_low();
        } else {
            robot.led_feedback.set_high();
        }
    } else {
        robot.led_feedback.set_high();
    }
}

fn send_pneumatic_state<T, K>(
    robot: &mut Robot,
    spi: &mut Spi<T, K>,
    eth: &mut W5500,
    ip: &IpAddress,
    port: u16,
) where
    Spi<T, K>: FullDuplex<u8>,
{
    let pumps = [
        if robot.pumps.0.is_set_high() {
            IOState::On
        } else {
            IOState::Off
        },
        if robot.pumps.0.is_set_high() {
            IOState::On
        } else {
            IOState::Off
        },
    ];

    let mut valves = [IOState::Off; 6];
    for (state, valve) in robot.valves.iter().zip(valves.iter_mut()) {
        *valve = if state.is_set_high() {
            IOState::On
        } else {
            IOState::Off
        };
    }

    let state = Pneumatic {
        pump: pumps,
        pump_intensity: [0, 0],
        valves,
    };

    if let Ok(data) = state.to_string::<U2048>() {
        robot.led_feedback.set_low();
        let mut new_data: [u8; 2049] = [0u8; 2049];
        new_data[0] = 5;
        for (i, b) in data.as_bytes().iter().enumerate() {
            new_data[i + 1] = *b;
        }
        if let Ok(_) = eth.send_udp(spi, SOCKET_UDP, 51, ip, port, &new_data[0..data.len() + 1]) {
            robot.led_feedback.set_low();
        } else {
            robot.led_feedback.set_high();
        }
    } else {
        robot.led_feedback.set_high();
    }
}

#[entry]
fn main() -> ! {
    let chip = Peripherals::take().unwrap();
    let cortex = CortexPeripherals::take().unwrap();
    let (mut robot, mut spi, mut cs): (Robot, Spi<SPI1, SpiPins>, _) =
        init_peripherals(chip, cortex);
    let mut eth = { W5500::new(&mut spi, &mut cs) };
    {
        init_eth(
            &mut eth,
            &mut spi,
            &MacAddress::new(0x02, 0x01, 0x02, 0x03, 0x04, 0x55),
            &IpAddress::new(192, 168, 1, 4),
        );
    }
    let mut buffer = [0u8; 2048];

    let mut buzzer_state = BuzzerState::Rest;

    let mut tirette_already_detected = false;

    loop {
        if robot.tirette.is_low() && !tirette_already_detected {
            tirette_already_detected = true;
            send_tirette_state(
                &mut robot,
                &mut spi,
                &mut eth,
                &buzzer_state,
                &IpAddress::new(192, 168, 1, 254),
                5004,
            )
        } else if robot.tirette.is_high() && tirette_already_detected {
            tirette_already_detected = false;
            send_tirette_state(
                &mut robot,
                &mut spi,
                &mut eth,
                &buzzer_state,
                &IpAddress::new(192, 168, 1, 254),
                5004,
            )
        }

        if let Ok(Some((ip, port, size))) = eth.try_receive_udp(&mut spi, SOCKET_UDP, &mut buffer) {
            let id = buffer[0];

            match id {
                4 => {
                    use BuzzerState::*;
                    match IO::from_json_slice(&buffer[1..size]) {
                        Ok(io) => {
                            robot.led_communication.set_low();
                            match (io.buzzer, buzzer_state) {
                                (PlayErrorSound, Rest) => {
                                    robot.speaker.play_score(&LAVENTURIER, &mut robot.delay);
                                    buzzer_state = PlayErrorSound;
                                }
                                (PlaySuccessSound, Rest) => {
                                    robot
                                        .speaker
                                        .play_score(&MARIO_THEME_INTRO, &mut robot.delay);
                                    buzzer_state = PlaySuccessSound;
                                }

                                (Rest, _) => {
                                    buzzer_state = Rest;
                                }

                                _ => {}
                            }
                            send_tirette_state(
                                &mut robot,
                                &mut spi,
                                &mut eth,
                                &mut buzzer_state,
                                &ip,
                                port,
                            );
                        }
                        Err(e) => {
                            robot.led_communication.set_high();
                            panic!("{:#?}", e)
                        }
                    }
                }
                5 => {
                    match Pneumatic::from_json_slice(&buffer[1..size]) {
                        Ok(pneumatic) => {
                            robot.led_communication.set_low();

                            // Gestion des vannes
                            for (state, valve) in
                                pneumatic.valves.iter().zip(robot.valves.iter_mut())
                            {
                                match state {
                                    IOState::On => valve.set_high(),
                                    IOState::Off => valve.set_low(),
                                }
                            }

                            // Gestion des pompes
                            match pneumatic.pump[0] {
                                IOState::On => robot.pumps.0.set_high(),
                                IOState::Off => robot.pumps.0.set_low(),
                            }

                            match pneumatic.pump[1] {
                                IOState::On => robot.pumps.1.set_high(),
                                IOState::Off => robot.pumps.1.set_low(),
                            }
                            send_pneumatic_state(
                                &mut robot,
                                &mut spi,
                                &mut eth,
                                &IpAddress::new(192, 168, 1, 254),
                                5005,
                            );
                        }
                        Err(e) => {
                            robot.led_communication.set_high();
                            panic!("{:#?}", e)
                        }
                    }
                }
                _ => {}
            }
        }
    }
}
