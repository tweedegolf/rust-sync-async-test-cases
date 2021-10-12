#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]
#![allow(incomplete_features)]

#[path = "../example_common.rs"]
mod example_common;

use defmt::*;
use embassy::executor::Spawner;
use embassy_nrf::gpio::{Input, Pull};
use embassy_nrf::gpiote::PortInput;
use embassy_nrf::wdt::{Config, Watchdog};
use embassy_nrf::Peripherals;
use embassy_traits::gpio::{WaitForHigh, WaitForLow};

#[embassy::main]
async fn main(_spawner: Spawner, p: Peripherals) {
    info!("Hello World!");

    let mut config = Config::default();
    config.timeout_ticks = 32768 * 3; // 3 seconds

    // This is needed for `probe-run` to be able to catch the panic message
    // in the WDT interrupt. The core resets 2 ticks after firing the interrupt.
    config.run_during_debug_halt = false;

    let (_wdt, [mut handle]) = match Watchdog::try_new(p.WDT, config) {
        Ok(x) => x,
        Err(_) => {
            info!("Watchdog already active with wrong config, waiting for it to timeout...");
            loop {}
        }
    };

    let mut button = PortInput::new(Input::new(p.P0_11, Pull::Up));

    info!("Watchdog started, press button 1 to pet it or I'll reset in 3 seconds!");

    loop {
        button.wait_for_high().await;
        button.wait_for_low().await;
        info!("Button pressed, petting watchdog!");
        handle.pet();
    }
}
