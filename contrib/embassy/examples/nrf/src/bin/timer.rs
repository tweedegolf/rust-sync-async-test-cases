#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

#[path = "../example_common.rs"]
mod example_common;
use embassy_nrf::Peripherals;
use example_common::*;

use embassy::executor::Spawner;
use embassy::time::{Duration, Timer};

#[embassy::task]
async fn run1() {
    loop {
        info!("BIG INFREQUENT TICK");
        Timer::after(Duration::from_ticks(64000)).await;
    }
}

#[embassy::task]
async fn run2() {
    loop {
        info!("tick");
        Timer::after(Duration::from_ticks(13000)).await;
    }
}

#[embassy::main]
async fn main(spawner: Spawner, _p: Peripherals) {
    unwrap!(spawner.spawn(run1()));
    unwrap!(spawner.spawn(run2()));
}
