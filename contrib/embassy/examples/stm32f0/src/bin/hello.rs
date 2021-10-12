#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use defmt::info;

use embassy::executor::Spawner;
use embassy::time::{Duration, Timer};
use embassy_stm32::Peripherals;

#[path = "../example_common.rs"]
mod example_common;

#[embassy::main]
async fn main(_spawner: Spawner, _p: Peripherals) -> ! {
    loop {
        Timer::after(Duration::from_secs(1)).await;
        info!("Hello");
    }
}
