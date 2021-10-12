#![no_std]
#![no_main]
#![feature(trait_alias)]
#![feature(type_alias_impl_trait)]
#![allow(incomplete_features)]

#[path = "../example_common.rs"]
mod example_common;
use embassy::executor::Spawner;
use embassy_stm32::exti::ExtiInput;
use embassy_stm32::gpio::{Input, Pull};
use embassy_stm32::Peripherals;
use embassy_traits::gpio::{WaitForFallingEdge, WaitForRisingEdge};
use example_common::*;

#[embassy::main]
async fn main(_spawner: Spawner, p: Peripherals) {
    info!("Hello World!");

    let button = Input::new(p.PA0, Pull::Up);
    let mut button = ExtiInput::new(button, p.EXTI0);

    info!("Press the USER button...");

    loop {
        button.wait_for_falling_edge().await;
        info!("Pressed!");
        button.wait_for_rising_edge().await;
        info!("Released!");
    }
}
