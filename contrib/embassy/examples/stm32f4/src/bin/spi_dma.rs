#![no_std]
#![no_main]
#![feature(trait_alias)]
#![feature(type_alias_impl_trait)]
#![allow(incomplete_features)]

#[path = "../example_common.rs"]
mod example_common;
use core::fmt::Write;
use core::str::from_utf8;
use embassy::executor::Spawner;
use embassy_stm32::spi::{Config, Spi};
use embassy_stm32::time::Hertz;
use embassy_stm32::Peripherals;
use embassy_traits::spi::FullDuplex;
use example_common::*;
use heapless::String;

#[embassy::main]
async fn main(_spawner: Spawner, p: Peripherals) {
    info!("Hello World!");

    let mut spi = Spi::new(
        p.SPI1,
        p.PB3,
        p.PB5,
        p.PB4,
        p.DMA2_CH3,
        p.DMA2_CH2,
        Hertz(1_000_000),
        Config::default(),
    );

    for n in 0u32.. {
        let mut write: String<128> = String::new();
        let mut read = [0; 128];
        core::write!(&mut write, "Hello DMA World {}!\r\n", n).unwrap();
        spi.read_write(&mut read[0..write.len()], write.as_bytes())
            .await
            .ok();
        info!("read via spi+dma: {}", from_utf8(&read).unwrap());
    }
}
