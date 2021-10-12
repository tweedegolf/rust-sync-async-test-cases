#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

#[path = "../example_common.rs"]
mod example_common;

use cortex_m_rt::entry;
use embassy_stm32::dma::NoDma;
use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_stm32::spi::{Config, Spi};
use embassy_stm32::time::Hertz;
use embedded_hal::blocking::spi::Transfer;
use embedded_hal::digital::v2::OutputPin;
use example_common::*;

#[entry]
fn main() -> ! {
    info!("Hello World, dude!");

    let p = embassy_stm32::init(Default::default());

    let mut spi = Spi::new(
        p.SPI3,
        p.PC10,
        p.PC12,
        p.PC11,
        NoDma,
        NoDma,
        Hertz(1_000_000),
        Config::default(),
    );

    let mut cs = Output::new(p.PE0, Level::High, Speed::VeryHigh);

    loop {
        let mut buf = [0x0Au8; 4];
        unwrap!(cs.set_low());
        unwrap!(spi.transfer(&mut buf));
        unwrap!(cs.set_high());
        info!("xfer {=[u8]:x}", buf);
    }
}
