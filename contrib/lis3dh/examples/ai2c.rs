#![feature(type_alias_impl_trait)]
#![feature(min_type_alias_impl_trait)]
#![feature(impl_trait_in_bindings)]
#![no_std]
#![no_main]

// use circuit_playground_express as hal;
extern crate panic_halt;

use accelerometer::{RawAccelerometer, Tracker};
use cortex_m_rt::entry;

use defmt::panic;
use defmt::*;
use defmt_rtt as _; // global logger

use embedded_hal::blocking::delay::DelayUs;

use embassy::executor::Spawner;
use embassy::util::Steal;
use embassy_nrf::gpio::{Level, Output, OutputDrive};
use embassy_nrf::Peripherals;
use embassy_nrf::{interrupt, twim};
use embassy_traits::i2c::I2c;
use embedded_hal::digital::v2::*;
use futures::pin_mut;

use lis3dh::asynci2c::Lis3dh;
use lis3dh::Configuration;
use lis3dh::SlaveAddr;

#[embassy::main]
async fn main(spawner: Spawner, p: Peripherals) {
    defmt::info!("yay main");

    let twim0_scl = p.P0_28;
    let twim0_sda = p.P0_29;

    let mut config = twim::Config::default();
    config.frequency = twim::Frequency::K400;

    let irq = interrupt::take!(SPIM0_SPIS0_TWIM0_TWIS0_SPI0_TWI0);
    let twim = twim::Twim::new(p.TWISPI0, irq, twim0_sda, twim0_scl, config);

    // Register::WHOAMI

    let mut lis3dh = Lis3dh::new(twim, SlaveAddr::Default, Configuration::default())
        .await
        .unwrap();

    let mut tracker = Tracker::new(3700.0);

    loop {
        let accel = lis3dh.accel_raw().await.unwrap();
        let orientation = tracker.update(accel);
        defmt::info!("foobar {}", orientation.is_flat());
    }
}
