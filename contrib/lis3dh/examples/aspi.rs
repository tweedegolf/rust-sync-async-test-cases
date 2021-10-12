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
use embassy_nrf::{interrupt, spim};
use embassy_traits::spi::FullDuplex;
use embedded_hal::digital::v2::*;
use futures::pin_mut;

use lis3dh::asyncspi::Lis3dh;

#[embassy::main]
async fn main(spawner: Spawner) {
    defmt::info!("yay main");
    let p = unsafe { Peripherals::steal() };

    let config = spim::Config {
        frequency: spim::Frequency::K500,
        mode: spim::MODE_0,
        orc: 0x00,
    };

    let irq = interrupt::take!(SPIM3);
    // let spim = spim::Spim::new(p.SPIM3, irq, p.P0_29, p.P0_28, p.P0_30, config);
    let spim = spim::Spim::new(p.SPIM3, irq, p.P0_31, p.P0_29, p.P0_30, config);
    let mut ncs = Output::new(p.P0_28, Level::Low, OutputDrive::Standard);

    // sck, miso, mosi
    // let spim = spim::Spim::new(p.SPIM3, irq, p.P0_05, p.P0_04, p.P0_06, config);
    // let ncs = Output::new(p.P0_03, Level::Low, OutputDrive::Standard);
    let delay = embassy::time::Delay::new();

    pin_mut!(delay);

    defmt::info!("yay delay ");
    let mut lis3dh = Lis3dh::new(spim, ncs, delay.as_mut()).await.unwrap();
    // let mut lis3dh = Lis3dh::new(spi, cs, delay, Configuration::default()).unwrap();
    // lis3dh.set_range(lis3dh::Range::G8).unwrap();

    let mut tracker = Tracker::new(3700.0);

    loop {
        let accel = lis3dh.accel_raw().await.unwrap();
        let orientation = tracker.update(accel);
        // info!("info {}, {}, {}", accel.x, accel.y, accel.z);
        defmt::info!("foobar {}", orientation.is_flat());
        // hprintln!("{:?}", accel).ok();
        // delay.delay_ms(1000u16)
    }
}
