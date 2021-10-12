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
use nrf52840_hal::gpio;
use nrf52840_hal::gpio::p0::*;
use nrf52840_hal::gpio::Level;
use nrf52840_hal::gpio::*;
use nrf52840_hal::prelude::_embedded_hal_blocking_delay_DelayMs;
use nrf52840_hal::twim::{Frequency as TwimFrequency, Pins as TwimPins, Twim};
use nrf52840_hal::Delay;

use lis3dh::{i2c::Lis3dh, Configuration, Lis3dhImpl, SlaveAddr};

#[entry]
fn main() -> ! {
    let peripherals = nrf52840_hal::pac::Peripherals::take().unwrap();

    let pins = p0::Parts::new(peripherals.P0);

    let twim0_scl = pins.p0_28.into_floating_input().degrade();
    let twim0_sda = pins.p0_29.into_floating_input().degrade();
    let i2c = nrf52840_hal::twim::Twim::new(
        peripherals.TWIM0,
        nrf52840_hal::twim::Pins {
            scl: twim0_scl,
            sda: twim0_sda,
        },
        nrf52840_hal::twim::Frequency::K400,
    );

    let mut lis3dh = Lis3dh::new(i2c, SlaveAddr::Default, Configuration::default()).unwrap();

    defmt::info!(
        "Found Lis3dh. Device id: {}",
        lis3dh.get_device_id().unwrap()
    );

    lis3dh.set_range(lis3dh::Range::G8).unwrap();

    let mut tracker = Tracker::new(3700.0);

    loop {
        let accel = lis3dh.accel_raw().unwrap();
        let orientation = tracker.update(accel);
        defmt::info!("foobar {}", orientation.is_flat());
        // hprintln!("{:?}", accel).ok();
        // delay.delay_ms(1000u16)
    }
}
