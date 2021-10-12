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
use nrf52840_hal::spim::Spim;
use nrf52840_hal::Delay;

use lis3dh::{spi::Lis3dh, Configuration, Lis3dhImpl, SlaveAddr};

#[entry]
fn main() -> ! {
    let mut peripherals = nrf52840_hal::pac::Peripherals::take().unwrap();

    let port0 = p0::Parts::new(peripherals.P0);

    // let cs: P0_03<gpio::Output<PushPull>> = port0.p0_03.into_push_pull_output(Level::Low);
    let cs: P0_28<gpio::Output<PushPull>> = port0.p0_28.into_push_pull_output(Level::Low);

    let _btn1 = port0.p0_13.into_pullup_input();
    let _btn2 = port0.p0_14.into_pullup_input();
    let _btn3 = port0.p0_15.into_pullup_input();
    let _btn4 = port0.p0_16.into_pullup_input();

    // let spim2 = spim::Spim::new(p.SPIM2, irq2, p.P0_01, p.P0_05, p.P0_02, config2);

    let spiclk = port0.p0_31.into_push_pull_output(Level::Low).degrade();
    let spimosi = port0.p0_30.into_push_pull_output(Level::Low).degrade();
    let spimiso = port0.p0_29.into_floating_input().degrade();

    //    let spiclk = port0.p0_05.into_push_pull_output(Level::Low).degrade();
    //    let spimosi = port0.p0_06.into_push_pull_output(Level::Low).degrade();
    //    let spimiso = port0.p0_04.into_floating_input().degrade();

    let mut tests_ok = true;
    let pins = nrf52840_hal::spim::Pins {
        sck: spiclk,
        miso: Some(spimiso),
        mosi: Some(spimosi),
    };
    let mut spi = Spim::new(
        peripherals.SPIM2,
        pins,
        nrf52840_hal::spim::Frequency::K500,
        nrf52840_hal::spim::MODE_0,
        0,
    );

    let core_peripherals = nrf52840_hal::pac::CorePeripherals::take().unwrap();
    let mut delay = Delay::new(core_peripherals.SYST);

    let mut lis3dh = match Lis3dh::new(spi, cs, delay, Configuration::default()) {
        Ok(v) => v,
        Err(_) => panic!("error while initializing"),
    };
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
