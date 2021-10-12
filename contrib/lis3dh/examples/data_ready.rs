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

use embassy_nrf::gpio::{Input, Pull};
use embassy_nrf::gpiote::{InputChannel, InputChannelPolarity};

use lis3dh::asynci2c::Lis3dh;
use lis3dh::{Configuration, DataRate, SlaveAddr};

#[embassy::main]
async fn main(_spawner: Spawner, p: Peripherals) {
    defmt::info!("yay main");

    let twim0_scl = p.P0_28;
    let twim0_sda = p.P0_29;

    // listen for when data is ready
    let data_ready_channel = InputChannel::new(
        p.GPIOTE_CH0,
        Input::new(p.P0_02, Pull::Up),
        InputChannelPolarity::LoToHi,
    );

    let mut config = twim::Config::default();
    config.frequency = twim::Frequency::K400;

    let irq = interrupt::take!(SPIM0_SPIS0_TWIM0_TWIS0_SPI0_TWI0);
    let twim = twim::Twim::new(p.TWISPI0, irq, twim0_sda, twim0_scl, config);

    let config = Configuration {
        // Enable temperature readings. Device should run on 2V for temp sensor to work
        temp_en: false,
        // Continuously update data register
        block_data_update: true,
        datarate: DataRate::PowerDown,
        ..Configuration::default()
    };

    let mut lis3dh = Lis3dh::new(twim, SlaveAddr::Default, config).await.unwrap();

    // Configure the threshold value for interrupt 1 to 1.1g
    lis3dh
        .configure_irq_threshold(lis3dh::Interrupt1, 69)
        .await
        .unwrap();

    // The time in 1/ODR an axis value should be above threshold in order for an
    // interrupt to be raised
    lis3dh
        .configure_irq_duration(lis3dh::Interrupt1, 0)
        .await
        .unwrap();

    use lis3dh::{InterruptSource, IrqPin1Conf};

    lis3dh
        .configure_irq_src(lis3dh::Interrupt1, InterruptSource::default(), false, false)
        .await
        .unwrap();

    lis3dh
        .configure_int_pin(IrqPin1Conf {
            // Raise if interrupt 1 is raised
            ia1_en: true,
            // Raise interrupt 1 if accelerometer data is ready
            zyxda_en: true,
            // Disable for all other interrupts
            ..IrqPin1Conf::default()
        })
        .await
        .unwrap();

    lis3dh.set_datarate(DataRate::Hz_1).await.unwrap();

    let mut tracker = Tracker::new(3700.0);

    // ask for data once to get the process started
    let _ = lis3dh.accel_raw().await.unwrap();

    loop {
        data_ready_channel.wait().await;
        let accel = lis3dh.accel_raw().await.unwrap();
        let orientation = tracker.update(accel);
        defmt::info!("foobar {}", orientation.is_flat());
    }
}
