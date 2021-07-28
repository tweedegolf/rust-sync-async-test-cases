#![no_std]
#![no_main]
#![feature(min_type_alias_impl_trait)]
#![feature(impl_trait_in_bindings)]
#![feature(type_alias_impl_trait)]
#![allow(incomplete_features)]
#![feature(async_closure)]

use defmt_rtt as _; // global logger

use defmt::panic;
use embassy::executor::Spawner;

use embassy_nrf::gpio::{Input, Level, Output, OutputDrive, Pull};
use embassy_nrf::gpiote::{InputChannel, InputChannelPolarity};
use embassy_nrf::twim;
use embassy_nrf::{interrupt, Peripherals};

use ccs811_async::SlaveAddr;

use sleepy_test_case::ram_persisted::{Cached, RamPersisted};
use sleepy_test_case::test_case_common_async::*;
use sleepy_test_case::test_case_common::*;

#[link_section = ".foobar"]
static mut CCS811_BOX: RamPersisted<SensorCcs811<'static>> = RamPersisted::new();

#[link_section = ".foobar"]
static mut LIS3DH_BOX: RamPersisted<SensorLis3dh<'static>> = RamPersisted::new();

#[embassy::main]
async fn main(_spawner: Spawner, p: Peripherals) {
    // retain the `.foobar` section
    configure_ram();

    let (ccs_awake_because_int, lis3dh_awake_because_int) = {
        use nrf52840_hal::prelude::InputPin;

        let wait = {
            let p0_register_block = nrf52840_hal::pac::P0::ptr();
            0 == unsafe { (*p0_register_block).latch.read().bits() }
        };

        let p = unsafe { nrf52840_hal::pac::Peripherals::steal() };

        let pins = nrf52840_hal::gpio::p0::Parts::new(p.P0);

        let int_ccs = pins.p0_31.into_pulldown_input();
        let int_lis3dh = pins.p0_02.into_pullup_input();

        // time to configure the pin
        // experimentally, it takes about 5us to switch states. At
        // 64MHz, that's 64e6 ticks per second * 5e-6 seconds = 320 ticks
        if wait {
            cortex_m::asm::delay(320);
        }

        (int_ccs.is_low().unwrap(), int_lis3dh.is_high().unwrap())
    };

    let lis3dh_data_ready_channel = InputChannel::new(
        p.GPIOTE_CH1,
        Input::new(p.P0_02, Pull::Up),
        InputChannelPolarity::LoToHi,
    );

    let twim0_scl = p.P0_28;
    let twim0_sda = p.P0_29;

    let config = twim::Config::default();
    let irq = interrupt::take!(SPIM0_SPIS0_TWIM0_TWIS0_SPI0_TWI0);
    let twim = twim::Twim::new(p.TWISPI0, irq, twim0_sda, twim0_scl, config);

    // listen for when data is ready
    let ccs_data_ready_channel = InputChannel::new(
        p.GPIOTE_CH0,
        Input::new(p.P0_31, Pull::Down),
        InputChannelPolarity::HiToLo,
    );

    let address = SlaveAddr::default();
    let nwake = Output::new(p.P0_30, Level::High, OutputDrive::Standard);

    let init_from_scratch = || async move { init_ccs(twim, address, nwake).await };

    let ccs_init_task = async {
        let cached_sensor = unsafe { CCS811_BOX.async_take_with_default(init_from_scratch).await };

        match cached_sensor {
            Cached::Cached(sensor) => {
                // the sensor was already there;
                sensor
            }
            Cached::Fresh(sensor) | Cached::StillInitializing(sensor) => {
                defmt::info!("ccs is NOT done already");
                loop {
                    match sensor.data().await {
                        Ok(_data) => {
                            // we're fully initiallized
                            break;
                        }
                        Err(nb::Error::WouldBlock) => {
                            // this happens on startup
                            ccs_data_ready_channel.wait().await;

                            continue;
                        }
                        Err(nb::Error::Other(e)) => panic!("Error while retrieving data {:?}", e),
                    };
                }

                sensor
            }
        }
    };

    let twim1_scl = p.P0_04;
    let twim1_sda = p.P0_03;

    let config = twim::Config::default();
    let irq = interrupt::take!(SPIM1_SPIS1_TWIM1_TWIS1_SPI1_TWI1);
    let twim = twim::Twim::new(p.TWISPI1, irq, twim1_sda, twim1_scl, config);

    let init_from_scratch = || async move { init_lis3dh(twim).await };

    let lis3dh_init_task = async {
        let cached_sensor = unsafe { LIS3DH_BOX.async_take_with_default(init_from_scratch).await };

        //        match cached_sensor {
        //            Cached::Cached(_) => defmt::info!("lis3dh: cached"),
        //            Cached::StillInitializing(_) => defmt::info!("lis3dh: still initing"),
        //            Cached::Fresh(_) => defmt::info!("lis3dh: fresh"),
        //        };

        match cached_sensor {
            Cached::Cached(sensor) | Cached::Fresh(sensor) | Cached::StillInitializing(sensor) => {
                sensor
            }
        }
    };

    let (ccs_sensor, lis3dh_sensor) = futures::join!(ccs_init_task, lis3dh_init_task);

    let ccs_task = async {
        loop {
            if !ccs_awake_because_int {
                ccs_data_ready_channel.wait().await;
            }

            match ccs_sensor.data().await {
                Ok(_data) => {
                    unsafe {
                        CCS811_BOX.put_back_init(ccs_sensor);
                        LIS3DH_BOX.unsafe_put_back();
                    }

                    go_to_sleep()
                }
                Err(nb::Error::WouldBlock) => {
                    // this happens on startup
                    ccs_data_ready_channel.wait().await;

                    continue;
                }
                Err(nb::Error::Other(e)) => panic!("Error while retrieving data {:?}", e),
            };
        }
    };

    let lis3dh_task = async {
        if !lis3dh_awake_because_int {
            lis3dh_data_ready_channel.wait().await;
        }

        loop {
            match lis3dh_sensor.accel_raw().await {
                Ok(_data) => {
                    unsafe {
                        LIS3DH_BOX.put_back_init(lis3dh_sensor);
                        CCS811_BOX.unsafe_put_back();
                    }

                    go_to_sleep()
                }
                Err(_) => panic!("Error while retrieving data {:?}", ()),
            };
        }
    };

    futures::join!(ccs_task, lis3dh_task);
}
