#![no_std]
#![no_main]

use cortex_m_rt::entry;

use defmt::panic;
use defmt_rtt as _;
use hal::pac::{TWIM0, TWIM1};
use lis3dh::SlaveAddr as LisSlaveAddr;
use panic_probe as _; // global logger

use accelerometer::{RawAccelerometer, Tracker};
use embedded_ccs811::{mode, prelude::*, Ccs811, MeasurementMode, SlaveAddr as CcsSlaveAddr};
use lis3dh::{i2c::Lis3dh, Configuration, DataRate, Lis3dhImpl};

use sleepy_test_case::ram_persisted::{Cached, RamPersisted};
use sleepy_test_case::test_case_common::*;

use hal::gpio::Level;
use hal::gpio::{p0, Output, Pin, PushPull};
use hal::prelude::*;
use hal::twim::Twim;
use hal::Delay;
use hal::{pac, twim};
use nrf52840_hal as hal;

#[link_section = ".foobar"]
static mut CCS811_BOX: RamPersisted<SyncSensorCcs811<'static>> = RamPersisted::new();

#[link_section = ".foobar"]
static mut LIS3DH_BOX: RamPersisted<SyncSensorLis3dh> = RamPersisted::new();

#[entry]
fn main() -> ! {
    // retain the `.foobar` section
    configure_ram();

    let peripherals = pac::Peripherals::take().unwrap();
    let port0 = p0::Parts::new(peripherals.P0);

    // Find out which of the two sensors woke up the device
    let (ccs_awake_because_int, lis3dh_awake_because_int) = {
        let wait = {
            let port0_register_block = pac::P0::ptr();
            0 == unsafe { (*port0_register_block).latch.read().bits() }
        };

        let int_ccs = port0.p0_31.into_pulldown_input();
        let int_lis3dh = port0.p0_02.into_pullup_input();

        // time to configure the pin
        // experimentally, it takes about 5us to switch states. At
        // 64MHz, that's 64e6 ticks per second * 5e-6 seconds = 320 ticks
        if wait {
            cortex_m::asm::delay(320);
        }

        (int_ccs.is_low().unwrap(), int_lis3dh.is_high().unwrap())
    };

    let twim0_scl = port0.p0_28.into_floating_input().degrade();
    let twim0_sda = port0.p0_29.into_floating_input().degrade();
    let twim0 = twim::Twim::new(
        peripherals.TWIM0,
        twim::Pins {
            scl: twim0_scl,
            sda: twim0_sda,
        },
        twim::Frequency::K400,
    );

    let nwake = port0.p0_30.into_push_pull_output(Level::High).degrade();

    let core_peripherals = pac::CorePeripherals::take().unwrap();
    let delay = Delay::new(core_peripherals.SYST);

    let init_from_scratch = || init_ccs_sync(twim0, nwake, delay);
    let cached_sensor = unsafe { CCS811_BOX.take_with_default(init_from_scratch) };

    match cached_sensor {
        Cached::Cached(sensor) => unsafe {
            if ccs_awake_because_int {
                // clear data
                match sensor.data() {
                    Err(_) => (),
                    Ok(data) => drop(data),
                };
            }
            CCS811_BOX.put_back_init(sensor);
        },
        Cached::Fresh(sensor) | Cached::StillInitializing(sensor) => unsafe {
            // Clear buffer
            let _ = sensor.data();

            // put the sensor back, but do not assert that it is initialized
            CCS811_BOX.unsafe_put_back();
        },
    }

    let twim1_scl = port0.p0_04.into_floating_input().degrade();
    let twim1_sda = port0.p0_03.into_floating_input().degrade();
    let twim1 = twim::Twim::new(
        peripherals.TWIM1,
        twim::Pins {
            scl: twim1_scl,
            sda: twim1_sda,
        },
        twim::Frequency::K400,
    );

    let init_from_scratch = || init_lis3dh_sync(twim1);
    let cached_sensor = unsafe { LIS3DH_BOX.take_with_default(init_from_scratch) };

    match cached_sensor {
        Cached::Cached(sensor) => {
            if lis3dh_awake_because_int {
                let accel = sensor.accel_raw().unwrap();
                let mut tracker = Tracker::new(3700.0);
                let _orientation = tracker.update(accel);
            }
            unsafe {
                LIS3DH_BOX.put_back_init(sensor);
            }
        }

        Cached::Fresh(sensor) | Cached::StillInitializing(sensor) => unsafe {
            // Clear buffer
            let _ = sensor.accel_raw().unwrap();
            LIS3DH_BOX.put_back_init(sensor);
        },
    }

    go_to_sleep();
}

fn init_lis3dh_sync(twim1: Twim<TWIM1>) -> Lis3dh<Twim<TWIM1>> {
    let config = Configuration {
        // Enable temperature readings. Device should run on 2V for temp sensor to work
        temp_en: false,
        // Continuously update data register
        block_data_update: true,
        datarate: DataRate::PowerDown,
        ..Configuration::default()
    };
    let mut lis3dh = Lis3dh::new(twim1, LisSlaveAddr::Default, config).unwrap();
    lis3dh
        .configure_irq_threshold(lis3dh::Interrupt1, 69)
        .unwrap();
    lis3dh
        .configure_irq_duration(lis3dh::Interrupt1, 0)
        .unwrap();
    use lis3dh::{InterruptSource, IrqPin1Conf};
    lis3dh
        .configure_irq_src(lis3dh::Interrupt1, InterruptSource::default(), false, false)
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
        .unwrap();
    lis3dh.set_datarate(DataRate::Hz_1).unwrap();
    lis3dh
}

fn init_ccs_sync(
    twim: Twim<TWIM0>,
    nwake: Pin<Output<PushPull>>,
    delay: Delay,
) -> Ccs811<Twim<TWIM0>, Pin<Output<PushPull>>, Delay, mode::App> {
    let address = CcsSlaveAddr::default();
    let sensor = Ccs811::new(twim, address, nwake, delay);
    let mut sensor = sensor.start_application().unwrap_or_else(|_| {
        panic!(
            "error starting application {:?}",
            /* e.error*/ "unknown"
        )
    });
    sensor
        // .set_mode(MeasurementMode::ConstantPower1s)
        .set_mode(MeasurementMode::PulseHeating10s)
        .unwrap();
    sensor
        .set_interrupt_mode(embedded_ccs811::InterruptMode::OnDataReady)
        .unwrap();
    sensor
}
