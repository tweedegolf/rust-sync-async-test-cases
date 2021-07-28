#![no_std]
#![no_main]

use cortex_m_rt::entry;

use defmt_rtt as _;
use panic_probe as _; // global logger

use defmt::panic;

use accelerometer::{RawAccelerometer, Tracker};
use embedded_ccs811::{prelude::*, Ccs811, MeasurementMode};
use lis3dh::{i2c::Lis3dh, Configuration, DataRate, Lis3dhImpl};

// use crate::{SyncSensorCcs811, SyncSensorLis3dh};
// use sleepy::ram_persisted::{Cached, RamPersisted};
// use sleepy::*;

use sleepy_test_case::ram_persisted::{Cached, RamPersisted};
use sleepy_test_case::test_case_common::*;

use cortex_m::peripheral::NVIC;
use nrf52840_hal::gpio::p0;
use nrf52840_hal::gpio::Level;
use nrf52840_hal::pac::{interrupt, Interrupt};
use nrf52840_hal::twim::Twim;
use nrf52840_hal::Delay;

use core::cell::RefCell;
use cortex_m::interrupt::Mutex;
use nrf52840_hal::gpiote::Gpiote;

#[link_section = ".foobar"]
static mut CCS811_BOX: RamPersisted<SyncSensorCcs811<'static>> = RamPersisted::new();

#[link_section = ".foobar"]
static mut LIS3DH_BOX: RamPersisted<SyncSensorLis3dh> = RamPersisted::new();

// Handle to the GPIOTE peripheral. Uninitialized on reset.
// Must be initialized before use
static GPIOTE_HANDLE: Mutex<RefCell<Option<Gpiote>>> = Mutex::new(RefCell::new(None));

#[entry]
fn main() -> ! {
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

    let peripherals = unsafe { nrf52840_hal::pac::Peripherals::steal() };

    let pins = p0::Parts::new(peripherals.P0);

    // LEDs show who woke us up
    use nrf52840_hal::prelude::OutputPin;
    if ccs_awake_because_int {
        pins.p0_13
            .into_push_pull_output(Level::Low)
            .set_low()
            .unwrap();
    }

    if lis3dh_awake_because_int {
        pins.p0_14
            .into_push_pull_output(Level::Low)
            .set_low()
            .unwrap();
    }

    let twim0_scl = pins.p0_28.into_floating_input().degrade();
    let twim0_sda = pins.p0_29.into_floating_input().degrade();
    let twim0 = nrf52840_hal::twim::Twim::new(
        peripherals.TWIM0,
        nrf52840_hal::twim::Pins {
            scl: twim0_scl,
            sda: twim0_sda,
        },
        nrf52840_hal::twim::Frequency::K400,
    );

    let nwake = pins.p0_30.into_push_pull_output(Level::High).degrade();

    let mut core_peripherals = nrf52840_hal::pac::CorePeripherals::take().unwrap();
    let delay = Delay::new(core_peripherals.SYST);

    let init_from_scratch = || init_ccs_sync(twim0, nwake, delay);
    let cached_sensor = unsafe { CCS811_BOX.take_with_default(init_from_scratch) };

    match cached_sensor {
        Cached::Cached(sensor) => unsafe {
            if ccs_awake_because_int {
                // clear data
                let _ = sensor.data();
            }
            CCS811_BOX.put_back_init(sensor);
        },
        Cached::Fresh(sensor) | Cached::StillInitializing(sensor) => unsafe {
            let _ = sensor.data();

            // put the sensor back, but do not assert that it is initialized
            CCS811_BOX.unsafe_put_back();
        },
    }

    let twim1_scl = pins.p0_04.into_floating_input().degrade();
    let twim1_sda = pins.p0_03.into_floating_input().degrade();
    let twim1 = nrf52840_hal::twim::Twim::new(
        peripherals.TWIM1,
        nrf52840_hal::twim::Pins {
            scl: twim1_scl,
            sda: twim1_sda,
        },
        nrf52840_hal::twim::Frequency::K400,
    );

    let init_from_scratch = || init_lis3dh_sync(twim1);
    let cached_sensor = unsafe { LIS3DH_BOX.take_with_default(init_from_scratch) };

    match cached_sensor {
        Cached::Cached(sensor) => {
            if lis3dh_awake_because_int {
                let _ = sensor.accel_raw().unwrap();
            }
            unsafe {
                LIS3DH_BOX.put_back_init(sensor);
            }
        }

        Cached::Fresh(sensor) | Cached::StillInitializing(sensor) => unsafe {
            let _ = sensor.accel_raw().unwrap();
            LIS3DH_BOX.put_back_init(sensor);
        },
    }

    use nrf52840_hal::gpiote::Gpiote;
    let gpiote = Gpiote::new(peripherals.GPIOTE);

    gpiote
        .channel0()
        .input_pin(&pins.p0_31.into_pulldown_input().degrade())
        .hi_to_lo()
        .enable_interrupt();

    gpiote
        .channel1()
        .input_pin(&pins.p0_02.into_pullup_input().degrade())
        .lo_to_hi()
        .enable_interrupt();

    // Initialize the TIMER0 and GPIOTE handles, passing the initialized
    // peripherals.
    use cortex_m::interrupt::{free as interrupt_free, CriticalSection};
    interrupt_free(|cs: &CriticalSection| {
        // Interrupts are disabled globally in this block
        GPIOTE_HANDLE.borrow(cs).replace(Some(gpiote));
    });

    // Unmask interrupts in NVIC,
    // enabling them globally.
    // Before unmasking, interrupts are disabled
    // Do not unmask the interrupts before intializing the
    // TIMER0 and GPIOTE handles
    unsafe {
        core_peripherals.NVIC.set_priority(Interrupt::GPIOTE, 1);
        NVIC::unmask(Interrupt::GPIOTE);
    }

    loop {
        cortex_m::asm::wfi();
    }
}

fn init_lis3dh_sync(
    twim1: Twim<nrf52840_hal::pac::TWIM1>,
) -> Lis3dh<Twim<nrf52840_hal::pac::TWIM1>> {
    let config = Configuration {
        // Enable temperature readings. Device should run on 2V for temp sensor to work
        temp_en: false,
        // Continuously update data register
        block_data_update: true,
        datarate: DataRate::PowerDown,
        ..Configuration::default()
    };
    let mut lis3dh = Lis3dh::new(twim1, lis3dh::SlaveAddr::Default, config).unwrap();
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
    twim: Twim<nrf52840_hal::pac::TWIM0>,
    nwake: nrf52840_hal::gpio::Pin<nrf52840_hal::gpio::Output<nrf52840_hal::gpio::PushPull>>,
    delay: Delay,
) -> Ccs811<
    Twim<nrf52840_hal::pac::TWIM0>,
    nrf52840_hal::gpio::Pin<nrf52840_hal::gpio::Output<nrf52840_hal::gpio::PushPull>>,
    Delay,
    embedded_ccs811::mode::App,
> {
    let address = embedded_ccs811::SlaveAddr::default();
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

#[interrupt]
unsafe fn GPIOTE() {
    use core::ops::Deref;
    use cortex_m::interrupt::CriticalSection;

    // SAFETY: we're only borrowing GPIOTE_HANDLE, which is never used
    // outside of this interrupt handler, except for initialization which
    // happens before the GPTIOTE interrupt is unmasked.
    let cs = CriticalSection::new();
    // SAFETY: before GPIOTE interrupt is unmasked, GPIOTE_HANDLE is initialized.
    // Therefore it is always initialized before we reach this code.
    if let Some(ref gpiote) = GPIOTE_HANDLE.borrow(&cs).borrow().deref() {
        // Check if something happened on channel 0
        if gpiote.channel0().is_event_triggered() {
            let ccs811 = CCS811_BOX.take();
            match ccs811.data() {
                Err(_) => (),
                Ok(_data) => {
                    // defmt::info!("eCO2: {}, eTVOC: {}", data.eco2, data.etvoc);
                    gpiote.channel0().reset_events();
                    go_to_sleep();
                }
            }

            gpiote.channel0().reset_events();
        }

        // Check if something happened on channel 1
        if gpiote.channel1().is_event_triggered() {
            let lis3dh = LIS3DH_BOX.take();
            let accel = lis3dh.accel_raw().unwrap();
            let mut tracker = Tracker::new(3700.0);
            let _orientation = tracker.update(accel);
            // defmt::info!("data: {}", orientation.is_portrait());

            // LIS3DH_BOX.put_back_init(lis3dh);

            gpiote.channel1().reset_events();

            go_to_sleep();
        }
    };
}
