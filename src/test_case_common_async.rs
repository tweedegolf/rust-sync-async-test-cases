use embassy_nrf::gpio::Output;
use embassy_nrf::twim;

use ccs811_async::{Ccs811, MeasurementMode, SlaveAddr};

pub type SensorCcs811<'a> = Ccs811<
    twim::Twim<'a, embassy_nrf::peripherals::TWISPI0>,
    Output<'a, embassy_nrf::peripherals::P0_30>,
    ccs811_async::mode::App,
>;

pub type SensorLis3dh<'a> =
    lis3dh::asynci2c::Lis3dh<twim::Twim<'a, embassy_nrf::peripherals::TWISPI1>>;

pub async fn init_lis3dh(
    twim: twim::Twim<'_, embassy_nrf::peripherals::TWISPI1>,
) -> SensorLis3dh<'_> {
    defmt::info!("making a new lis3dh");
    use lis3dh::asynci2c::Lis3dh;
    use lis3dh::{Configuration, DataRate, SlaveAddr};
    let config = Configuration {
        // Enable temperature readings. Device should run on 2V for temp sensor to work
        temp_en: false,
        // Continuously update data register
        block_data_update: true,
        datarate: DataRate::PowerDown,
        ..Configuration::default()
    };
    let mut lis3dh = Lis3dh::new(twim, SlaveAddr::Default, config).await.unwrap();
    lis3dh
        .configure_irq_threshold(lis3dh::Interrupt1, 69)
        .await
        .unwrap();
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

    lis3dh
}

pub async fn init_ccs<'a>(
    twim: twim::Twim<'a, embassy_nrf::peripherals::TWISPI0>,
    address: SlaveAddr,
    nwake: Output<'a, embassy_nrf::peripherals::P0_30>,
) -> SensorCcs811<'a> {
    let sensor = Ccs811::new(twim, address, nwake);
    let mut sensor = sensor
        .start_application()
        .await
        .unwrap_or_else(|e| panic!("error starting application {:?}", e.error));
    sensor
        // .set_mode(MeasurementMode::ConstantPower1s)
        .set_mode(MeasurementMode::PulseHeating10s)
        .await
        .unwrap();
    sensor
        .set_interrupt_mode(ccs811_async::InterruptMode::OnDataReady)
        .await
        .unwrap();
    sensor
}
