use crate::hal::digital::v2::OutputPin;
use crate::{
    hal, mode, register_access::get_errors, AlgorithmResult, BitFlags, Ccs811, Ccs811AppMode,
    Ccs811Awake, Error, ErrorAwake, InterruptMode, MeasurementMode, ModeChangeError, Register,
};
use embassy::time::{Duration, Timer};

impl<I2C, E> Ccs811Awake<I2C, mode::App>
where
    I2C: embassy_traits::i2c::I2c<Error = E>,
{
    pub async fn set_mode(&mut self, mode: MeasurementMode) -> Result<(), ErrorAwake<E>> {
        let idle_mode = self.meas_mode_reg & 0b0000_1100;
        let meas_mode = match mode {
            MeasurementMode::Idle => idle_mode,
            MeasurementMode::ConstantPower1s => idle_mode | 1 << 4,
            MeasurementMode::PulseHeating10s => idle_mode | 2 << 4,
            MeasurementMode::LowPowerPulseHeating60s => idle_mode | 3 << 4,
            MeasurementMode::ConstantPower250ms => idle_mode | 4 << 4,
        };
        self.write_register_1byte(Register::MEAS_MODE, meas_mode)
            .await?;
        self.meas_mode_reg = meas_mode;
        Ok(())
    }

    async fn has_data_ready(&mut self) -> Result<bool, ErrorAwake<E>> {
        let status = self.read_status().await?;
        Ok((status & BitFlags::DATA_READY) != 0)
    }

    async fn raw_data(&mut self) -> Result<(u8, u16), ErrorAwake<E>> {
        let data = self.read_register_2bytes(Register::RAW_DATA).await?;
        Ok(handle_raw_data(data[0], data[1]))
    }

    pub async fn data(&mut self) -> nb::Result<AlgorithmResult, ErrorAwake<E>> {
        let mut data = [0; 8];
        self.i2c
            .write_read(self.address, &mut [Register::ALG_RESULT_DATA], &mut data)
            .await
            .map_err(ErrorAwake::I2C)?;
        let status = data[4];
        if (status & BitFlags::ERROR) != 0 {
            get_errors(data[5]).map_err(ErrorAwake::Device)?;
        } else if (status & BitFlags::DATA_READY) == 0 {
            return Err(nb::Error::WouldBlock);
        }
        let raw = handle_raw_data(data[6], data[7]);
        Ok(AlgorithmResult {
            eco2: (u16::from(data[0]) << 8) | u16::from(data[1]),
            etvoc: (u16::from(data[2]) << 8) | u16::from(data[3]),
            raw_current: raw.0,
            raw_voltage: raw.1,
        })
    }

    async fn set_environment(
        &mut self,
        humidity_percentage: f32,
        temperature_celsius: f32,
    ) -> Result<(), ErrorAwake<E>> {
        if humidity_percentage < 0.0
            || humidity_percentage > 100.0
            || temperature_celsius > 254.998_05
        {
            return Err(ErrorAwake::InvalidInputData);
        }
        let raw_humidity = get_raw_humidity(humidity_percentage);
        let raw_temp = get_raw_temperature(temperature_celsius);
        let raw = [
            Register::ENV_DATA,
            raw_humidity.0,
            raw_humidity.1,
            raw_temp.0,
            raw_temp.1,
        ];
        self.i2c
            .write(self.address, &raw)
            .await
            .map_err(ErrorAwake::I2C)?;
        self.check_status_error().await
    }

    async fn baseline(&mut self) -> Result<[u8; 2], ErrorAwake<E>> {
        self.read_register_2bytes(Register::BASELINE).await
    }

    async fn set_baseline(&mut self, baseline: [u8; 2]) -> Result<(), ErrorAwake<E>> {
        self.i2c
            .write(
                self.address,
                &[Register::BASELINE, baseline[0], baseline[1]],
            )
            .await
            .map_err(ErrorAwake::I2C)?;
        self.check_status_error().await
    }

    async fn set_eco2_thresholds(
        &mut self,
        low_to_medium: u16,
        medium_to_high: u16,
    ) -> Result<(), ErrorAwake<E>> {
        self.i2c
            .write(
                self.address,
                &[
                    Register::THRESHOLDS,
                    (low_to_medium >> 8) as u8,
                    low_to_medium as u8,
                    (medium_to_high >> 8) as u8,
                    medium_to_high as u8,
                ],
            )
            .await
            .map_err(ErrorAwake::I2C)?;

        self.check_status_error().await
    }

    pub async fn set_interrupt_mode(&mut self, mode: InterruptMode) -> Result<(), ErrorAwake<E>> {
        let int_mask = match mode {
            InterruptMode::Disabled => 0,
            InterruptMode::OnDataReady => BitFlags::INTERRUPT,
            InterruptMode::OnThresholdCrossed => BitFlags::INTERRUPT | BitFlags::THRESH,
        };
        let meas_mode = (self.meas_mode_reg & (0b111 << 4)) | int_mask;
        self.write_register_1byte(Register::MEAS_MODE, meas_mode)
            .await?;
        self.meas_mode_reg = meas_mode;
        Ok(())
    }

    // Note: is_verifying is false after a reset
    async fn software_reset(
        mut self,
    ) -> Result<Ccs811Awake<I2C, mode::Boot>, ModeChangeError<ErrorAwake<E>, Self>> {
        match self.write_sw_reset().await {
            Err(e) => Err(ModeChangeError::new(self, e)),
            Ok(_) => Ok(Ccs811Awake::create(self.i2c, self.address)),
        }
    }
}

fn get_raw_humidity(humidity_percentage: f32) -> (u8, u8) {
    get_raw_environment_data(humidity_percentage)
}

fn get_raw_temperature(temperature_celsius: f32) -> (u8, u8) {
    let value = temperature_celsius + 25.0;
    if value < 0.0 {
        (0, 0)
    } else {
        get_raw_environment_data(value)
    }
}

fn get_raw_environment_data(value: f32) -> (u8, u8) {
    let main = (value as u8) << 1;
    let rest = value - f32::from(value as u8);
    let rest = (rest * 512.0) as u16;
    (main | (((rest & (1 << 8)) >> 8) as u8), rest as u8)
}

fn handle_raw_data(data0: u8, data1: u8) -> (u8, u16) {
    (
        (data1 >> 2) as u8,
        u16::from(data0) | (u16::from(data1 & 0x3) << 8),
    )
}

use crate::{on_awaken, on_awaken_nb};

impl<I2C, CommE, PinE, NWAKE> Ccs811<I2C, NWAKE, mode::App>
where
    I2C: embassy_traits::i2c::I2c<Error = CommE>,
    NWAKE: OutputPin<Error = PinE>,
{
    pub fn wake(&mut self) -> Result<(), Error<CommE, PinE>> {
        self.n_wake_pin.set_low().map_err(Error::Pin)
    }

    pub fn sleep(&mut self) -> Result<(), Error<CommE, PinE>> {
        self.n_wake_pin.set_high().map_err(Error::Pin)
    }

    pub async fn set_mode(&mut self, mode: MeasurementMode) -> Result<(), Error<CommE, PinE>> {
        on_awaken!(self, self.dev.set_mode(mode))
    }

    pub async fn has_data_ready(&mut self) -> Result<bool, Error<CommE, PinE>> {
        on_awaken!(self, self.dev.has_data_ready())
    }

    async fn raw_data(&mut self) -> Result<(u8, u16), Error<CommE, PinE>> {
        on_awaken!(self, self.dev.raw_data())
    }

    pub async fn data(&mut self) -> nb::Result<AlgorithmResult, Error<CommE, PinE>> {
        on_awaken_nb!(self, self.dev.data())
    }

    async fn baseline(&mut self) -> Result<[u8; 2], Error<CommE, PinE>> {
        on_awaken!(self, self.dev.baseline())
    }

    async fn set_baseline(&mut self, baseline: [u8; 2]) -> Result<(), Error<CommE, PinE>> {
        on_awaken!(self, self.dev.set_baseline(baseline))
    }

    async fn set_environment(
        &mut self,
        humidity_percentage: f32,
        temperature_celsius: f32,
    ) -> Result<(), Error<CommE, PinE>> {
        on_awaken!(
            self,
            self.dev
                .set_environment(humidity_percentage, temperature_celsius)
        )
    }

    async fn set_eco2_thresholds(
        &mut self,
        low_to_medium: u16,
        medium_to_high: u16,
    ) -> Result<(), Error<CommE, PinE>> {
        on_awaken!(
            self,
            self.dev.set_eco2_thresholds(low_to_medium, medium_to_high)
        )
    }

    pub async fn set_interrupt_mode(
        &mut self,
        mode: InterruptMode,
    ) -> Result<(), Error<CommE, PinE>> {
        on_awaken!(self, self.dev.set_interrupt_mode(mode))
    }

    async fn software_reset(
        mut self,
    ) -> Result<Ccs811<I2C, NWAKE, mode::Boot>, ModeChangeError<Error<CommE, PinE>, Self>> {
        if let Err(e) = self.n_wake_pin.set_low() {
            return Err(ModeChangeError::new(self, Error::Pin(e)));
        }
        Timer::after(Duration::from_micros(50)).await;
        let Ccs811 {
            dev,
            mut n_wake_pin,
            ..
        } = self;
        let result = dev.software_reset().await;
        if let Err(e) = n_wake_pin.set_high() {
            return match result {
                Ok(Ccs811Awake { i2c, address, .. }) => Err(ModeChangeError {
                    dev: Ccs811::create(i2c, address, n_wake_pin),
                    error: Error::Pin(e),
                }),
                Err(ModeChangeError { dev, error }) => Err(ModeChangeError {
                    dev: Ccs811::from_awake_dev(dev, n_wake_pin),
                    error: error.into(),
                }),
            };
        }
        Timer::after(Duration::from_micros(20)).await;
        match result {
            Ok(dev) => Ok(Ccs811::from_awake_dev(dev, n_wake_pin)),
            Err(ModeChangeError { dev, error }) => Err(ModeChangeError {
                dev: Ccs811::from_awake_dev(dev, n_wake_pin),
                error: error.into(),
            }),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn convert_humidity() {
        assert_eq!((0, 0), get_raw_humidity(0.0));
        assert_eq!((0x64, 0), get_raw_humidity(50.0));
        assert_eq!((0x61, 0), get_raw_humidity(48.5));
        assert_eq!((0x60, 0x80), get_raw_humidity(48.25));
        assert_eq!((0x60, 0x40), get_raw_humidity(48.125));
        assert_eq!((0x60, 0x20), get_raw_humidity(48.0625));
        assert_eq!((0x60, 0x10), get_raw_humidity(48.03125));
        assert_eq!((0x60, 0x08), get_raw_humidity(48.015_625));
        assert_eq!((0x60, 0x04), get_raw_humidity(48.007_813));
        assert_eq!((0x60, 0x02), get_raw_humidity(48.003_906));
        assert_eq!((0x60, 0x01), get_raw_humidity(48.001_953));
        assert_eq!((0x61, 0xFF), get_raw_humidity(48.998_047));
    }

    #[test]
    fn convert_temperature() {
        assert_eq!((0, 0), get_raw_temperature(-25.5));
        assert_eq!((0, 0), get_raw_temperature(-25.0));
        assert_eq!((0x64, 0), get_raw_temperature(25.0));
        assert_eq!((0x61, 0), get_raw_temperature(23.5));
    }
}
