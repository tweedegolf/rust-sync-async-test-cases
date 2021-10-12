use crate::hal::{
    blocking::delay::{DelayMs, DelayUs},
    digital::v2::OutputPin,
};
use crate::{
    hal, mode, ActionInProgress, BitFlags, Ccs811, Ccs811Awake, Ccs811BootMode, Ccs811Device,
    Error, ErrorAwake, ModeChangeError, Register,
};
use embassy::time::{Duration, Timer};

impl<I2C, E> Ccs811Awake<I2C, mode::Boot>
where
    I2C: embassy_traits::i2c::I2c<Error = E>,
{
    async fn start_application(
        mut self,
    ) -> Result<Ccs811Awake<I2C, mode::App>, ModeChangeError<ErrorAwake<E>, Self>> {
        match self.has_valid_app().await {
            Err(e) => {
                defmt::info!("not valid app");
                Err(ModeChangeError::new(self, e))
            }
            Ok(is_valid) => {
                if !is_valid {
                    defmt::info!("!is_valid");
                    Err(ModeChangeError::new(self, ErrorAwake::NoValidApp))
                } else {
                    defmt::info!("good so far");
                    match self.write_register_no_data(Register::APP_START).await {
                        Err(e) => Err(ModeChangeError::new(self, e)),
                        Ok(_) => Ok(Ccs811Awake::create(self.i2c, self.address)),
                    }
                }
            }
        }
    }

    async fn verify_application(&mut self) -> nb::Result<(), ErrorAwake<E>> {
        let status = self.read_status().await.map_err(nb::Error::Other)?;
        let verified = (status & BitFlags::APP_VERIFY) != 0;
        if !verified {
            if self.in_progress == ActionInProgress::Verification {
                Err(nb::Error::WouldBlock)
            } else {
                let result = self
                    .i2c
                    .write(self.address, &[Register::APP_VERIFY])
                    .await
                    .map_err(ErrorAwake::I2C);
                match result {
                    Ok(_) => {
                        self.in_progress = ActionInProgress::Verification;
                        Err(nb::Error::WouldBlock)
                    }
                    Err(e) => Err(nb::Error::Other(e)),
                }
            }
        } else {
            self.in_progress = ActionInProgress::None;
            Ok(())
        }
    }

    async fn erase_application(&mut self) -> nb::Result<(), ErrorAwake<E>> {
        let status = self.read_status().await.map_err(nb::Error::Other)?;
        let erased = (status & BitFlags::APP_ERASE) != 0;
        if !erased {
            if self.in_progress == ActionInProgress::Erase {
                Err(nb::Error::WouldBlock)
            } else {
                let result = self
                    .i2c
                    .write(self.address, &[Register::APP_ERASE, 0xE7, 0xA7, 0xE6, 0x09])
                    .await
                    .map_err(ErrorAwake::I2C);
                match result {
                    Ok(_) => {
                        self.in_progress = ActionInProgress::Erase;
                        Err(nb::Error::WouldBlock)
                    }
                    Err(e) => Err(nb::Error::Other(e)),
                }
            }
        } else {
            self.in_progress = ActionInProgress::None;
            Ok(())
        }
    }

    async fn download_application(&mut self, bin: &[u8]) -> Result<(), ErrorAwake<E>> {
        if bin.len() % 8 != 0 {
            return Err(ErrorAwake::InvalidInputData);
        }
        let mut data = [0; 9];
        data[0] = Register::REG_BOOT_APP;
        for chunk in bin.chunks(8) {
            data[1..].copy_from_slice(chunk);
            self.i2c
                .write(self.address, &data)
                .await
                .map_err(ErrorAwake::I2C)?;
            Timer::after(Duration::from_millis(50)).await;
        }
        self.check_status_error().await
    }

    async fn update_application(&mut self, bin: &[u8]) -> Result<(), ErrorAwake<E>> {
        self.write_sw_reset().await?;
        Timer::after(Duration::from_millis(20)).await;
        loop {
            match self.erase_application().await {
                Err(nb::Error::WouldBlock) => {
                    Timer::after(Duration::from_millis(500)).await;
                }
                Err(nb::Error::Other(e)) => return Err(e),
                Ok(_) => break,
            }
        }
        self.download_application(bin).await?;
        loop {
            match self.verify_application().await {
                Err(nb::Error::WouldBlock) => {
                    Timer::after(Duration::from_millis(70)).await;
                }
                Err(nb::Error::Other(e)) => return Err(e),
                Ok(_) => break,
            }
        }
        Ok(())
    }

    // Note: is_verifying is false after a reset
    async fn software_reset(&mut self) -> Result<(), ErrorAwake<E>> {
        self.write_sw_reset().await
    }
}

use crate::{on_awaken, on_awaken_nb};

impl<I2C, CommE, PinE, NWAKE> Ccs811<I2C, NWAKE, mode::Boot>
where
    I2C: embassy_traits::i2c::I2c<Error = CommE>,
    NWAKE: OutputPin<Error = PinE>,
{
    pub async fn start_application(
        mut self,
    ) -> Result<Ccs811<I2C, NWAKE, mode::App>, ModeChangeError<Error<CommE, PinE>, Self>> {
        if let Err(e) = self.n_wake_pin.set_low() {
            defmt::info!("wake pin cannot be set low");
            return Err(ModeChangeError::new(self, Error::Pin(e)));
        }
        Timer::after(Duration::from_micros(50)).await;
        let Ccs811 {
            dev,
            mut n_wake_pin,
            ..
        } = self;
        let result = dev.start_application().await;
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

    async fn verify_application(&mut self) -> nb::Result<(), Error<CommE, PinE>> {
        on_awaken_nb!(self, self.dev.verify_application())
    }

    async fn erase_application(&mut self) -> nb::Result<(), Error<CommE, PinE>> {
        on_awaken_nb!(self, self.dev.erase_application())
    }

    async fn download_application(&mut self, bin: &[u8]) -> Result<(), Error<CommE, PinE>> {
        on_awaken!(self, self.dev.download_application(bin))
    }

    async fn update_application(&mut self, bin: &[u8]) -> Result<(), Error<CommE, PinE>> {
        on_awaken!(self, self.dev.update_application(bin,))
    }

    async fn software_reset(&mut self) -> Result<(), Error<CommE, PinE>> {
        on_awaken!(self, self.dev.software_reset())
    }
}
