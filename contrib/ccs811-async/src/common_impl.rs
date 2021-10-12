use crate::hal::{blocking::delay::DelayUs, digital::v2::OutputPin};
use crate::{
    hal, mode, ActionInProgress, BitFlags, Ccs811, Ccs811Awake, Ccs811Device, Error, ErrorAwake,
    FirmwareMode, ModeChangeError, Register, SlaveAddr,
};
use core::marker::PhantomData;

impl<I2C, NWAKE> Ccs811<I2C, NWAKE, mode::Boot> {
    /// Create new instance of the CCS811 device.
    ///
    /// See `Ccs811Awake` for the case where the nWAKE pin is not used.
    pub fn new(i2c: I2C, address: SlaveAddr, n_wake_pin: NWAKE) -> Self {
        Self::create(i2c, address.addr(), n_wake_pin)
    }
}

impl<I2C, NWAKE, MODE> Ccs811<I2C, NWAKE, MODE> {
    pub(crate) fn create(i2c: I2C, address: u8, n_wake_pin: NWAKE) -> Self {
        Self::from_awake_dev(Ccs811Awake::create(i2c, address), n_wake_pin)
    }

    pub(crate) fn from_awake_dev(dev: Ccs811Awake<I2C, MODE>, n_wake_pin: NWAKE) -> Self {
        Ccs811 {
            dev,
            n_wake_pin,
            _mode: PhantomData,
        }
    }
}

impl<I2C> Ccs811Awake<I2C, mode::Boot> {
    /// Create new instance of an already awake CCS811 device.
    pub fn new(i2c: I2C, address: SlaveAddr) -> Self {
        Self::create(i2c, address.addr())
    }
}

impl<I2C, MODE> Ccs811Awake<I2C, MODE> {
    pub(crate) fn create(i2c: I2C, address: u8) -> Self {
        Ccs811Awake {
            i2c,
            address,
            meas_mode_reg: 0,
            in_progress: ActionInProgress::None,
            _mode: PhantomData,
        }
    }
}

impl<I2C, E, MODE> Ccs811Awake<I2C, MODE>
where
    I2C: embassy_traits::i2c::I2c<Error = E>,
{
    /// Destroy driver instance, return I²C bus instance.
    pub fn destroy(self) -> I2C {
        self.i2c
    }
}
impl<I2C, E, MODE> Ccs811Awake<I2C, MODE>
where
    I2C: embassy_traits::i2c::I2c<Error = E>,
{
    pub(crate) async fn write_sw_reset(&mut self) -> Result<(), ErrorAwake<E>> {
        self.i2c
            .write(self.address, &[Register::SW_RESET, 0x11, 0xE5, 0x72, 0x8A])
            .await
            .map_err(ErrorAwake::I2C)
    }
}

impl<I2C, CommE, PinE, NWAKE, MODE> Ccs811<I2C, NWAKE, MODE>
where
    I2C: embassy_traits::i2c::I2c<Error = CommE>,
    NWAKE: OutputPin<Error = PinE>,
{
    /// Destroy driver instance, return I²C bus, nWAKE pin
    /// and wake delay instances.
    pub fn destroy(self) -> (I2C, NWAKE) {
        (self.dev.destroy(), self.n_wake_pin)
    }
}

impl<I2C, E, MODE> Ccs811Awake<I2C, MODE>
where
    I2C: embassy_traits::i2c::I2c<Error = E>,
{
    async fn firmware_mode(&mut self) -> Result<FirmwareMode, ErrorAwake<E>> {
        let status = self.read_status().await?;
        let mode = if (status & BitFlags::FW_MODE) != 0 {
            FirmwareMode::Application
        } else {
            FirmwareMode::Boot
        };
        Ok(mode)
    }

    pub async fn has_valid_app(&mut self) -> Result<bool, ErrorAwake<E>> {
        let status = self.read_status().await?;
        Ok((status & BitFlags::APP_VALID) != 0)
    }

    async fn hardware_id(&mut self) -> Result<u8, ErrorAwake<E>> {
        self.read_register_1byte(Register::HW_ID).await
    }

    async fn hardware_version(&mut self) -> Result<(u8, u8), ErrorAwake<E>> {
        let version = self.read_register_1byte(Register::HW_VERSION).await?;
        Ok(((version & 0xF0) >> 4, version & 0xF))
    }

    async fn firmware_bootloader_version(&mut self) -> Result<(u8, u8, u8), ErrorAwake<E>> {
        let version = self.read_register_2bytes(Register::FW_BOOT_VERSION).await?;
        Ok(((version[0] & 0xF0) >> 4, version[0] & 0xF, version[1]))
    }

    async fn firmware_application_version(&mut self) -> Result<(u8, u8, u8), ErrorAwake<E>> {
        let version = self.read_register_2bytes(Register::FW_APP_VERSION).await?;
        Ok(((version[0] & 0xF0) >> 4, version[0] & 0xF, version[1]))
    }
}

#[macro_export]
macro_rules! on_awaken {
    ($self:expr, $f:expr) => {{
        use embassy::time::{Duration, Timer};

        $self.n_wake_pin.set_low().map_err(Error::Pin)?;
        Timer::after(Duration::from_micros(50)).await;
        let result = $f.await.map_err(|e| e.into());
        $self.n_wake_pin.set_high().map_err(Error::Pin)?;
        Timer::after(Duration::from_micros(20)).await;
        result
    }};
}

#[macro_export]
macro_rules! on_awaken_nb {
    ($self:expr, $f:expr) => {{
        use embassy::time::{Duration, Timer};

        $self
            .n_wake_pin
            .set_low()
            .map_err(Error::Pin)
            .map_err(nb::Error::Other)?;
        Timer::after(Duration::from_micros(50)).await;
        let result = match $f.await {
            Ok(v) => Ok(v),
            Err(nb::Error::Other(e)) => Err(nb::Error::Other(e.into())),
            Err(nb::Error::WouldBlock) => Err(nb::Error::WouldBlock),
        };
        $self
            .n_wake_pin
            .set_high()
            .map_err(Error::Pin)
            .map_err(nb::Error::Other)?;
        Timer::after(Duration::from_micros(20)).await;
        result
    }};
}

impl<I2C, CommE, PinE, NWAKE, MODE> Ccs811<I2C, NWAKE, MODE>
where
    I2C: embassy_traits::i2c::I2c<Error = CommE>,
    NWAKE: OutputPin<Error = PinE>,
{
    async fn firmware_mode<'a>(&'a mut self) -> Result<FirmwareMode, Error<CommE, PinE>> {
        on_awaken!(self, self.dev.firmware_mode())
    }

    async fn has_valid_app(&mut self) -> Result<bool, Error<CommE, PinE>> {
        on_awaken!(self, self.dev.has_valid_app())
    }

    async fn hardware_id(&mut self) -> Result<u8, Error<CommE, PinE>> {
        on_awaken!(self, self.dev.hardware_id())
    }

    async fn hardware_version(&mut self) -> Result<(u8, u8), Error<CommE, PinE>> {
        on_awaken!(self, self.dev.hardware_version())
    }

    async fn firmware_bootloader_version(&mut self) -> Result<(u8, u8, u8), Error<CommE, PinE>> {
        on_awaken!(self, self.dev.firmware_bootloader_version())
    }

    async fn firmware_application_version(&mut self) -> Result<(u8, u8, u8), Error<CommE, PinE>> {
        on_awaken!(self, self.dev.firmware_application_version())
    }
}
