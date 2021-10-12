use crate::Error;
// use crate::{register::Register, Configuration, BDU};
use crate::register::*;
use crate::Configuration;
use accelerometer::error::Error as AccelerometerError;
use accelerometer::vector::{F32x3, I16x3};
use core::convert::TryInto;
use core::pin::Pin;
use embassy::time::{Duration, Timer};
use embassy_nrf::spim::{Instance, Spim};
use embassy_traits::delay::Delay;
use embassy_traits::spi::FullDuplex;
use futures::pin_mut;

// use embedded_hal::digital::v2::OutputPin;
use async_output_pin::AsyncOutputPin;

pub struct Lis3dh<'d, SPI, NSS, DELAY> {
    /// Underlying SPI device
    spi: SPI,
    /// Active-low slave-select pin
    nss: NSS,
    /// Blocking delay
    delay: Pin<&'d mut DELAY>,
}

impl<'d, SPI, NSS, ENSS, DELAY> Lis3dh<'d, SPI, NSS, DELAY>
where
    SPI: FullDuplex<u8> + Unpin,
    NSS: AsyncOutputPin<Error = ENSS> + Unpin,
    DELAY: Delay,
{
    type ESPI = <SPI as embassy_traits::spi::FullDuplex<u8>>::Error;

    pub async fn new(
        spi: SPI,
        nss: NSS,
        delay: Pin<&'d mut DELAY>,
    ) -> Result<Lis3dh<'d, SPI, NSS, DELAY>, Error<SPI::Error, ENSS>> {
        let mut lis3dh = Self { spi, nss, delay };
        lis3dh.init(Configuration::default()).await?;

        Ok(lis3dh)
    }

    async fn init(&mut self, conf: Configuration) -> Result<(), Error<SPI::Error, ENSS>> {
        let id = self.get_device_id().await?;

        if id != crate::register::DEVICE_ID {
            return Err(Error::WrongAddress);
        }

        if conf.block_data_update || conf.temp_en {
            // Block data update
            self.write_register(Register::CTRL4, BDU).await?;
        }

        self.set_mode(conf.mode).await?;

        self.set_datarate(conf.datarate).await?;

        self.enable_axis((conf.enable_x_axis, conf.enable_y_axis, conf.enable_z_axis))
            .await?;

        if conf.temp_en {
            self.enable_temp(true).await?;
        }

        // Enable ADCs.
        self.write_register(Register::TEMP_CFG, ADC_EN).await
    }

    /// `WHO_AM_I` register.
    async fn get_device_id(&mut self) -> Result<u8, Error<SPI::Error, ENSS>> {
        self.read_register(Register::WHOAMI).await
    }

    pub async fn write(&mut self, buffer: &[u8]) -> Result<(), Error<SPI::Error, ENSS>> {
        match Pin::new(&mut self.spi).as_mut().write(buffer).await {
            Err(e) => Err(Error::Bus(e)),
            Ok(_) => Ok(()),
        }
    }

    pub async fn read(&mut self, buffer: &mut [u8]) -> Result<(), Error<SPI::Error, ENSS>> {
        match Pin::new(&mut self.spi).as_mut().read(buffer).await {
            Err(e) => Err(Error::Bus(e)),
            Ok(_) => Ok(()),
        }
    }

    pub async fn read_register(
        &mut self,
        register: Register,
    ) -> Result<u8, Error<SPI::Error, ENSS>> {
        let mut data = [0];

        Pin::new(&mut self.nss)
            .set_low()
            .await
            .map_err(Error::Pin)?;

        Timer::after(Duration::from_micros(1)).await;

        core::sync::atomic::compiler_fence(core::sync::atomic::Ordering::SeqCst);

        // transfer data
        defmt::info!("got: {}", register.addr());
        self.write(&[register.addr() | 0x80]).await?;
        self.read(&mut data).await?;

        core::sync::atomic::compiler_fence(core::sync::atomic::Ordering::SeqCst);

        Timer::after(Duration::from_micros(1)).await;

        if let Err(e) = Pin::new(&mut self.nss).set_high().await {
            return Err(Error::Pin(e));
        }

        Ok(data[0])
    }

    /// Write a byte to the given register.
    async fn write_register(
        &mut self,
        register: Register,
        value: u8,
    ) -> Result<(), Error<SPI::Error, ENSS>> {
        if register.read_only() {
            return Err(Error::WriteToReadOnly);
        }
        unsafe { self.write_multiple_registers(register, &[value]).await }
    }

    pub async fn read_multiple_registers(
        &mut self,
        start_register: Register,
        buf: &mut [u8],
    ) -> Result<(), Error<SPI::Error, ENSS>> {
        Pin::new(&mut self.nss)
            .set_low()
            .await
            .map_err(Error::Pin)?;

        Timer::after(Duration::from_micros(1)).await;

        self.write(&[start_register.addr() | 0xC0]).await?;

        // transfer the buf
        self.write(buf).await?;
        self.read(buf).await?;

        core::sync::atomic::compiler_fence(core::sync::atomic::Ordering::SeqCst);

        Timer::after(Duration::from_micros(1)).await;

        if let Err(e) = Pin::new(&mut self.nss).set_high().await {
            return Err(Error::Pin(e));
        }

        Ok(())
    }

    /// Writes to many registers. Does not check whether all registers
    /// can be written to
    pub async unsafe fn write_multiple_registers(
        &mut self,
        start_register: Register,
        data: &[u8],
    ) -> Result<(), Error<SPI::Error, ENSS>> {
        Pin::new(&mut self.nss)
            .set_low()
            .await
            .map_err(Error::Pin)?;

        Timer::after(Duration::from_micros(1)).await;

        let res = match self.write(&[start_register.addr() | 0x40]).await {
            Err(e) => Err(e),
            Ok(_) => self.write(data).await,
        };

        core::sync::atomic::compiler_fence(core::sync::atomic::Ordering::SeqCst);

        Timer::after(Duration::from_micros(1)).await;

        Pin::new(&mut self.nss)
            .set_high()
            .await
            .map_err(Error::Pin)?;
        res
    }

    /// Read from the registers for each of the 3 axes.
    pub async fn read_accel_bytes(&mut self) -> Result<[u8; 6], Error<SPI::Error, ENSS>> {
        let mut data = [0u8; 6];
        self.read_multiple_registers(Register::OUT_X_L, &mut data)
            .await?;

        Ok(data)
    }

    pub async fn accel_raw(&mut self) -> Result<I16x3, Error<SPI::Error, ENSS>> {
        let accel_bytes = self.read_accel_bytes().await?;

        let x = i16::from_le_bytes(accel_bytes[0..2].try_into().unwrap());
        let y = i16::from_le_bytes(accel_bytes[2..4].try_into().unwrap());
        let z = i16::from_le_bytes(accel_bytes[4..6].try_into().unwrap());

        Ok(I16x3::new(x, y, z))
    }

    async fn modify_register<F>(
        &mut self,
        register: Register,
        f: F,
    ) -> Result<(), Error<SPI::Error, ENSS>>
    where
        F: FnOnce(u8) -> u8,
    {
        let value = self.read_register(register).await?;

        self.write_register(register, f(value)).await
    }

    /// Clear the given bits in the given register.
    async fn register_clear_bits(
        &mut self,
        reg: Register,
        bits: u8,
    ) -> Result<(), Error<SPI::Error, ENSS>> {
        self.modify_register(reg, |v| v & !bits).await
    }

    /// Set the given bits in the given register.
    async fn register_set_bits(
        &mut self,
        reg: Register,
        bits: u8,
    ) -> Result<(), Error<SPI::Error, ENSS>> {
        self.modify_register(reg, |v| v | bits).await
    }

    /// Set or clear the given given bits in the given register, depending on
    /// the value of `set`.
    async fn register_xset_bits(
        &mut self,
        reg: Register,
        bits: u8,
        set: bool,
    ) -> Result<(), Error<SPI::Error, ENSS>> {
        if set {
            self.register_set_bits(reg, bits).await
        } else {
            self.register_clear_bits(reg, bits).await
        }
    }

    async fn set_mode(&mut self, mode: Mode) -> Result<(), Error<SPI::Error, ENSS>> {
        match mode {
            Mode::LowPower => {
                self.register_set_bits(Register::CTRL1, LP_EN).await?;
                self.register_clear_bits(Register::CTRL4, HR).await?;
            }
            Mode::Normal => {
                self.register_clear_bits(Register::CTRL1, LP_EN).await?;
                self.register_clear_bits(Register::CTRL4, HR).await?;
            }
            Mode::HighResolution => {
                self.register_clear_bits(Register::CTRL1, LP_EN).await?;
                self.register_set_bits(Register::CTRL4, HR).await?;
            }
        }

        Ok(())
    }

    /// Data rate selection.
    async fn set_datarate(&mut self, datarate: DataRate) -> Result<(), Error<SPI::Error, ENSS>> {
        self.modify_register(Register::CTRL1, |mut ctrl1| {
            // Mask off lowest 4 bits
            ctrl1 &= !ODR_MASK;
            // Write in new output data rate to highest 4 bits
            ctrl1 |= datarate.bits() << 4;

            ctrl1
        })
        .await
    }

    async fn enable_axis(
        &mut self,
        (x, y, z): (bool, bool, bool),
    ) -> Result<(), Error<SPI::Error, ENSS>> {
        self.modify_register(Register::CTRL1, |mut ctrl1| {
            ctrl1 &= !(X_EN | Y_EN | Z_EN); // disable all axes

            ctrl1 |= if x { X_EN } else { 0 };
            ctrl1 |= if y { Y_EN } else { 0 };
            ctrl1 |= if z { Z_EN } else { 0 };

            ctrl1
        })
        .await
    }

    async fn enable_temp(&mut self, enable: bool) -> Result<(), Error<SPI::Error, ENSS>> {
        self.register_xset_bits(Register::TEMP_CFG, ADC_EN & TEMP_EN, enable)
            .await?;

        // enable block data update (required for temp reading)
        if enable {
            self.register_xset_bits(Register::CTRL4, BDU, true).await?;
        }

        Ok(())
    }
}
