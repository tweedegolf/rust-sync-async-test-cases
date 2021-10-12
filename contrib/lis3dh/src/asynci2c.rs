use crate::Error;
use core::convert::Infallible;
// use crate::{register::Register, Configuration, BDU};
use crate::register::*;
use crate::Configuration;
use accelerometer::error::Error as AccelerometerError;
use accelerometer::vector::{F32x3, I16x3};
use core::convert::TryInto;
use core::pin::Pin;
use embassy_nrf::spim::{Instance, Spim};
use embassy_traits::delay::Delay;
use embassy_traits::i2c::I2c;
use embassy_traits::spi::FullDuplex;
use futures::pin_mut;

// use embedded_hal::digital::v2::OutputPin;
use async_output_pin::AsyncOutputPin;

pub struct Lis3dh<I2C> {
    /// Underlying I2C device
    i2c: I2C,

    /// Current I²C slave address
    address: u8,
}

type EPIN = ();

impl<I2C, EI2C> Lis3dh<I2C>
where
    I2C: I2c<Error = EI2C> + Unpin,
{
    /// Create a new LIS3DH driver from the given I2C peripheral.
    pub async fn new(
        i2c: I2C,
        address: SlaveAddr,
        conf: Configuration,
    ) -> Result<Self, Error<EI2C, EPIN>> {
        let mut lis3dh = Lis3dh {
            i2c,
            address: address.addr(),
        };

        lis3dh.init(conf).await?;

        Ok(lis3dh)
    }

    async fn init(&mut self, conf: Configuration) -> Result<(), Error<EI2C, EPIN>> {
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
    async fn get_device_id(&mut self) -> Result<u8, Error<EI2C, EPIN>> {
        self.read_register(Register::WHOAMI).await
    }

    pub async fn write(&mut self, buffer: &[u8]) -> Result<(), Error<EI2C, EPIN>> {
        match Pin::new(&mut self.i2c)
            .as_mut()
            .write(self.address, buffer)
            .await
        {
            Err(e) => Err(Error::Bus(e)),
            Ok(_) => Ok(()),
        }
    }

    pub async fn read(&mut self, buffer: &mut [u8]) -> Result<(), Error<EI2C, EPIN>> {
        match Pin::new(&mut self.i2c)
            .as_mut()
            .read(self.address, buffer)
            .await
        {
            Err(e) => Err(Error::Bus(e)),
            Ok(_) => Ok(()),
        }
    }

    pub async fn read_register(&mut self, register: Register) -> Result<u8, Error<EI2C, EPIN>> {
        let mut data = [0];

        Pin::new(&mut self.i2c)
            .as_mut()
            .write_read(self.address, &[register.addr()], &mut data)
            .await
            .map_err(Error::Bus)
            .and(Ok(data[0]))
    }

    /// Write a byte to the given register.
    async fn write_register(
        &mut self,
        register: Register,
        value: u8,
    ) -> Result<(), Error<EI2C, EPIN>> {
        if register.read_only() {
            return Err(Error::WriteToReadOnly);
        }

        self.i2c
            .write(self.address, &[register.addr(), value])
            .await
            .map_err(Error::Bus)
    }

    /// Full-scale selection.
    pub async fn set_range(&mut self, range: Range) -> Result<(), Error<EI2C, EPIN>> {
        self.modify_register(Register::CTRL4, |mut ctrl4| {
            // Mask off lowest 4 bits
            ctrl4 &= !FS_MASK;
            // Write in new full-scale to highest 4 bits
            ctrl4 |= range.bits() << 4;

            ctrl4
        })
        .await
    }

    /// Read from the registers for each of the 3 axes.
    pub async fn read_accel_bytes(&mut self) -> Result<[u8; 6], Error<EI2C, EPIN>> {
        let mut data = [0u8; 6];

        Pin::new(&mut self.i2c)
            .write_read(self.address, &[Register::OUT_X_L.addr() | 0x80], &mut data)
            .await
            .map_err(Error::Bus)
            .and(Ok(data))
    }

    pub async fn accel_raw(&mut self) -> Result<I16x3, Error<EI2C, EPIN>> {
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
    ) -> Result<(), Error<EI2C, EPIN>>
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
    ) -> Result<(), Error<EI2C, EPIN>> {
        self.modify_register(reg, |v| v & !bits).await
    }

    /// Set the given bits in the given register.
    async fn register_set_bits(
        &mut self,
        reg: Register,
        bits: u8,
    ) -> Result<(), Error<EI2C, EPIN>> {
        self.modify_register(reg, |v| v | bits).await
    }

    /// Set or clear the given given bits in the given register, depending on
    /// the value of `set`.
    async fn register_xset_bits(
        &mut self,
        reg: Register,
        bits: u8,
        set: bool,
    ) -> Result<(), Error<EI2C, EPIN>> {
        if set {
            self.register_set_bits(reg, bits).await
        } else {
            self.register_clear_bits(reg, bits).await
        }
    }

    async fn set_mode(&mut self, mode: Mode) -> Result<(), Error<EI2C, EPIN>> {
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
    pub async fn set_datarate(&mut self, datarate: DataRate) -> Result<(), Error<EI2C, EPIN>> {
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
    ) -> Result<(), Error<EI2C, EPIN>> {
        self.modify_register(Register::CTRL1, |mut ctrl1| {
            ctrl1 &= !(X_EN | Y_EN | Z_EN); // disable all axes

            ctrl1 |= if x { X_EN } else { 0 };
            ctrl1 |= if y { Y_EN } else { 0 };
            ctrl1 |= if z { Z_EN } else { 0 };

            ctrl1
        })
        .await
    }

    async fn enable_temp(&mut self, enable: bool) -> Result<(), Error<EI2C, EPIN>> {
        self.register_xset_bits(Register::TEMP_CFG, ADC_EN & TEMP_EN, enable)
            .await?;

        // enable block data update (required for temp reading)
        if enable {
            self.register_xset_bits(Register::CTRL4, BDU, true).await?;
        }

        Ok(())
    }

    /// Configure an IRQ source
    pub async fn configure_irq_src<I: Interrupt>(
        &mut self,
        _int: I,
        irq_src: InterruptSource,
        latch_en: bool,
        d4d_en: bool,
    ) -> Result<(), Error<EI2C, EPIN>> {
        if latch_en || d4d_en {
            self.register_set_bits(
                Register::CTRL5,
                (latch_en as u8) << I::lir_int_bit() | (d4d_en as u8) << I::d4d_int_bit(),
            )
            .await?;
        }
        self.write_register(I::cfg_reg(), irq_src.bits()).await
    }

    /// Configure one of the interrupt pins
    pub async fn configure_int_pin<P: IrqPin>(&mut self, pin: P) -> Result<(), Error<EI2C, EPIN>> {
        self.write_register(P::ctrl_reg(), pin.bits()).await
    }

    /// Configure a duration threshold for an interrupt
    pub async fn configure_irq_duration<I: Interrupt>(
        &mut self,
        _int: I,
        duration: u8,
    ) -> Result<(), Error<EI2C, EPIN>> {
        self.write_register(I::duration_reg(), duration).await
    }

    /// Configure a magnitude threshold for an interrupt
    pub async fn configure_irq_threshold<I: Interrupt>(
        &mut self,
        _int: I,
        threshold: u8,
    ) -> Result<(), Error<EI2C, EPIN>> {
        self.write_register(I::ths_reg(), threshold).await
    }
}
