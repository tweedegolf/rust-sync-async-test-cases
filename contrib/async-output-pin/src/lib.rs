#![feature(generic_associated_types)]
#![feature(type_alias_impl_trait)]
#![no_std]

use core::future::Future;
use core::pin::Pin;

use embedded_hal::digital::v2::OutputPin;

use async_mvar::MVar;

pub trait AsyncOutputPin {
    /// Error type
    type Error;

    type SetLowFuture<'a>: Future<Output = Result<(), Self::Error>> + 'a
    where
        Self: 'a;

    type SetHighFuture<'a>: Future<Output = Result<(), Self::Error>> + 'a
    where
        Self: 'a;

    /// Drives the pin low
    ///
    /// *NOTE* the actual electrical state of the pin may not actually be low, e.g. due to external
    /// electrical sources
    fn set_low<'a>(self: Pin<&'a mut Self>) -> Self::SetLowFuture<'a>;

    /// Drives the pin high
    ///
    /// *NOTE* the actual electrical state of the pin may not actually be high, e.g. due to external
    /// electrical sources
    fn set_high<'a>(self: Pin<&'a mut Self>) -> Self::SetHighFuture<'a>;
}

use core::convert::Infallible;

#[derive(Default)]
pub struct Token(u64);

impl Token {
    pub const fn new() -> Self {
        Token(42)
    }
}

pub struct AsyncOutput<'d, T: embassy_nrf::gpio::Pin> {
    lock: &'d MVar<Token>,
    output: embassy_nrf::gpio::Output<'d, T>,
}

impl<'d, T: embassy_nrf::gpio::Pin> AsyncOutput<'d, T> {
    pub fn new(lock: &'d MVar<Token>, output: embassy_nrf::gpio::Output<'d, T>) -> Self {
        Self { lock, output }
    }
}

async fn set_low_help<'a, T>(zelf: Pin<&'a mut AsyncOutput<'_, T>>) -> Result<(), Infallible>
where
    T: embassy_nrf::gpio::Pin,
    T: core::marker::Unpin,
{
    let token = zelf.lock.take().await;

    core::sync::atomic::compiler_fence(core::sync::atomic::Ordering::SeqCst);

    let this = zelf.get_mut();
    this.output.set_low()?;

    core::sync::atomic::compiler_fence(core::sync::atomic::Ordering::SeqCst);

    Ok(())
}

async fn set_high_help<'a, T>(zelf: Pin<&'a mut AsyncOutput<'_, T>>) -> Result<(), Infallible>
where
    T: embassy_nrf::gpio::Pin,
    T: core::marker::Unpin,
{
    core::sync::atomic::compiler_fence(core::sync::atomic::Ordering::SeqCst);
    let this = zelf.get_mut();
    this.output.set_high()?;
    core::sync::atomic::compiler_fence(core::sync::atomic::Ordering::SeqCst);

    this.lock.put(Token::new()).await;

    Ok(())
}

impl<'d, T> AsyncOutputPin for AsyncOutput<'d, T>
where
    T: embassy_nrf::gpio::Pin,
    T: core::marker::Unpin,
{
    type Error = Infallible;

    #[rustfmt::skip]
    type SetLowFuture<'a> where Self: 'a = impl Future<Output = Result<(), Self::Error>> + 'a;

    #[rustfmt::skip]
    type SetHighFuture<'a> where Self: 'a = impl Future<Output = Result<(), Self::Error>> + 'a;

    fn set_low<'a>(self: Pin<&'a mut Self>) -> Self::SetLowFuture<'a> {
        set_low_help(self)
    }

    fn set_high<'a>(self: Pin<&'a mut Self>) -> Self::SetHighFuture<'a> {
        set_high_help(self)
    }
}
