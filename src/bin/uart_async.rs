#![no_std]
#![no_main]
#![feature(min_type_alias_impl_trait)]
#![feature(impl_trait_in_bindings)]
#![feature(type_alias_impl_trait)]
#![allow(incomplete_features)]

use embassy::executor::Spawner;
use embassy::traits::uart::{Read, Write};
use embassy_nrf::gpio::NoPin;
use embassy_nrf::{interrupt, uarte, Peripherals};

use panic_halt as _;

#[embassy::main]
async fn main(_spawner: Spawner, p: Peripherals) {
    let mut config = uarte::Config::default();
    config.parity = uarte::Parity::EXCLUDED;
    config.baudrate = uarte::Baudrate::BAUD115200;

    let irq = interrupt::take!(UARTE0_UART0);
    let mut uart =
        unsafe { uarte::Uarte::new(p.UARTE0, irq, p.P0_08, p.P0_06, NoPin, NoPin, config) };

    // Message must be in SRAM
    let msg = b"Hello!\r\n";
    let msg = *msg;

    uart.write(&msg).await.unwrap();

    loop {
        uart.write(&msg).await.unwrap();
    }
}
