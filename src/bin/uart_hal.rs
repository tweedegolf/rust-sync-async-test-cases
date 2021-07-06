#![no_std]
#![no_main]

use cortex_m_rt::entry;
use hal::{
    gpio::{p0::Parts, Level},
    pac,
    uarte::{Baudrate, Parity, Pins},
    Uarte,
};
use nrf52840_hal as hal;
use panic_halt as _;

#[entry]
fn main() -> ! {
    let peripherals = pac::Peripherals::take().unwrap();

    let port0 = Parts::new(peripherals.P0);
    // Configure uarte1 using the HAL
    let mut uart = Uarte::new(
        peripherals.UARTE1,
        Pins {
            txd: port0.p0_06.into_push_pull_output(Level::Low).degrade(),
            rxd: port0.p0_08.into_floating_input().degrade(),
            cts: None,
            rts: None,
        },
        Parity::EXCLUDED,
        Baudrate::BAUD115200,
    );

    // Message must be in SRAM
    let msg = b"Hello!\r\n";
    let msg = *msg;

    loop {
        uart.write(&msg).unwrap();
    }
}
