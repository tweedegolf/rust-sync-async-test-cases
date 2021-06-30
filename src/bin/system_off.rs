#![no_std]
#![no_main]

use cortex_m_rt::entry;
use nrf52840_hal::pac;
use panic_halt as _; // memory layout

#[entry]
fn main() -> ! {
    let peripherals = pac::Peripherals::take().unwrap();
    peripherals
        .POWER
        .systemoff
        .write(|w| w.systemoff().set_bit());
    loop {
        cortex_m::asm::nop();
    }
}
