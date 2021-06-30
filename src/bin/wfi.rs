#![no_std]
#![no_main]

use cortex_m_rt::entry;
use panic_halt as _;
use nrf52840_hal::pac as _; // memory layout

#[entry]
fn main() -> ! {
    loop {
        cortex_m::asm::wfi();
    }
}