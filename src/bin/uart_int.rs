#![no_std]
#![no_main]

//! Constantly writes 'Hello\r\n' over UARTE0,
//! using nrf52840-hal only for peripheral initialization.
//! It configures the UARTE1 interrupt to be called
//! when DMA transfer is done, and sleeps in between messages

use cortex_m_rt::entry;
use hal::{
    gpio::{p0::Parts, Level},
    pac::{self, interrupt},
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
    let uart = Uarte::new(
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
    // Prevent the uart handle from being dropped and thus being deinitalized
    core::mem::forget(uart);

    let uarte1 = unsafe { &*pac::UARTE1::ptr() };
    uarte1.intenset.write(|w| w.endtx().set_bit());

    unsafe {
        pac::NVIC::unmask(pac::Interrupt::UARTE1);
    }

    // Message must be in SRAM
    let msg = b"Hello!\r\n";
    let msg = *msg;

    uarte1
        .txd
        .ptr
        .write(|w| unsafe { w.ptr().bits(&msg as *const _ as u32) });
    uarte1
        .txd
        .maxcnt
        .write(|w| unsafe { w.maxcnt().bits(msg.len() as u16) });
    loop {
        uarte1.tasks_starttx.write(|w| w.tasks_starttx().set_bit());
        cortex_m::asm::wfi();
    }
}

#[interrupt]
fn UARTE1() {
    let uarte1 = unsafe { &*pac::UARTE1::ptr() };
    if uarte1.events_endtx.read().events_endtx().bit_is_set() {
        uarte1.events_endtx.write(|w| w.events_endtx().clear_bit());
    }
}
