#![no_std]
#![no_main]

use core::{
    mem::MaybeUninit,
    sync::atomic::{AtomicBool, Ordering},
};

use cortex_m_rt::entry;
use hal::{
    pac::{self, interrupt, Interrupt, NVIC},
    prelude::*,
    timer::Periodic,
    Timer,
};
use nrf52840_hal as hal; // memory layout
use panic_halt as _;
// use sleepy_test_case as _;

static mut TIMER0_HANDLE: MaybeUninit<Timer<pac::TIMER0, Periodic>> = MaybeUninit::uninit();
static TIMER0_FIRED: AtomicBool = AtomicBool::new(false);

#[entry]
fn main() -> ! {
    let peripherals = pac::Peripherals::take().unwrap();

    let mut timer0 = Timer::periodic(peripherals.TIMER0);
    {
        let timer0 = unsafe { &*pac::TIMER0::ptr() };
        timer0.prescaler.write(|w| unsafe { w.bits(0x09) });
    };

    timer0.enable_interrupt();
    timer0.start(500_000u32);

    unsafe {
        *(&mut *TIMER0_HANDLE.as_mut_ptr()) = timer0;
        NVIC::unmask(Interrupt::TIMER0);
    }

    // Busy wait
    while !TIMER0_FIRED.swap(false, Ordering::Relaxed) {}

    // Go to sleep
    while !TIMER0_FIRED.swap(false, Ordering::Relaxed) {
        cortex_m::asm::wfi();
    }
    // Go to system off mode
    peripherals
        .POWER
        .systemoff
        .write(|w| w.systemoff().set_bit());
    loop {
        cortex_m::asm::nop();
    }
}

#[interrupt]
fn TIMER0() {
    let timer0 = unsafe { &*TIMER0_HANDLE.as_ptr() };
    if timer0.event_compare_cc0().read().bits() != 0x00u32 {
        TIMER0_FIRED.store(true, Ordering::Relaxed);
        timer0.event_compare_cc0().write(|w| unsafe { w.bits(0) })
    }
}
