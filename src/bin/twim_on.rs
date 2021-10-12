#![no_std]
#![no_main]

use cortex_m_rt::entry;
use hal::{gpio::p0::Parts, pac, twim};
use nrf52840_hal as hal;
use panic_halt as _; // memory layout

#[entry]
fn main() -> ! {
    let peripherals = pac::Peripherals::take().unwrap();

    let port0 = Parts::new(peripherals.P0);

    let twim0_scl = port0.p0_28.into_floating_input().degrade();
    let twim0_sda = port0.p0_29.into_floating_input().degrade();
    let _twim0 = twim::Twim::new(
        peripherals.TWIM0,
        twim::Pins {
            scl: twim0_scl,
            sda: twim0_sda,
        },
        twim::Frequency::K400,
    );

    let twim1_scl = port0.p0_04.into_floating_input().degrade();
    let twim1_sda = port0.p0_03.into_floating_input().degrade();
    let _twim1 = twim::Twim::new(
        peripherals.TWIM1,
        twim::Pins {
            scl: twim1_scl,
            sda: twim1_sda,
        },
        twim::Frequency::K400,
    );

    loop {
        cortex_m::asm::wfi();
    }
}
