use embedded_ccs811::{mode, Ccs811};
use hal::pac;
use nrf52840_hal::{
    self as hal,
    gpio::{Output, Pin, PushPull},
    twim::Twim,
    Delay,
};
use pac::TWIM0;

pub type SyncSensorCcs811<'a> = Ccs811<Twim<TWIM0>, Pin<Output<PushPull>>, Delay, mode::App>;

pub type SyncSensorLis3dh = lis3dh::i2c::Lis3dh<Twim<pac::TWIM1>>;

pub fn go_to_sleep() -> ! {
    let sense_when_goes_to = Level::Low;
    configure_sense_pin(31, sense_when_goes_to);

    let sense_when_goes_to = Level::High;
    configure_sense_pin(2, sense_when_goes_to);

    cortex_m::asm::delay(100_000);

    let stolen_p = unsafe { pac::Peripherals::steal() };
    stolen_p.POWER.systemoff.write(|w| w.systemoff().set_bit());

    loop {
        cortex_m::asm::wfi();
    }
}

pub fn configure_ram() {
    let block = unsafe { &*pac::POWER::ptr() };

    block.ram7.power.write(|w| w.s1retention().on());
}

pub fn configure_sense_pin(id: usize, sense_when_goes_to: Level) {
    let p0_register_block = pac::P0::ptr();

    {
        unsafe { &(*p0_register_block).pin_cnf[id] }.write(|w| {
            w.dir().input();
            w.input().connect();
            match sense_when_goes_to {
                Level::Low => w.pull().pullup(),
                Level::High => w.pull().pulldown(),
            };
            w.drive().s0s1();

            match sense_when_goes_to {
                Level::Low => w.sense().low(),
                Level::High => w.sense().high(),
            };

            w
        });

        unsafe {
            // Clear latch
            (*p0_register_block).latch.write(|w| w.bits(0xFFFF_FFFF));
        };
    }
}

pub enum Level {
    Low,
    High,
}
