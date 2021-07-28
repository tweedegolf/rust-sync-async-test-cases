pub type SyncSensorCcs811<'a> = embedded_ccs811::Ccs811<
    nrf52840_hal::twim::Twim<nrf52840_hal::pac::TWIM0>,
    nrf52840_hal::gpio::Pin<nrf52840_hal::gpio::Output<nrf52840_hal::gpio::PushPull>>,
    nrf52840_hal::Delay,
    embedded_ccs811::mode::App,
>;

pub type SyncSensorLis3dh = lis3dh::i2c::Lis3dh<nrf52840_hal::twim::Twim<nrf52840_hal::pac::TWIM1>>;

pub fn go_to_sleep() -> ! {
    let sense_when_goes_to = Level::Low;
    configure_sense_pin(31, sense_when_goes_to);

    let sense_when_goes_to = Level::High;
    configure_sense_pin(2, sense_when_goes_to);

    cortex_m::asm::delay(100_000);

    let stolen_p = unsafe { nrf52840_hal::pac::Peripherals::steal() };
    stolen_p.POWER.systemoff.write(|w| w.systemoff().set_bit());

    cortex_m::asm::delay(1_000_000);

    loop {
        cortex_m::asm::nop();
    }
}

// fn configure_ram(_: nrf52840_hal::pac::POWER) {
pub fn configure_ram() {
    let block = unsafe { &*nrf52840_hal::pac::POWER::ptr() };

    block.ram7.power.write(|w| w.s1retention().on());
}

pub fn configure_sense_pin(id: usize, sense_when_goes_to: Level) {
    let p0_register_block = nrf52840_hal::pac::P0::ptr();

    cortex_m::peripheral::NVIC::mask(nrf52840_hal::pac::Interrupt::GPIOTE);

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

    unsafe {
        cortex_m::peripheral::NVIC::unmask(nrf52840_hal::pac::Interrupt::GPIOTE);
    }
}

pub enum Level {
    Low,
    High,
}
