#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

#[path = "../example_common.rs"]
mod example_common;

use embassy_stm32::gpio::NoPin;
use example_common::*;

use cortex_m_rt::entry;
use embassy_stm32::dac::{Channel, Dac, Value};

#[entry]
fn main() -> ! {
    info!("Hello World, dude!");

    let p = embassy_stm32::init(config());

    let mut dac = Dac::new(p.DAC1, p.PA4, NoPin);

    loop {
        for v in 0..=255 {
            unwrap!(dac.set(Channel::Ch1, Value::Bit8(to_sine_wave(v))));
            unwrap!(dac.trigger(Channel::Ch1));
        }
    }
}

use micromath::F32Ext;

fn to_sine_wave(v: u8) -> u8 {
    if v >= 128 {
        // top half
        let r = 3.14 * ((v - 128) as f32 / 128.0);
        (r.sin() * 128.0 + 127.0) as u8
    } else {
        // bottom half
        let r = 3.14 + 3.14 * (v as f32 / 128.0);
        (r.sin() * 128.0 + 127.0) as u8
    }
}
