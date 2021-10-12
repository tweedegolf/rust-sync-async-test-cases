#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

#[path = "../example_common.rs"]
mod example_common;

use example_common::*;

use embassy::executor::Spawner;
use embassy_stm32::usart::{Config, Uart};
use embassy_stm32::Peripherals;
use embassy_traits::uart::{Read, Write};

#[embassy::main]
async fn main(_spawner: Spawner, p: Peripherals) {
    let mut usart = Uart::new(
        p.USART1,
        p.PB7,
        p.PB6,
        p.DMA1_CH2,
        p.DMA1_CH3,
        Config::default(),
    );

    usart.write(b"Hello Embassy World!\r\n").await.unwrap();
    info!("wrote Hello, starting echo");

    let mut buf = [0; 1];
    loop {
        usart.read(&mut buf[..]).await.unwrap();
        usart.write(&buf[..]).await.unwrap();
    }
}
