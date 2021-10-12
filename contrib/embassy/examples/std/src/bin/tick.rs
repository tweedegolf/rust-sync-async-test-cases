#![feature(type_alias_impl_trait)]
#![allow(incomplete_features)]

use embassy::executor::Executor;
use embassy::time::{Duration, Timer};
use embassy::util::Forever;
use log::*;

#[embassy::task]
async fn run() {
    loop {
        info!("tick");
        Timer::after(Duration::from_secs(1)).await;
    }
}

static EXECUTOR: Forever<Executor> = Forever::new();

fn main() {
    env_logger::builder()
        .filter_level(log::LevelFilter::Debug)
        .format_timestamp_nanos()
        .init();

    let executor = EXECUTOR.put(Executor::new());
    executor.run(|spawner| {
        spawner.spawn(run()).unwrap();
    });
}
