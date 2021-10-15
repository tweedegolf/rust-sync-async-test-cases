# Sleepy test case

A series of test cases to evaluate async Rust on the nrf52840 in terms of power usage and ergonomics.
This is an experiment that uses unstable features only available on nightly rust. 

## Setup
```bash
rustup override set nightly

rustup target add thumbv7em-none-eabihf

rustup component add llvm-tools-preview

cargo install probe-run cargo-embed cargo-binutils flip-link
```
## Running

Take a look in [`.cargo/config.toml`](.cargo/config.toml), e.g.

```bash
cargo run --bin wfi --features="async"
```
