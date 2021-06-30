# Sleepy test case

A series of test cases to evaluate async Rust on the nrf52840 in terms of power usage and ergonomics.

## Setup
```bash
rustup target add thumbv7em-none-eabihf

rustup component add llvm-tools-preview

cargo install probe-run cargo-embed cargo-binutils flip-link

git submodule init

git submodule update
```
## Running

Take a look in [`.cargo/config.toml`](.cargo/config.toml)