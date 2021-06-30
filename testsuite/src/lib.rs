#![no_std]
#![cfg_attr(test, no_main)]

use sleepy_test_case as _; // memory layout + panic handler

#[defmt_test::tests]
mod tests {}
