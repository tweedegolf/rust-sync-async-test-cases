#![macro_use]

#[cfg_attr(eth_v1, path = "v1.rs")]
#[cfg_attr(eth_v2, path = "v2/mod.rs")]
mod _version;
pub mod lan8742a;

pub use _version::*;

/// Station Management Interface (SMI) on an ethernet PHY
///
/// # Safety
///
/// The methods cannot move out of self
pub unsafe trait StationManagement {
    /// Read a register over SMI.
    fn smi_read(&mut self, reg: u8) -> u16;
    /// Write a register over SMI.
    fn smi_write(&mut self, reg: u8, val: u16);
}

/// Traits for an Ethernet PHY
///
/// # Safety
///
/// The methods cannot move S
pub unsafe trait PHY {
    /// Reset PHY and wait for it to come out of reset.
    fn phy_reset<S: StationManagement>(sm: &mut S);
    /// PHY initialisation.
    fn phy_init<S: StationManagement>(sm: &mut S);
    /// Poll link to see if it is up and FD with 100Mbps
    fn poll_link<S: StationManagement>(sm: &mut S) -> bool;
}
