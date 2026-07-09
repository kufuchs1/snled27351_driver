//! I2C transport implementation for the SNLED27351 driver.
//!
//! Uses [`I2c`] with one 7-bit device address per driver chip (see the
//! `ADDRESS_*` constants), plus an optional shared SDB pin (use
//! [`NoSdb`] if SDB is strapped high in hardware).
//!
//! # Framing
//!
//! The SNLED27351 treats every I2C write frame as `[register, data...]`,
//! so each register access is up to two bus frames:
//!
//! 1. A write to the command register (`0xFD`) selecting the active page.
//!    Skipped when the page is already selected - the transport caches the
//!    selected page per chip, as permitted by datasheet section 4.2.
//! 2. The register write (or a write-read for register reads).
//!
//! The [`I2c::transaction`] contract merges adjacent write operations into
//! a single frame (no STOP or repeated START between them). This cuts both
//! ways:
//!
//! - The page select must be its own STOP-terminated frame: combined with the
//!   following access inside one `transaction()`, the chip would see `[0xFD,
//!   page, reg, data...]` and auto-increment the payload into the wrong
//!   registers.
//! - The data frame exploits the merge: the register byte and the payload are
//!   two `Write` operations in one transaction, which the bus emits as the
//!   single frame `[reg, data...]` without buffering the payload. This relies
//!   on the HAL implementing the transaction contract correctly.
//!
//! Splitting into two frames is still race-free for this driver: it holds
//! `&mut` access to the bus (or bus device) for the whole call, and traffic
//! to other devices on a shared bus does not affect this chip's page state.
pub use crate::transport::Error as TransportError;
use crate::{
    registers::{CONFIGURE_CMD_PAGE, PWM_REGISTER_COUNT},
    transport::{NoSdb, Transport, cycle_sdb, drive_sdb},
};
use embedded_hal::digital::OutputPin;
use embedded_hal_async::i2c::{I2c, Operation};

/// 7-bit device address when the ADDR/CS pin is connected to GND.
pub const ADDRESS_GND: u8 = 0x74;
/// 7-bit device address when the ADDR/CS pin is connected to SCL.
pub const ADDRESS_SCL: u8 = 0x75;
/// 7-bit device address when the ADDR/CS pin is connected to SDA.
pub const ADDRESS_SDA: u8 = 0x76;
/// 7-bit device address when the ADDR/CS pin is connected to VDDIO.
pub const ADDRESS_VDDIO: u8 = 0x77;

/// I2C transport for `N` SNLED27351 driver chips on a shared bus.
///
/// All chips share one [`I2c`] bus instance and are distinguished by their
/// 7-bit addresses in `addrs`. `S` is the shared SDB pin type.
pub struct Controller<B, S, const N: usize>
where
    B: I2c,
    S: OutputPin,
{
    /// 7-bit I2C addresses, one per driver chip, in driver-index order.
    address: [u8; N],
    /// The I2C bus peripheral.
    bus:     B,
    /// Last page selected on each chip; `None` when unknown.
    pages:   [Option<u8>; N],
    /// Shared hardware shutdown / enable pin (high = normal operation).
    sdb:     S,
}

impl<B, const N: usize> Controller<B, NoSdb, N>
where
    B: I2c,
{
    /// Creates a new [`Controller`] from the given I2C bus and address list,
    /// for designs where the SDB line is strapped high in hardware.
    ///
    /// `addrs` must be in the same order as the driver indices used by
    /// [`Driver`](crate::driver::Driver). Use
    /// [`with_sdb`](Controller::with_sdb) when the MCU drives the SDB line.
    #[inline]
    pub const fn new(bus: B, addrs: [u8; N]) -> Self { Self::with_sdb(bus, addrs, NoSdb) }
}

impl<B, S, const N: usize> Controller<B, S, N>
where
    B: I2c,
    S: OutputPin,
{
    /// Selects the active register page on chip `driver_index` by writing
    /// the page number to the command register (`0xFD`), and returns the
    /// chip's bus address for the follow-up transfer.
    ///
    /// The page write is skipped when the cached page already matches; see
    /// the module documentation for why it is a separate bus frame.
    async fn select_page(&mut self, driver_index: usize, page: u8) -> Result<u8, TransportError<B::Error>> {
        let Some(&address) = self.address.get(driver_index) else {
            return Err(TransportError::InvalidIndex);
        };
        let Some(cached) = self.pages.get_mut(driver_index) else {
            return Err(TransportError::InvalidIndex);
        };
        if *cached == Some(page) {
            return Ok(address);
        }
        // Invalidate first so a failed transfer never leaves a stale entry.
        *cached = None;
        self.bus.write(address, &[CONFIGURE_CMD_PAGE, page]).await?;
        *cached = Some(page);
        Ok(address)
    }

    /// Creates a new [`Controller`] from the given I2C bus, address list,
    /// and SDB pin.
    ///
    /// `addrs` must be in the same order as the driver indices used by
    /// [`Driver`](crate::driver::Driver).
    #[inline]
    pub const fn with_sdb(bus: B, addrs: [u8; N], sdb: S) -> Self {
        Self { address: addrs, bus, pages: [None; N], sdb }
    }
}

impl<B, S, const N: usize> Transport for Controller<B, S, N>
where
    B: I2c,
    S: OutputPin,
{
    type Error = TransportError<B::Error>;

    #[inline]
    async fn read_reg(&mut self, driver_index: usize, page: u8, reg: u8) -> Result<u8, Self::Error> {
        let address = self.select_page(driver_index, page).await?;

        // Write the register pointer, then read one byte after a repeated
        // START (datasheet section 4.2, steps 2 and 3).
        let mut buf = [0_u8; 1];
        self.bus.write_read(address, &[reg], &mut buf).await?;

        let [value] = buf;
        Ok(value)
    }

    #[inline]
    async fn reset(&mut self) -> Result<(), Self::Error> {
        // The page cache cannot be trusted across a power-state cycle.
        self.pages = [None; N];
        // Cycle SDB through hardware sleep and back to reach a known power
        // state (a no-op with `NoSdb`; registers are retained either way).
        cycle_sdb(&mut self.sdb).await
    }

    #[inline]
    fn set_sdb(&mut self, enable: bool) -> Result<(), Self::Error> { drive_sdb(&mut self.sdb, enable) }

    #[inline]
    async fn write_page(&mut self, driver_index: usize, page: u8, reg: u8, data: &[u8]) -> Result<(), Self::Error> {
        if data.len() > PWM_REGISTER_COUNT {
            return Err(TransportError::PayloadTooLarge);
        }

        let address = self.select_page(driver_index, page).await?;

        // Send [reg, payload...] as one frame without buffering: the
        // transaction contract merges adjacent write operations into a
        // single frame (no repeated START between them) - the same merge
        // rule that forces the page select above to be a separate frame.
        self.bus.transaction(address, &mut [Operation::Write(&[reg]), Operation::Write(data)]).await?;
        Ok(())
    }
}
