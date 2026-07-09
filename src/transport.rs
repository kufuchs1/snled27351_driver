//! Transport trait and concrete implementations for SNLED27351 bus
//! communication.
//!
//! The driver is generic over any type implementing [`Transport`], allowing
//! the same register logic to run over both SPI and I2C without duplication.
//! Select the appropriate transport for your hardware:
//!
//! - `spi::Controller` - wraps N
//!   [`SpiDevice`](embedded_hal_async::spi::SpiDevice)s (one per chip, each
//!   managing its own CS line) and one SDB pin.
//! - `i2c::Controller` - wraps an [`I2c`](embedded_hal_async::i2c::I2c) bus,
//!   one 7-bit address per chip, and an optional SDB pin (see [`NoSdb`]).
//!
//! When exactly one transport feature is enabled, its controller is also
//! re-exported as `transport::Controller` for convenience. When both features
//! are enabled (Cargo features are additive, so this can happen via feature
//! unification), refer to the controllers by their module paths
//! `transport::spi::Controller` and `transport::i2c::Controller`.
#[cfg(feature = "i2c")] pub mod i2c;
#[cfg(feature = "spi")] pub mod spi;
use core::{convert::Infallible, fmt::Debug};
#[cfg(any(feature = "i2c", feature = "spi"))]
use embassy_time::Timer;
use embedded_hal::digital::{ErrorType, OutputPin, PinState};
#[cfg(all(feature = "i2c", not(feature = "spi")))]
pub use i2c::Controller;
#[cfg(all(feature = "spi", not(feature = "i2c")))]
pub use spi::Controller;

/// Stand-in [`OutputPin`] for designs where the SDB line is strapped high in
/// hardware instead of being driven by the MCU.
///
/// With `NoSdb`, [`Transport::set_sdb`] and the SDB cycle in
/// [`Transport::reset`] become no-ops, and hardware sleep is unavailable.
#[derive(Clone, Copy, Debug, Default)]
#[non_exhaustive]
pub struct NoSdb;

impl NoSdb {
    /// Creates a new [`NoSdb`] placeholder pin.
    #[inline]
    #[must_use]
    pub const fn new() -> Self { Self }
}

impl ErrorType for NoSdb {
    type Error = Infallible;
}

impl OutputPin for NoSdb {
    #[inline]
    fn set_high(&mut self) -> Result<(), Infallible> { Ok(()) }

    #[inline]
    fn set_low(&mut self) -> Result<(), Infallible> { Ok(()) }

    #[inline]
    fn set_state(&mut self, _state: PinState) -> Result<(), Infallible> { Ok(()) }
}

/// Abstraction over the physical bus used to communicate with one or more
/// SNLED27351 driver chips.
///
/// Implementors handle all framing, CS/address management, and page selection
/// internally so the driver core only deals with register-level semantics.
///
/// # Page selection
///
/// - SPI encodes the page in the command byte header directly.
/// - I2C writes to the command register (`0xFD`) before each register access,
///   skipping the write when the page is already selected (permitted by
///   datasheet section 4.2).
///
/// Both cases are handled transparently inside the transport; the driver
/// core always calls [`write_page`](Transport::write_page) and
/// [`read_reg`](Transport::read_reg) with a plain page number.
pub trait Transport {
    /// The error type returned by bus operations.
    type Error: Debug;

    /// Selects `page` on chip `driver_index`, then reads and returns the
    /// single byte at register `reg`.
    #[expect(
        async_fn_in_trait,
        reason = "This crate targets single-core Cortex-M; Send bounds on futures are unnecessary"
    )]
    async fn read_reg(&mut self, driver_index: usize, page: u8, reg: u8) -> Result<u8, Self::Error>;

    /// Prepares the chips for register programming.
    ///
    /// Called once by [`Driver::init`](crate::driver::Driver::init) before
    /// any register writes. Both transports cycle the shared SDB line
    /// through hardware sleep and back to reach a known power state (a
    /// no-op with [`NoSdb`]). Note this is not a register reset: registers
    /// are retained across hardware sleep, and a full register reset
    /// requires the SYSRST pin (not managed by this crate). `init`
    /// reprograms every register anyway.
    #[expect(
        async_fn_in_trait,
        reason = "This crate targets single-core Cortex-M; Send bounds on futures are unnecessary"
    )]
    async fn reset(&mut self) -> Result<(), Self::Error>;

    /// Drives the shared SDB (hardware shutdown) line.
    ///
    /// `enable = true` releases the line for normal operation; `false`
    /// pulls it low, putting every chip into hardware sleep mode (registers
    /// retained but inaccessible). Transports whose SDB line is strapped in
    /// hardware (see [`NoSdb`]) treat this as a no-op.
    ///
    /// Callers are responsible for the datasheet timing: 16 µs to enter
    /// hardware sleep and 128 µs to wake from it. The driver methods
    /// [`hardware_sleep`](crate::driver::Driver::hardware_sleep) and
    /// [`hardware_wake`](crate::driver::Driver::hardware_wake) handle this.
    ///
    /// # Errors
    ///
    /// Returns `Err(Self::Error)` if driving the pin fails.
    fn set_sdb(&mut self, enable: bool) -> Result<(), Self::Error>;

    /// Selects `page` on chip `driver_index`, then writes `data` starting at
    /// register `reg`.
    ///
    /// `data` must not exceed
    /// [`PWM_REGISTER_COUNT`](crate::registers::PWM_REGISTER_COUNT) bytes.
    #[expect(
        async_fn_in_trait,
        reason = "This crate targets single-core Cortex-M; Send bounds on futures are unnecessary"
    )]
    async fn write_page(&mut self, driver_index: usize, page: u8, reg: u8, data: &[u8]) -> Result<(), Self::Error>;
}

/// Errors that can occur in the provided bus transports.
///
/// `E` is the underlying bus error type ([`SpiDevice::Error`] for SPI,
/// [`I2c::Error`] for I2C). Also available as `TransportError` from the
/// crate root and the transport submodules.
///
/// [`SpiDevice::Error`]: embedded_hal_async::spi::ErrorType::Error
/// [`I2c::Error`]: embedded_hal_async::i2c::ErrorType::Error
#[derive(Debug)]
#[non_exhaustive]
pub enum Error<E> {
    /// An underlying bus (SPI or I2C) error occurred.
    Bus(E),
    /// The driver index was out of range.
    InvalidIndex,
    /// The data payload exceeded the maximum transfer size.
    PayloadTooLarge,
    /// An `OutputPin` operation failed.
    Pin,
}

impl<E> From<E> for Error<E> {
    #[inline]
    fn from(error: E) -> Self { Self::Bus(error) }
}

/// Cycles the SDB line through hardware sleep and back with the datasheet
/// timing. Shared by the transports' [`Transport::reset`] implementations.
#[cfg(any(feature = "i2c", feature = "spi"))]
pub(crate) async fn cycle_sdb<P, E>(sdb: &mut P) -> Result<(), Error<E>>
where
    P: OutputPin,
{
    // Datasheet: entering hardware sleep takes 16 µs.
    drive_sdb(sdb, false)?;
    Timer::after_micros(250).await;
    drive_sdb(sdb, true)?;
    // Datasheet: 128 µs wakeup time before the first register access.
    Timer::after_micros(500).await;
    Ok(())
}

/// Drives an SDB pin, mapping pin failures to [`Error::Pin`]. Shared by the
/// transports' [`Transport::set_sdb`] implementations.
#[cfg(any(feature = "i2c", feature = "spi"))]
pub(crate) fn drive_sdb<P, E>(sdb: &mut P, enable: bool) -> Result<(), Error<E>>
where
    P: OutputPin,
{
    let result = if enable { sdb.set_high() } else { sdb.set_low() };
    if result.is_err() {
        return Err(Error::Pin);
    }
    Ok(())
}
