//! Async embedded-hal driver for the SNLED27351 matrix LED
//! driver IC.
//!
//! Supports both SPI and I2C transports via a common [`transport::Transport`]
//! trait. The driver core in [`driver`] is generic over any
//! [`transport::Transport`] implementation and never duplicates register logic
//! between transports. Enable the `spi` and/or `i2c` Cargo feature to pull in
//! the corresponding transport.
//!
//! # Example
//!
//! ```ignore
//! use snled27351_driver::{pwm_register, transport::i2c, Driver, Led};
//!
//! // Two RGB LEDs on chip 0: red on CB1, green on CB3, blue on CB2.
//! static LEDS: [Led; 2] = [
//!     Led::new(0, pwm_register(1, 1), pwm_register(3, 1), pwm_register(2, 1)),
//!     Led::new(0, pwm_register(1, 2), pwm_register(3, 2), pwm_register(2, 2)),
//! ];
//!
//! let transport = i2c::Controller::new(i2c_bus, [i2c::ADDRESS_GND]);
//! let mut driver: Driver<_, 1> = Driver::new(transport, &LEDS);
//! driver.init(0xCC).await?; // 0xCC ≈ 32 mA constant current
//! driver.set_led(0, 255, 0, 128).await?;
//! ```
#![no_std]
pub mod driver;
pub mod led_address;
pub mod registers;
pub mod transport;
pub use driver::{Driver, Led, LedStatusMap};
pub use led_address::pwm_register;
pub use transport::{Error as TransportError, NoSdb, Transport};
