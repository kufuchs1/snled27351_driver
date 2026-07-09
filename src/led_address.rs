//! SNLED27351 PWM register address computation.
//!
//! The PWM RAM (page 1) is laid out as a 16×12 matrix: each of the 12 CB
//! sink channels occupies a block of 16 consecutive registers, one per CA
//! source channel (datasheet sections 2.2 and 6.7).
//!
//! Address formula: `(cb - 1) * 0x10 + (ca - 1)` where both indices are
//! 1-based, exactly as printed in the datasheet RAM map.

/// Returns the PWM register address for the LED at sink channel `cb`
/// (CB1-CB12) and source channel `ca` (CA1-CA16), both 1-based as in the
/// datasheet.
///
/// # Example
///
/// ```ignore
/// use snled27351_driver::led_address::pwm_register;
///
/// assert_eq!(pwm_register(1, 1), 0x00); // CB1/CA1
/// assert_eq!(pwm_register(2, 1), 0x10); // CB2/CA1
/// assert_eq!(pwm_register(12, 16), 0xBF); // CB12/CA16
/// ```
///
/// # Panics
///
/// Panics if `cb` is not in `1..=12` or `ca` is not in `1..=16`. When called
/// in a const context (e.g. a `static` LED layout), this becomes a
/// compile-time error.
#[inline]
#[must_use]
pub const fn pwm_register(cb: u8, ca: u8) -> u8 {
    assert!(matches!(cb, 1..=12), "cb must be in 1..=12");
    assert!(matches!(ca, 1..=16), "ca must be in 1..=16");
    cb.wrapping_sub(1).wrapping_mul(0x10).wrapping_add(ca.wrapping_sub(1))
}
