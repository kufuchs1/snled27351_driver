//! SNLED27351 register addresses, page selectors, and configuration values.
//!
//! All constants are derived from the SNLED27351 datasheet. The SPI frame
//! format is: `WRITE_CMD | PATTERN_CMD | (page & 0x0F)`, register address,
//! then the data payload. I2C uses a prior write to [`CONFIGURE_CMD_PAGE`]
//! to select the page.

/// SPI command byte for write operations (bit 7 = 0).
pub const WRITE_CMD: u8 = 0x00;
/// SPI command byte for read operations (bit 7 = 1).
pub const READ_CMD: u8 = 0x80;
/// Checking pattern field of the SPI command byte: bits \[6:4\] = 0b010.
/// Fixed value required by the protocol; OR'd with `WRITE_CMD` or `READ_CMD`
/// and the page nibble to form the header byte.
pub const PATTERN_CMD: u8 = 2 << 4;

/// I2C register address used to select the active page.
/// Write the desired page number here before accessing any page registers.
pub const CONFIGURE_CMD_PAGE: u8 = 0xFD;

/// Page address for the LED control, open, and short registers (Page 0).
///
/// - 0x00-0x17: LED Control Register (on/off per channel, 24 bytes)
/// - 0x18-0x2F: LED Open Register   (open-detection status, 24 bytes)
/// - 0x30-0x47: LED Short Register  (short-detection status, 24 bytes)
pub const PAGE_LED_CONTROL: u8 = 0x00;
/// Page address for the PWM duty-cycle registers (Page 1).
///
/// 192 bytes (0x00-0xBF), one per output channel; 0 = off, 255 = full.
pub const PAGE_PWM: u8 = 0x01;
/// Page address for the function configuration registers (Page 3).
///
/// Contains shutdown, scan phase, slew rate, open/short detection, sleep, etc.
pub const PAGE_FUNCTION: u8 = 0x03;
/// Page address for the per-channel constant-current step registers (Page 4).
///
/// 12 bytes (0x00-0x0B), one per CB channel.
pub const PAGE_CURRENT_TUNE: u8 = 0x04;

/// Software shutdown control register (bit 0 = SSD).
pub const REG_SOFTWARE_SHUTDOWN: u8 = 0x00;
/// SSD = 0: driver enters shutdown mode; all outputs disabled.
pub const SOFTWARE_SHUTDOWN_SSD_SHUTDOWN: u8 = 0x00;
/// SSD = 1: driver operates normally (shutdown released).
pub const SOFTWARE_SHUTDOWN_SSD_NORMAL: u8 = 0x01;

/// LED driver ID register (read-only, default 0x8A).
pub const REG_ID: u8 = 0x11;
/// Expected value of [`REG_ID`] (`1000 1010b`).
///
/// Read the ID register and compare against this to confirm the chip is
/// powered and communicating.
pub const DRIVER_ID: u8 = 0x8A;

/// Thermal detector flag register (bit 0 = TDF, read-only).
///
/// TDF = 0: temperature is below 70 °C.
/// TDF = 1: temperature has reached or exceeded 70 °C.
pub const REG_THERMAL: u8 = 0x12;
/// Bit mask for the thermal detect flag (TDF, bit 0) in [`REG_THERMAL`].
pub const MSK_THERMAL_FLAG: u8 = 0x01;

/// Pull-down / pull-up resistor configuration register.
///
/// Bit layout: \[CAPD, -, CAPD2, -, CBPU, -, CBPU2, -\].
pub const REG_PULLDOWNUP: u8 = 0x13;
/// All four PDU control bits enabled (0xAA = datasheet recommended value).
pub const PULLDOWNUP_ALL_ENABLED: u8 = 0xAA;
/// Pull-down for CA1-CA16 on latch-data time (CAPD, bit 7).
pub const PULLDOWNUP_CAPD: u8 = 1 << 7;
/// Pull-down for CA1-CA16 on scan (PWM-off) time (CAPD2, bit 5).
pub const PULLDOWNUP_CAPD2: u8 = 1 << 5;
/// Pull-up for CB1-CB12 on latch-data time (CBPU, bit 3).
pub const PULLDOWNUP_CBPU: u8 = 1 << 3;
/// Pull-up for CB1-CB12 on scan time (CBPU2, bit 1).
pub const PULLDOWNUP_CBPU2: u8 = 1 << 1;

/// Scan-phase configuration register (bits \[3:0\] = SP).
///
/// Fewer active phases shorten the frame, raising the effective duty (and
/// brightness) of the remaining CB channels. Adjust the current tune
/// accordingly (see the datasheet note in section 6.9).
pub const REG_SCAN_PHASE: u8 = 0x14;
/// All 12 scan phases active (CB1-CB12, 1/12 duty).
pub const SCAN_PHASE_12_CHANNEL: u8 = 0x00;
/// CB1-CB11 active, 1/11 duty; CB12 inactive.
pub const SCAN_PHASE_11_CHANNEL: u8 = 0x01;
/// CB1-CB10 active, 1/10 duty; CB11-CB12 inactive.
pub const SCAN_PHASE_10_CHANNEL: u8 = 0x02;
/// CB1-CB9 active, 1/9 duty; CB10-CB12 inactive.
pub const SCAN_PHASE_9_CHANNEL: u8 = 0x03;
/// CB1-CB8 active, 1/8 duty; CB9-CB12 inactive.
pub const SCAN_PHASE_8_CHANNEL: u8 = 0x04;
/// CB1-CB7 active, 1/7 duty; CB8-CB12 inactive.
pub const SCAN_PHASE_7_CHANNEL: u8 = 0x05;
/// CB1-CB6 active, 1/6 duty; CB7-CB12 inactive.
pub const SCAN_PHASE_6_CHANNEL: u8 = 0x06;
/// CB1-CB5 active, 1/5 duty; CB6-CB12 inactive.
pub const SCAN_PHASE_5_CHANNEL: u8 = 0x07;
/// CB1-CB4 active, 1/4 duty; CB5-CB12 inactive.
pub const SCAN_PHASE_4_CHANNEL: u8 = 0x08;
/// CB1-CB3 active, 1/3 duty; CB4-CB12 inactive.
pub const SCAN_PHASE_3_CHANNEL: u8 = 0x09;
/// CB1-CB2 active, 1/2 duty; CB3-CB12 inactive.
pub const SCAN_PHASE_2_CHANNEL: u8 = 0x0A;
/// Only CB1 active, 1/1 duty; CB2-CB12 inactive.
pub const SCAN_PHASE_1_CHANNEL: u8 = 0x0B;

/// Slew-rate control mode 1 register (bit 2 = `PDP_EN`).
///
/// Note: the datasheet requires this register to be set to 0x00 or 0x04 only.
pub const REG_SLEW_RATE_CONTROL_MODE_1: u8 = 0x15;
/// PWM delay-phase enable (`PDP_EN`, bit 2).
pub const SLEW_RATE_CONTROL_MODE_1_PDP_ENABLE: u8 = 1 << 2;

/// Slew-rate control mode 2 register (bit 7 = DSL, bit 6 = SSL).
pub const REG_SLEW_RATE_CONTROL_MODE_2: u8 = 0x16;
/// Driving-channel (CA) slew-rate enable (DSL, bit 7).
pub const SLEW_RATE_CONTROL_MODE_2_DSL_ENABLE: u8 = 1 << 7;
/// Sinking-channel (CB) slew-rate enable (SSL, bit 6).
pub const SLEW_RATE_CONTROL_MODE_2_SSL_ENABLE: u8 = 1 << 6;
/// Both DSL and SSL enabled; value used in the datasheet sample code (0xC0).
pub const SLEW_RATE_CONTROL_MODE_2_ALL_ENABLE: u8 =
    SLEW_RATE_CONTROL_MODE_2_DSL_ENABLE | SLEW_RATE_CONTROL_MODE_2_SSL_ENABLE;

/// Open/short detection enable register.
/// Bit 7 = ODS (start open detection), bit 6 = SDS (start short detection).
/// Both bits are write-only and cleared automatically by hardware.
pub const REG_OPEN_SHORT_ENABLE: u8 = 0x17;
/// Start open-circuit detection (ODS, bit 7).
pub const OPEN_SHORT_ENABLE_ODS: u8 = 1 << 7;
/// Start short-circuit detection (SDS, bit 6).
pub const OPEN_SHORT_ENABLE_SDS: u8 = 1 << 6;

/// Open/short detection latched PWM duty register (OSDD).
///
/// Valid range is 0x01-0xFA; 0x00 is reserved. Writing this register
/// triggers a detection scan; hardware clears it automatically when done.
pub const REG_OPEN_SHORT_DUTY: u8 = 0x18;
/// Smallest valid OSDD value for [`REG_OPEN_SHORT_DUTY`].
pub const OPEN_SHORT_DUTY_MIN: u8 = 0x01;
/// Largest valid OSDD value for [`REG_OPEN_SHORT_DUTY`].
pub const OPEN_SHORT_DUTY_MAX: u8 = 0xFA;

/// Open/short detection completion flags register.
/// Bit 7 = ODINT (open done), bit 6 = SDINT (short done); set by hardware.
pub const REG_OPEN_SHORT_FLAG: u8 = 0x19;
/// Open-detection scan completed (ODINT, bit 7).
pub const OPEN_SHORT_FLAG_ODINT: u8 = 1 << 7;
/// Short-detection scan completed (SDINT, bit 6).
pub const OPEN_SHORT_FLAG_SDINT: u8 = 1 << 6;

/// Software sleep control register.
/// Bit 1 = SLEEP (write-only, cleared on wakeup), bit 0 = `IREF_EN` (keep 0).
pub const REG_SOFTWARE_SLEEP: u8 = 0x1A;
/// Normal operation (sleep disabled).
pub const SOFTWARE_SLEEP_DISABLE: u8 = 0x00;
/// Enter software sleep mode (SLEEP, bit 1).
/// Wakeup sources: SDB pin, SYSRST pin, or SCL/SCK level change.
pub const SOFTWARE_SLEEP_ENABLE: u8 = 1 << 1;

/// Number of LED-control registers per chip (24 = 0x18, addresses 0x00-0x17).
pub const LED_CONTROL_REGISTER_COUNT: usize = 0x18;
/// First LED open-detection status register in page 0 (0x18-0x2F).
pub const REG_LED_OPEN_FIRST: u8 = 0x18;
/// Number of LED open-detection status registers per chip (24 = 0x18).
pub const LED_OPEN_REGISTER_COUNT: usize = 0x18;
/// First LED short-detection status register in page 0 (0x30-0x47).
pub const REG_LED_SHORT_FIRST: u8 = 0x30;
/// Number of LED short-detection status registers per chip (24 = 0x18).
pub const LED_SHORT_REGISTER_COUNT: usize = 0x18;
/// Number of PWM registers per chip (192 = 0xC0, addresses 0x00-0xBF).
pub const PWM_REGISTER_COUNT: usize = 0xC0;
/// Number of current-tuning registers per chip (12 = 0x0C, addresses
/// 0x00-0x0B).
pub const CURRENT_TUNE_REGISTER_COUNT: usize = 0x0C;
