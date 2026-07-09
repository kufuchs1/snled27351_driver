//! SNLED27351 LED driver.
//!
//! Generic over a [`Transport`] `T` so the same driver logic covers both
//! SPI and I2C without any duplication. `N` is the number of driver chips
//! and must match the number of devices managed by the transport.
use crate::{
    registers::{
        CURRENT_TUNE_REGISTER_COUNT,
        LED_CONTROL_REGISTER_COUNT,
        LED_OPEN_REGISTER_COUNT,
        MSK_THERMAL_FLAG,
        OPEN_SHORT_DUTY_MAX,
        OPEN_SHORT_DUTY_MIN,
        OPEN_SHORT_ENABLE_ODS,
        OPEN_SHORT_ENABLE_SDS,
        OPEN_SHORT_FLAG_ODINT,
        OPEN_SHORT_FLAG_SDINT,
        PAGE_CURRENT_TUNE,
        PAGE_FUNCTION,
        PAGE_LED_CONTROL,
        PAGE_PWM,
        PULLDOWNUP_ALL_ENABLED,
        PWM_REGISTER_COUNT,
        REG_ID,
        REG_LED_OPEN_FIRST,
        REG_LED_SHORT_FIRST,
        REG_OPEN_SHORT_DUTY,
        REG_OPEN_SHORT_ENABLE,
        REG_OPEN_SHORT_FLAG,
        REG_PULLDOWNUP,
        REG_SCAN_PHASE,
        REG_SLEW_RATE_CONTROL_MODE_1,
        REG_SLEW_RATE_CONTROL_MODE_2,
        REG_SOFTWARE_SHUTDOWN,
        REG_SOFTWARE_SLEEP,
        REG_THERMAL,
        SCAN_PHASE_12_CHANNEL,
        SLEW_RATE_CONTROL_MODE_1_PDP_ENABLE,
        SLEW_RATE_CONTROL_MODE_2_ALL_ENABLE,
        SOFTWARE_SHUTDOWN_SSD_NORMAL,
        SOFTWARE_SHUTDOWN_SSD_SHUTDOWN,
        SOFTWARE_SLEEP_DISABLE,
        SOFTWARE_SLEEP_ENABLE,
    },
    transport::Transport,
};
use embassy_time::Timer;

/// Function-page `(register, value)` pairs written to every chip during
/// [`Driver::init`], in write order.
const INIT_FUNCTION_CONFIG: [(u8, u8); 5] = [
    (REG_PULLDOWNUP, PULLDOWNUP_ALL_ENABLED),
    (REG_SCAN_PHASE, SCAN_PHASE_12_CHANNEL),
    (REG_SLEW_RATE_CONTROL_MODE_1, SLEW_RATE_CONTROL_MODE_1_PDP_ENABLE),
    (REG_SLEW_RATE_CONTROL_MODE_2, SLEW_RATE_CONTROL_MODE_2_ALL_ENABLE),
    (REG_SOFTWARE_SLEEP, SOFTWARE_SLEEP_DISABLE),
];

/// Poll interval while waiting for an open/short detection scan to finish.
///
/// One full matrix re-scan takes ~418.5 µs (datasheet section 7.1).
const DETECTION_POLL_INTERVAL_US: u64 = 500;
/// Maximum number of completion-flag polls before a detection scan is
/// considered timed out (~25 ms total).
const DETECTION_POLL_ATTEMPTS: u32 = 50;

/// Mapping of a single RGB LED to its three PWM register addresses on one
/// driver chip.
#[derive(Copy, Clone, Debug)]
#[non_exhaustive]
pub struct Led {
    /// PWM register address for the blue channel.
    blue:   u8,
    /// Index of the driver chip controlling this LED.
    driver: u8,
    /// PWM register address for the green channel.
    green:  u8,
    /// PWM register address for the red channel.
    red:    u8,
}

impl Led {
    /// Returns the PWM register address for the blue channel.
    #[inline]
    #[must_use]
    pub const fn blue(self) -> u8 { self.blue }

    /// Returns the index of the driver chip controlling this LED.
    ///
    /// The index corresponds to the position in the transport's device array
    /// (e.g. the `addrs` slice for I2C or the `devices` array for SPI).
    #[inline]
    #[must_use]
    pub const fn driver(self) -> u8 { self.driver }

    /// Returns the PWM register address for the green channel.
    #[inline]
    #[must_use]
    pub const fn green(self) -> u8 { self.green }

    /// Creates a new [`Led`] with the given driver index and channel register
    /// addresses.
    ///
    /// `driver` is the zero-based index of the chip that controls this LED,
    /// matching the order of devices passed to the transport constructor.
    /// `red`, `green`, and `blue` are the raw PWM register addresses on that
    /// chip, as computed by
    /// [`pwm_register`](crate::led_address::pwm_register).
    ///
    /// # Example
    ///
    /// ```ignore
    /// use snled27351_driver::driver::Led;
    /// use snled27351_driver::led_address::pwm_register;
    ///
    /// // Red on CB1/CA1, green on CB3/CA1, blue on CB2/CA1.
    /// const MY_LED: Led = Led::new(0, pwm_register(1, 1), pwm_register(3, 1), pwm_register(2, 1));
    /// ```
    #[inline]
    #[must_use]
    pub const fn new(driver: u8, red: u8, green: u8, blue: u8) -> Self { Self { blue, driver, green, red } }

    /// Returns the PWM register address for the red channel.
    #[inline]
    #[must_use]
    pub const fn red(self) -> u8 { self.red }
}

/// Per-LED status bitmap returned by an open/short detection scan.
///
/// Wraps the 24-byte LED open (or short) register block of one chip. Bits
/// are addressed by PWM register address, so results can be queried with the
/// same addresses used to build the [`Led`] layout.
#[derive(Copy, Clone, Debug)]
pub struct LedStatusMap {
    /// Raw status register contents; index is the offset within the block.
    flags: [u8; LED_OPEN_REGISTER_COUNT],
}

impl LedStatusMap {
    /// Returns `true` if the channel with PWM register address `pwm_addr`
    /// was flagged by the scan.
    ///
    /// Results are only valid for channels whose LED control bit was
    /// enabled during the scan (datasheet section 6.8); disabled channels
    /// always read as unflagged. Out-of-range addresses return `false`.
    #[inline]
    #[must_use]
    pub fn is_flagged(&self, pwm_addr: u8) -> bool {
        // One status bit per channel: byte = addr / 8, bit = addr % 8.
        let byte = usize::from(pwm_addr >> 3_u8);
        let mask = 1_u8 << (pwm_addr & 0x07);
        self.flags.get(byte).is_some_and(|&status_bits| status_bits & mask != 0)
    }

    /// Returns the raw 24-byte status register block.
    #[inline]
    #[must_use]
    pub const fn raw(&self) -> &[u8; LED_OPEN_REGISTER_COUNT] { &self.flags }
}

/// PWM shadow buffer and dirty flag for a single driver chip.
struct DriverBuf {
    /// Whether this driver's buffer has unsent changes.
    dirty: bool,
    /// PWM shadow buffer; index is the raw register address.
    pwm:   [u8; PWM_REGISTER_COUNT],
}

impl DriverBuf {
    /// Creates a new zeroed, clean [`DriverBuf`].
    const fn new() -> Self { Self { dirty: false, pwm: [0_u8; PWM_REGISTER_COUNT] } }

    /// Writes RGB values into the shadow buffer for one LED and marks dirty.
    fn stage(&mut self, led: Led, red: u8, green: u8, blue: u8) {
        for (addr, value) in [(led.red, red), (led.green, green), (led.blue, blue)] {
            if let Some(slot) = self.pwm.get_mut(usize::from(addr)) {
                *slot = value;
            }
        }
        self.dirty = true;
    }
}

/// SNLED27351 LED driver.
///
/// Generic over a [`Transport`] `T` so the same driver logic covers both
/// SPI and I2C without any duplication. `N` is the number of driver chips
/// and must match the number of devices managed by the transport.
///
/// # Usage
///
/// ```ignore
/// let transport = spi::Controller::new(devices, sdb_pin);
/// let mut driver = Driver::new(transport, &LED_LAYOUT);
/// driver.init(0xFF).await?;
/// driver.set_all_leds(255, 255, 255).await?;
/// ```
///
/// # Power states
///
/// The chip distinguishes three low-power states (datasheet section 3):
///
/// - [`shutdown`](Self::shutdown): software shutdown - outputs off, registers
///   accessible (~2.1 mA).
/// - [`software_sleep`](Self::software_sleep): software sleep - lowest power
///   (~1.5 µA), registers inaccessible, woken by bus activity.
/// - [`hardware_sleep`](Self::hardware_sleep): SDB pin low - same power as
///   software sleep, woken only by the SDB pin.
///
/// [`wake`](Self::wake) returns to normal mode from software shutdown or
/// software sleep; [`hardware_wake`](Self::hardware_wake) from hardware
/// sleep.
pub struct Driver<T, const N: usize>
where
    T: Transport,
{
    /// PWM shadow buffers, one per chip.
    bufs:      [DriverBuf; N],
    /// LED layout mapping logical indices to driver channels.
    leds:      &'static [Led],
    /// The underlying bus transport.
    transport: T,
}

impl<T, const N: usize> Driver<T, N>
where
    T: Transport,
{
    /// Reads the thermal detector flag (TDF) from driver chip `index`.
    ///
    /// Returns `Ok(true)` if the chip reports temperature >= 70 °C.
    /// (The chip additionally shuts itself down at 150 °C.)
    ///
    /// # Errors
    ///
    /// Returns `Err(T::Error)` if the index is out of range or the bus read
    /// fails.
    #[inline]
    pub async fn check_thermal_flag_set(&mut self, index: usize) -> Result<bool, T::Error> {
        let thermal_reg = self.transport.read_reg(index, PAGE_FUNCTION, REG_THERMAL).await?;
        Ok(thermal_reg & MSK_THERMAL_FLAG != 0)
    }

    /// Runs an open-circuit detection scan on driver chip `index`.
    ///
    /// `duty` is the latched PWM duty used during the scan (clamped to the
    /// valid 0x01-0xFA range). The chip must be initialized and in normal
    /// operating mode; only channels enabled in the LED control register
    /// are tested ([`init`](Self::init) enables all of them).
    ///
    /// Returns `Ok(None)` if the scan did not complete within ~25 ms.
    ///
    /// # Errors
    ///
    /// Returns `Err(T::Error)` if any bus transaction fails.
    #[inline]
    pub async fn detect_open(&mut self, index: usize, duty: u8) -> Result<Option<LedStatusMap>, T::Error> {
        self.run_detection(index, duty, OPEN_SHORT_ENABLE_ODS, OPEN_SHORT_FLAG_ODINT, REG_LED_OPEN_FIRST).await
    }

    /// Runs a short-circuit detection scan on driver chip `index`.
    ///
    /// See [`detect_open`](Self::detect_open) for parameters, prerequisites,
    /// and timeout behavior.
    ///
    /// # Errors
    ///
    /// Returns `Err(T::Error)` if any bus transaction fails.
    #[inline]
    pub async fn detect_short(&mut self, index: usize, duty: u8) -> Result<Option<LedStatusMap>, T::Error> {
        self.run_detection(index, duty, OPEN_SHORT_ENABLE_SDS, OPEN_SHORT_FLAG_SDINT, REG_LED_SHORT_FIRST).await
    }

    /// Flushes all dirty PWM shadow buffers to hardware.
    ///
    /// Only drivers with pending changes are written, so calling this after
    /// a no-op `stage_led` is free.
    ///
    /// # Errors
    ///
    /// Returns `Err(T::Error)` if a bus transaction fails while writing a
    /// dirty buffer. Buffers that were successfully flushed before the error
    /// are marked clean; the failed buffer remains dirty and will be retried
    /// on the next call.
    #[inline]
    pub async fn flush(&mut self) -> Result<(), T::Error> {
        // Borrow the buffers and the transport as disjoint fields so the
        // shadow buffer can be written directly from `self.bufs` while
        // `self.transport` is borrowed mutably for the bus call.
        let transport = &mut self.transport;
        for (chip_index, buf) in self.bufs.iter_mut().enumerate() {
            if !buf.dirty {
                continue;
            }
            transport.write_page(chip_index, PAGE_PWM, 0x00, &buf.pwm).await?;
            buf.dirty = false;
        }
        Ok(())
    }

    /// Puts all driver chips into hardware sleep mode by pulling the shared
    /// SDB line low.
    ///
    /// Enters software shutdown first (mirroring the datasheet power state
    /// machine), then drives SDB low. Registers are retained but cannot be
    /// accessed until [`hardware_wake`](Self::hardware_wake) is called.
    /// With [`NoSdb`](crate::transport::NoSdb) only the software shutdown
    /// takes effect.
    ///
    /// # Errors
    ///
    /// Returns `Err(T::Error)` if a bus transaction or the SDB pin fails.
    #[inline]
    pub async fn hardware_sleep(&mut self) -> Result<(), T::Error> {
        self.write_function_all(REG_SOFTWARE_SHUTDOWN, SOFTWARE_SHUTDOWN_SSD_SHUTDOWN).await?;
        self.transport.set_sdb(false)?;
        // Datasheet: entering hardware sleep takes 16 µs.
        Timer::after_micros(20).await;
        Ok(())
    }

    /// Wakes all driver chips from hardware sleep mode and restores normal
    /// operation.
    ///
    /// Releases the SDB line, waits out the 128 µs wakeup time, and then
    /// releases software shutdown.
    ///
    /// # Errors
    ///
    /// Returns `Err(T::Error)` if a bus transaction or the SDB pin fails.
    #[inline]
    pub async fn hardware_wake(&mut self) -> Result<(), T::Error> {
        self.transport.set_sdb(true)?;
        // Datasheet: system wakeup time is 128 µs.
        Timer::after_micros(150).await;
        self.write_function_all(REG_SOFTWARE_SHUTDOWN, SOFTWARE_SHUTDOWN_SSD_NORMAL).await
    }

    /// Initializes all `N` driver chips.
    ///
    /// Performs the reset sequence via the transport, then for each chip:
    /// configures the function page registers, clears all LEDs and PWM
    /// values, programs current tune, enables all channels, and releases
    /// shutdown. The sequence and register values follow the datasheet
    /// sample code (section 8.8).
    ///
    /// `current_tune` sets the constant current step for all CB channels
    /// (`Iout = current_tune × 0.157 mA`; the datasheet default 0xCC ≈
    /// 32 mA, 0xFF ≈ 40 mA).
    ///
    /// # Errors
    ///
    /// Returns `Err(T::Error)` if any bus transaction fails during
    /// initialization. The chip state is undefined if this occurs.
    #[inline]
    pub async fn init(&mut self, current_tune: u8) -> Result<(), T::Error> {
        self.transport.reset().await?;
        // Power-on stabilization before the first register access; the
        // datasheet sample code waits 25 ms after releasing SDB/SYSRST,
        // which also covers the 20 ms SYSRST system reset time.
        Timer::after_millis(25).await;

        let transport = &mut self.transport;
        for (chip_index, buf) in self.bufs.iter_mut().enumerate() {
            // Enter shutdown so registers can be safely programmed.
            transport
                .write_page(chip_index, PAGE_FUNCTION, REG_SOFTWARE_SHUTDOWN, &[SOFTWARE_SHUTDOWN_SSD_SHUTDOWN])
                .await?;

            // Configure function registers.
            for &(reg, value) in &INIT_FUNCTION_CONFIG {
                transport.write_page(chip_index, PAGE_FUNCTION, reg, &[value]).await?;
            }

            // Clear LED control registers (all off).
            transport.write_page(chip_index, PAGE_LED_CONTROL, 0x00, &[0x00_u8; LED_CONTROL_REGISTER_COUNT]).await?;

            // Zero all PWM registers and sync the shadow buffer.
            transport.write_page(chip_index, PAGE_PWM, 0x00, &[0x00_u8; PWM_REGISTER_COUNT]).await?;
            buf.pwm.fill(0x00);
            buf.dirty = false;

            // Program current tune for all CB channels.
            transport
                .write_page(chip_index, PAGE_CURRENT_TUNE, 0x00, &[current_tune; CURRENT_TUNE_REGISTER_COUNT])
                .await?;

            // Enable all LED channels.
            transport.write_page(chip_index, PAGE_LED_CONTROL, 0x00, &[0xFF_u8; LED_CONTROL_REGISTER_COUNT]).await?;

            // Release shutdown - chip enters normal operating mode.
            transport
                .write_page(chip_index, PAGE_FUNCTION, REG_SOFTWARE_SHUTDOWN, &[SOFTWARE_SHUTDOWN_SSD_NORMAL])
                .await?;
        }

        Ok(())
    }

    /// Consumes the driver and returns the underlying transport.
    #[inline]
    #[must_use]
    pub fn into_transport(self) -> T { self.transport }

    /// Creates a new driver from the given transport and LED layout.
    ///
    /// Call [`init`](Self::init) before writing any LED values.
    #[inline]
    pub const fn new(transport: T, leds: &'static [Led]) -> Self {
        Self { transport, leds, bufs: [const { DriverBuf::new() }; N] }
    }

    /// Reads the ID register of driver chip `index`.
    ///
    /// A functioning chip returns
    /// [`DRIVER_ID`](crate::registers::DRIVER_ID) (`0x8A`); use this to
    /// probe communication after power-up.
    ///
    /// # Errors
    ///
    /// Returns `Err(T::Error)` if the index is out of range or the bus read
    /// fails.
    #[inline]
    pub async fn read_id(&mut self, index: usize) -> Result<u8, T::Error> {
        self.transport.read_reg(index, PAGE_FUNCTION, REG_ID).await
    }

    /// Runs one open or short detection scan and collects the result map.
    async fn run_detection(
        &mut self,
        index: usize,
        duty: u8,
        start_flag: u8,
        done_flag: u8,
        first_status_reg: u8,
    ) -> Result<Option<LedStatusMap>, T::Error> {
        // Select the test type, then trigger the scan by latching the duty
        // (writing OSDD starts the re-scan; hardware clears it when done).
        self.transport.write_page(index, PAGE_FUNCTION, REG_OPEN_SHORT_ENABLE, &[start_flag]).await?;
        let clamped_duty = duty.clamp(OPEN_SHORT_DUTY_MIN, OPEN_SHORT_DUTY_MAX);
        self.transport.write_page(index, PAGE_FUNCTION, REG_OPEN_SHORT_DUTY, &[clamped_duty]).await?;

        // Wait for the completion interrupt flag.
        let mut completed = false;
        for _ in 0..DETECTION_POLL_ATTEMPTS {
            Timer::after_micros(DETECTION_POLL_INTERVAL_US).await;
            let flags = self.transport.read_reg(index, PAGE_FUNCTION, REG_OPEN_SHORT_FLAG).await?;
            if flags & done_flag != 0 {
                completed = true;
                break;
            }
        }
        if !completed {
            return Ok(None);
        }
        // Clear the completion flag for the next scan.
        self.transport.write_page(index, PAGE_FUNCTION, REG_OPEN_SHORT_FLAG, &[0x00]).await?;

        // Collect the per-LED status bitmap from page 0.
        let mut flags = [0_u8; LED_OPEN_REGISTER_COUNT];
        let mut reg = first_status_reg;
        for slot in &mut flags {
            *slot = self.transport.read_reg(index, PAGE_LED_CONTROL, reg).await?;
            reg = reg.wrapping_add(1);
        }
        Ok(Some(LedStatusMap { flags }))
    }

    /// Writes pre-corrected PWM values for all LEDs and flushes immediately.
    ///
    /// # Errors
    ///
    /// Returns `Err(T::Error)` if the flush fails. See [`flush`](Self::flush).
    #[inline]
    pub async fn set_all_leds(&mut self, red: u8, green: u8, blue: u8) -> Result<(), T::Error> {
        self.stage_all_leds(red, green, blue);
        self.flush().await
    }

    /// Updates the constant-current step for all CB channels on driver `index`.
    ///
    /// `value` maps to output current as `Iout = value × 0.157 mA` for
    /// values 4-255. Values 1-3 produce ~0.47 mA; 0 disables current.
    ///
    /// # Errors
    ///
    /// Returns `Err(T::Error)` if the bus transaction fails.
    #[inline]
    pub async fn set_current_tune(&mut self, index: usize, value: u8) -> Result<(), T::Error> {
        self.transport.write_page(index, PAGE_CURRENT_TUNE, 0x00, &[value; CURRENT_TUNE_REGISTER_COUNT]).await
    }

    /// Writes pre-corrected PWM values for a single LED and flushes
    /// immediately.
    ///
    /// # Errors
    ///
    /// Returns `Err(T::Error)` if the flush fails. See [`flush`](Self::flush).
    #[inline]
    pub async fn set_led(&mut self, led_index: usize, red: u8, green: u8, blue: u8) -> Result<(), T::Error> {
        self.stage_led(led_index, red, green, blue);
        self.flush().await
    }

    /// Sets the scan-phase configuration on driver chip `index`.
    ///
    /// Use one of the `SCAN_PHASE_*_CHANNEL` constants from
    /// [`registers`](crate::registers). Fewer active phases raise the duty
    /// (and brightness) of the remaining CB channels; adjust
    /// [`set_current_tune`](Self::set_current_tune) accordingly per the
    /// datasheet note in section 6.9. [`init`](Self::init) configures all
    /// 12 phases.
    ///
    /// # Errors
    ///
    /// Returns `Err(T::Error)` if the bus transaction fails.
    #[inline]
    pub async fn set_scan_phase(&mut self, index: usize, phase: u8) -> Result<(), T::Error> {
        self.transport.write_page(index, PAGE_FUNCTION, REG_SCAN_PHASE, &[phase]).await
    }

    /// Puts all driver chips into software shutdown mode.
    ///
    /// Cuts LED current immediately; registers remain accessible. Call
    /// [`wake`](Self::wake) to resume.
    ///
    /// # Errors
    ///
    /// Returns `Err(T::Error)` if any bus transaction fails.
    #[inline]
    pub async fn shutdown(&mut self) -> Result<(), T::Error> {
        self.write_function_all(REG_SOFTWARE_SHUTDOWN, SOFTWARE_SHUTDOWN_SSD_SHUTDOWN).await
    }

    /// Puts all driver chips into software sleep mode (lowest power).
    ///
    /// Enters software shutdown first, then sets the SLEEP bit, following
    /// the datasheet sample code. Registers are inaccessible while asleep;
    /// call [`wake`](Self::wake) to resume.
    ///
    /// Note: sleeping chips wake on any SCL/SCK level change. With multiple
    /// chips on a shared bus, putting a later chip to sleep therefore wakes
    /// the earlier ones back into software shutdown; only the last chip
    /// stays asleep. Use [`hardware_sleep`](Self::hardware_sleep) for
    /// reliable multi-chip low power.
    ///
    /// # Errors
    ///
    /// Returns `Err(T::Error)` if any bus transaction fails.
    #[inline]
    pub async fn software_sleep(&mut self) -> Result<(), T::Error> {
        self.write_function_all(REG_SOFTWARE_SHUTDOWN, SOFTWARE_SHUTDOWN_SSD_SHUTDOWN).await?;
        self.write_function_all(REG_SOFTWARE_SLEEP, SOFTWARE_SLEEP_ENABLE).await?;
        // The datasheet sample code delays 1 ms after setting the SLEEP bit.
        Timer::after_millis(1).await;
        Ok(())
    }

    /// Stages RGB values for all LEDs in the layout without flushing.
    #[inline]
    pub fn stage_all_leds(&mut self, red: u8, green: u8, blue: u8) {
        let leds = self.leds;
        for &led in leds {
            let Some(buf) = self.bufs.get_mut(usize::from(led.driver)) else { continue };
            buf.stage(led, red, green, blue);
        }
    }

    /// Stages RGB values for one LED by layout index without flushing.
    ///
    /// Does nothing if `led_index` is out of bounds. Call
    /// [`flush`](Self::flush) to transmit pending changes.
    #[inline]
    pub fn stage_led(&mut self, led_index: usize, red: u8, green: u8, blue: u8) {
        let Some(&led) = self.leds.get(led_index) else { return };
        let Some(buf) = self.bufs.get_mut(usize::from(led.driver)) else { return };
        buf.stage(led, red, green, blue);
    }

    /// Returns a shared reference to the underlying transport.
    #[inline]
    #[must_use]
    pub const fn transport(&self) -> &T { &self.transport }

    /// Returns a mutable reference to the underlying transport, e.g. to
    /// drive [`set_sdb`](Transport::set_sdb) directly.
    #[inline]
    #[must_use]
    pub const fn transport_mut(&mut self) -> &mut T { &mut self.transport }

    /// Releases all driver chips from software shutdown or software sleep
    /// mode.
    ///
    /// Safe to call from either state: the first transaction doubles as the
    /// wakeup signal for sleeping chips (which wake on SCL/SCK level
    /// changes and may not acknowledge it), then normal mode is restored
    /// after the 128 µs wakeup time.
    ///
    /// # Errors
    ///
    /// Returns `Err(T::Error)` if the final mode-restore transaction fails.
    #[inline]
    pub async fn wake(&mut self) -> Result<(), T::Error> {
        // Generate bus activity to wake sleeping chips. A sleeping chip may
        // ignore or NACK this write - only the clock edges matter, so the
        // result is intentionally discarded. For chips that are merely shut
        // down it rewrites the value they already hold.
        match self.write_function_all(REG_SOFTWARE_SHUTDOWN, SOFTWARE_SHUTDOWN_SSD_SHUTDOWN).await {
            Ok(()) | Err(_) => {},
        }
        // Datasheet: system wakeup time is 128 µs.
        Timer::after_micros(150).await;
        self.write_function_all(REG_SOFTWARE_SHUTDOWN, SOFTWARE_SHUTDOWN_SSD_NORMAL).await
    }

    /// Writes `value` to function-page register `reg` on all `N` chips.
    async fn write_function_all(&mut self, reg: u8, value: u8) -> Result<(), T::Error> {
        for chip_index in 0..N {
            self.transport.write_page(chip_index, PAGE_FUNCTION, reg, &[value]).await?;
        }
        Ok(())
    }
}
