//! Host-side tests for the transport-independent driver core, using a mock
//! [`Transport`] that records every bus transaction.
#![allow(
    clippy::restriction,
    clippy::pedantic,
    clippy::nursery,
    reason = "test code favors brevity over the library's strict lint set"
)]

use embassy_futures::block_on;
use snled27351_driver::{
    Driver,
    Led,
    Transport,
    TransportError,
    pwm_register,
    registers::{
        OPEN_SHORT_FLAG_ODINT,
        OPEN_SHORT_FLAG_SDINT,
        PAGE_CURRENT_TUNE,
        PAGE_FUNCTION,
        PAGE_LED_CONTROL,
        PAGE_PWM,
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
        SCAN_PHASE_4_CHANNEL,
        SCAN_PHASE_12_CHANNEL,
        SOFTWARE_SHUTDOWN_SSD_NORMAL,
        SOFTWARE_SHUTDOWN_SSD_SHUTDOWN,
        ScanPhase,
    },
};
use std::collections::HashMap;

/// One recorded `write_page` call: `(chip, page, reg, data)`.
type Write = (usize, u8, u8, Vec<u8>);

#[derive(Default)]
struct MockTransport {
    /// Every successful write, in order.
    writes:     Vec<Write>,
    /// Register contents returned by `read_reg`, keyed by `(chip, page, reg)`.
    regs:       HashMap<(usize, u8, u8), u8>,
    /// Number of upcoming writes to fail, per chip.
    fail_count: HashMap<usize, usize>,
}

impl MockTransport {
    fn pwm_writes(&self) -> Vec<Write> {
        self.writes.iter().filter(|(_, page, ..)| *page == PAGE_PWM).cloned().collect()
    }
}

/// Implemented for exactly two chips, the pattern the `Transport` docs
/// recommend, so `Driver::new` infers `N` in these tests.
impl Transport<2> for MockTransport {
    type Error = ();

    async fn read_reg(&mut self, driver_index: usize, page: u8, reg: u8) -> Result<u8, ()> {
        Ok(self.regs.get(&(driver_index, page, reg)).copied().unwrap_or(0))
    }

    async fn reset(&mut self) -> Result<(), ()> { Ok(()) }

    fn set_sdb(&mut self, _enable: bool) -> Result<(), ()> { Ok(()) }

    async fn write_page(&mut self, driver_index: usize, page: u8, reg: u8, data: &[u8]) -> Result<(), ()> {
        if let Some(remaining) = self.fail_count.get_mut(&driver_index) {
            if *remaining > 0 {
                *remaining -= 1;
                return Err(());
            }
        }
        self.writes.push((driver_index, page, reg, data.to_vec()));
        Ok(())
    }
}

/// Two LEDs on chip 0 and one on chip 1. Channels deliberately span
/// non-adjacent CB rows to exercise the dirty-range logic.
static LEDS: [Led; 3] = [
    Led::new(0, pwm_register(1, 1), pwm_register(3, 1), pwm_register(2, 1)),
    Led::new(0, pwm_register(1, 2), pwm_register(3, 2), pwm_register(2, 2)),
    Led::new(1, pwm_register(4, 5), pwm_register(4, 6), pwm_register(4, 7)),
];

fn initialized_driver() -> Driver<MockTransport, 2> {
    let mut driver = Driver::new(MockTransport::default(), &LEDS);
    block_on(driver.init(0xCC)).unwrap();
    driver.transport_mut().writes.clear();
    driver
}

#[test]
fn pwm_register_matches_datasheet_map() {
    assert_eq!(pwm_register(1, 1), 0x00);
    assert_eq!(pwm_register(2, 1), 0x10);
    assert_eq!(pwm_register(1, 16), 0x0F);
    assert_eq!(pwm_register(12, 16), 0xBF);
}

#[test]
fn init_programs_every_chip_with_the_datasheet_sequence() {
    let mut driver = Driver::new(MockTransport::default(), &LEDS);
    block_on(driver.init(0xCC)).unwrap();

    let expected = |chip: usize| -> Vec<Write> {
        vec![
            (chip, PAGE_FUNCTION, REG_SOFTWARE_SHUTDOWN, vec![SOFTWARE_SHUTDOWN_SSD_SHUTDOWN]),
            (chip, PAGE_FUNCTION, REG_PULLDOWNUP, vec![0xAA]),
            (chip, PAGE_FUNCTION, REG_SCAN_PHASE, vec![SCAN_PHASE_12_CHANNEL]),
            (chip, PAGE_FUNCTION, REG_SLEW_RATE_CONTROL_MODE_1, vec![0x04]),
            (chip, PAGE_FUNCTION, REG_SLEW_RATE_CONTROL_MODE_2, vec![0xC0]),
            (chip, PAGE_FUNCTION, REG_SOFTWARE_SLEEP, vec![0x00]),
            (chip, PAGE_LED_CONTROL, 0x00, vec![0x00; 24]),
            (chip, PAGE_PWM, 0x00, vec![0x00; 192]),
            (chip, PAGE_CURRENT_TUNE, 0x00, vec![0xCC; 12]),
            (chip, PAGE_LED_CONTROL, 0x00, vec![0xFF; 24]),
            (chip, PAGE_FUNCTION, REG_SOFTWARE_SHUTDOWN, vec![SOFTWARE_SHUTDOWN_SSD_NORMAL]),
        ]
    };
    let mut all = expected(0);
    all.extend(expected(1));
    assert_eq!(driver.transport().writes, all);
}

#[test]
fn flush_writes_only_the_changed_span() {
    let mut driver = initialized_driver();
    block_on(driver.set_led(0, 1, 2, 3)).unwrap();

    // Red at 0x00, blue at 0x10, green at 0x20: one burst covering 0x00..=0x20.
    let writes = driver.transport().pwm_writes();
    assert_eq!(writes.len(), 1);
    let (chip, _, reg, data) = &writes[0];
    assert_eq!((*chip, *reg, data.len()), (0, 0x00, 0x21));
    assert_eq!((data[0x00], data[0x20], data[0x10]), (1, 2, 3));
}

#[test]
fn flush_single_channel_change_is_one_byte() {
    let mut driver = initialized_driver();
    block_on(driver.set_led(1, 10, 20, 30)).unwrap();
    driver.transport_mut().writes.clear();

    block_on(driver.set_led(1, 10, 99, 30)).unwrap();
    assert_eq!(driver.transport().pwm_writes(), vec![(0, PAGE_PWM, pwm_register(3, 2), vec![99])]);
}

#[test]
fn dirty_range_grows_downward() {
    let mut driver = initialized_driver();
    // Stage the higher addresses first so the lower bound has to move.
    driver.stage_led(1, 1, 1, 1); // 0x01, 0x21, 0x11
    driver.stage_led(0, 2, 2, 2); // 0x00, 0x20, 0x10
    block_on(driver.flush()).unwrap();

    let writes = driver.transport().pwm_writes();
    assert_eq!(writes.len(), 1);
    let (_, _, reg, data) = &writes[0];
    assert_eq!((*reg, data.len()), (0x00, 0x22));
    assert_eq!((data[0x00], data[0x01]), (2, 1));
}

#[test]
fn restaging_unchanged_values_sends_nothing() {
    let mut driver = initialized_driver();
    block_on(driver.set_led(0, 5, 6, 7)).unwrap();
    driver.transport_mut().writes.clear();

    block_on(driver.set_led(0, 5, 6, 7)).unwrap();
    // Zero is the post-init value, so this is a no-op as well.
    block_on(driver.set_led(2, 0, 0, 0)).unwrap();
    // Out-of-range index is ignored.
    block_on(driver.set_led(99, 1, 1, 1)).unwrap();
    assert!(driver.transport().writes.is_empty());
}

#[test]
fn set_all_leds_flushes_each_chip_once() {
    let mut driver = initialized_driver();
    block_on(driver.set_all_leds(255, 255, 255)).unwrap();

    let writes = driver.transport().pwm_writes();
    assert_eq!(writes.len(), 2);
    assert_eq!((writes[0].0, writes[0].2, writes[0].3.len()), (0, 0x00, 0x22));
    assert_eq!(writes[1], (1, PAGE_PWM, pwm_register(4, 5), vec![255, 255, 255]));
}

#[test]
fn failed_flush_is_retried() {
    let mut driver = initialized_driver();
    driver.transport_mut().fail_count.insert(0, 1);
    assert!(block_on(driver.set_led(0, 1, 2, 3)).is_err());
    assert!(driver.transport().pwm_writes().is_empty());

    block_on(driver.flush()).unwrap();
    assert_eq!(driver.transport().pwm_writes().len(), 1);
}

#[test]
fn wake_reaches_every_chip_even_if_the_first_nacks() {
    let mut driver = initialized_driver();
    // Chip 0 is asleep and NACKs the wakeup write.
    driver.transport_mut().fail_count.insert(0, 1);
    block_on(driver.wake()).unwrap();

    let writes = &driver.transport().writes;
    assert_eq!(writes, &vec![
        (1, PAGE_FUNCTION, REG_SOFTWARE_SHUTDOWN, vec![SOFTWARE_SHUTDOWN_SSD_SHUTDOWN]),
        (0, PAGE_FUNCTION, REG_SOFTWARE_SHUTDOWN, vec![SOFTWARE_SHUTDOWN_SSD_NORMAL]),
        (1, PAGE_FUNCTION, REG_SOFTWARE_SHUTDOWN, vec![SOFTWARE_SHUTDOWN_SSD_NORMAL]),
    ]);
}

#[test]
fn detect_open_reports_flagged_channels() {
    let mut driver = initialized_driver();
    let open_addr = pwm_register(3, 2);
    let regs = &mut driver.transport_mut().regs;
    regs.insert((0, PAGE_FUNCTION, REG_OPEN_SHORT_FLAG), OPEN_SHORT_FLAG_ODINT);
    regs.insert((0, PAGE_LED_CONTROL, REG_LED_OPEN_FIRST + (open_addr >> 3)), 1 << (open_addr & 0x07));

    let map = block_on(driver.detect_open(0, 0x10)).unwrap().unwrap();
    assert!(map.is_flagged(open_addr));
    assert_eq!((0..=0xBF).filter(|&addr| map.is_flagged(addr)).count(), 1);
}

#[test]
fn detect_open_times_out_without_completion_flag() {
    let mut driver = initialized_driver();
    assert_eq!(block_on(driver.detect_open(0, 0x10)), Ok(None));
}

#[test]
fn detection_clears_stale_completion_flag_before_triggering() {
    let mut driver = initialized_driver();
    let _ = block_on(driver.detect_open(0, 0x00)).unwrap();
    // First three writes: clear flag, select test, latch (clamped) duty.
    assert_eq!(&driver.transport().writes[..3], &[
        (0, PAGE_FUNCTION, REG_OPEN_SHORT_FLAG, vec![0x00]),
        (0, PAGE_FUNCTION, REG_OPEN_SHORT_ENABLE, vec![0x80]),
        (0, PAGE_FUNCTION, REG_OPEN_SHORT_DUTY, vec![0x01]),
    ]);
}

#[test]
fn detect_short_uses_short_bits_and_registers() {
    let mut driver = initialized_driver();
    let short_addr = pwm_register(2, 3);
    let open_only_addr = pwm_register(5, 5);
    let regs = &mut driver.transport_mut().regs;
    regs.insert((0, PAGE_FUNCTION, REG_OPEN_SHORT_FLAG), OPEN_SHORT_FLAG_SDINT);
    regs.insert((0, PAGE_LED_CONTROL, REG_LED_SHORT_FIRST + (short_addr >> 3)), 1 << (short_addr & 0x07));
    regs.insert((0, PAGE_LED_CONTROL, REG_LED_OPEN_FIRST + (open_only_addr >> 3)), 1 << (open_only_addr & 0x07));

    let map = block_on(driver.detect_short(0, 0x10)).unwrap().unwrap();
    assert!(map.is_flagged(short_addr));
    assert!(!map.is_flagged(open_only_addr));
    assert_eq!((0..=0xBF).filter(|&addr| map.is_flagged(addr)).count(), 1);
    assert_eq!(driver.transport().writes[1], (0, PAGE_FUNCTION, REG_OPEN_SHORT_ENABLE, vec![0x40]));
}

#[test]
fn detect_short_ignores_open_completion_flag() {
    let mut driver = initialized_driver();
    driver.transport_mut().regs.insert((0, PAGE_FUNCTION, REG_OPEN_SHORT_FLAG), OPEN_SHORT_FLAG_ODINT);
    assert_eq!(block_on(driver.detect_short(0, 0x10)), Ok(None));
}

#[test]
fn detection_duty_is_clamped_to_upper_limit() {
    let mut driver = initialized_driver();
    let _ = block_on(driver.detect_open(0, 0xFF)).unwrap();
    assert_eq!(driver.transport().writes[2], (0, PAGE_FUNCTION, REG_OPEN_SHORT_DUTY, vec![0xFA]));
}

#[test]
fn set_scan_phase_writes_register_value() {
    let mut driver = initialized_driver();
    block_on(driver.set_scan_phase(1, ScanPhase::Channels4)).unwrap();
    assert_eq!(driver.transport().writes, vec![(1, PAGE_FUNCTION, REG_SCAN_PHASE, vec![SCAN_PHASE_4_CHANNEL])]);
}

#[test]
fn scan_phase_conversions() {
    assert_eq!(ScanPhase::default(), ScanPhase::Channels12);
    assert_eq!(ScanPhase::Channels12.register_value(), SCAN_PHASE_12_CHANNEL);
    for count in 1..=12 {
        let phase = ScanPhase::from_active_channels(count).unwrap();
        assert_eq!(phase.active_channels(), count);
        // Register value counts *inactive* channels: 12 - count.
        assert_eq!(phase.register_value(), 12 - count);
        assert_eq!(ScanPhase::from_register_value(12 - count), Some(phase));
    }
    assert_eq!(ScanPhase::from_active_channels(0), None);
    assert_eq!(ScanPhase::from_active_channels(13), None);
    assert_eq!(ScanPhase::from_register_value(0x0C), None);
}

#[test]
#[should_panic(expected = "driver chip the transport does not manage")]
fn new_rejects_layout_for_missing_chip() {
    static BAD: [Led; 1] = [Led::new(2, 0, 1, 2)];
    let _ = Driver::<MockTransport, 2>::new(MockTransport::default(), &BAD);
}

#[test]
#[should_panic(expected = "PWM register address must be at most 0xBF")]
fn led_new_rejects_out_of_range_register() { let _ = std::hint::black_box(Led::new)(0, 0xC0, 0, 0); }

#[test]
fn transport_error_display() {
    assert_eq!(TransportError::Bus("nack").to_string(), "bus error: \"nack\"");
    assert_eq!(TransportError::<()>::InvalidIndex.to_string(), "driver index out of range");
    let _: &dyn core::error::Error = &TransportError::<()>::Pin;
}
