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
    pwm_register,
    registers::{
        OPEN_SHORT_FLAG_ODINT,
        PAGE_FUNCTION,
        PAGE_LED_CONTROL,
        PAGE_PWM,
        REG_LED_OPEN_FIRST,
        REG_OPEN_SHORT_FLAG,
        REG_SOFTWARE_SHUTDOWN,
        SOFTWARE_SHUTDOWN_SSD_NORMAL,
        SOFTWARE_SHUTDOWN_SSD_SHUTDOWN,
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

impl Transport for MockTransport {
    type Error = ();

    async fn read_reg(&mut self, driver_index: usize, page: u8, reg: u8) -> Result<u8, ()> {
        Ok(self.regs.get(&(driver_index, page, reg)).copied().unwrap_or(0))
    }

    async fn reset(&mut self) -> Result<(), ()> { Ok(()) }

    fn set_sdb(&mut self, _enable: bool) -> Result<(), ()> { Ok(()) }

    async fn write_page(&mut self, driver_index: usize, page: u8, reg: u8, data: &[u8]) -> Result<(), ()> {
        if let Some(remaining) = self.fail_count.get_mut(&driver_index)
            && *remaining > 0
        {
            *remaining -= 1;
            return Err(());
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
fn init_programs_every_chip_and_ends_in_normal_mode() {
    let mut driver = Driver::<_, 2>::new(MockTransport::default(), &LEDS);
    block_on(driver.init(0xCC)).unwrap();
    for chip in 0..2 {
        let last = driver.transport().writes.iter().rev().find(|(c, ..)| *c == chip).unwrap().clone();
        assert_eq!(last, (chip, PAGE_FUNCTION, REG_SOFTWARE_SHUTDOWN, vec![SOFTWARE_SHUTDOWN_SSD_NORMAL]));
    }
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
