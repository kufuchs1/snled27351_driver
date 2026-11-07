# snled27351-driver

Async, `no_std` [embedded-hal] driver for the SONiX **SNLED27351** 16×12 matrix
LED driver IC, over SPI or I2C.

- One driver core, generic over a `Transport`. SPI and I2C share all register
  logic.
- Supports several chips per bus. The chip count is a const generic shared by
  the driver and the transport. With the provided transports, a mismatch is a
  compile error.
- Keeps a PWM shadow buffer per chip. `flush` sends only the register range
  that changed, and skips chips with no changes.
- Power management: software shutdown, software sleep, and hardware sleep via
  the SDB pin. If SDB is strapped high, use `NoSdb`.
- Open- and short-circuit detection with per-LED results.

[embedded-hal]: https://github.com/rust-embedded/embedded-hal

## Usage

Enable the transport you need:

```toml
[dependencies]
snled27351-driver = { version = "2", features = ["i2c"] } # or "spi"
```

```rust,ignore
use snled27351_driver::{pwm_register, transport::i2c, Driver, Led};

// Each LED maps to a chip index and three PWM registers (CB row, CA column).
static LEDS: [Led; 2] = [
    Led::new(0, pwm_register(1, 1), pwm_register(3, 1), pwm_register(2, 1)),
    Led::new(0, pwm_register(1, 2), pwm_register(3, 2), pwm_register(2, 2)),
];

let transport = i2c::Controller::new(i2c_bus, [i2c::ADDRESS_GND]);
let mut driver = Driver::new(transport, &LEDS); // chip count inferred: 1
driver.init(0xCC).await?; // constant current ≈ 0xCC × 0.157 mA ≈ 32 mA

driver.set_led(0, 255, 0, 128).await?; // stage + flush one LED

// Batch updates: stage several LEDs, then send them together.
driver.stage_led(0, 10, 20, 30);
driver.stage_led(1, 40, 50, 60);
driver.flush().await?;
```

Over SPI, pass one `SpiDevice` per chip (each owns its CS line) and the SDB pin:

```rust,ignore
use snled27351_driver::{transport::spi, Driver};

let transport = spi::Controller::new([spi_dev0, spi_dev1], sdb_pin);
let mut driver = Driver::new(transport, &LEDS); // chip count inferred: 2
```

The maximum SPI clock is 4 MHz.

`LEDS` is a `static`, so layout mistakes are caught at compile time:
- `pwm_register` rejects out-of-range CB/CA indices.
- `Led::new` rejects register addresses above `0xBF`.
- `Driver::new` rejects an LED that names a chip the transport doesn't have.
  This check panics at runtime if `Driver::new` isn't called in a const
  context.

### Diagnostics

```rust,ignore
use snled27351_driver::registers::DRIVER_ID;

assert_eq!(driver.read_id(0).await?, DRIVER_ID); // probe communication
let hot = driver.check_thermal_flag_set(0).await?; // ≥ 70 °C?

if let Some(open) = driver.detect_open(0, 0x20).await? {
    if open.is_flagged(pwm_register(1, 1)) { /* LED on CB1/CA1 is open */ }
}
```

### Tuning

```rust,ignore
use snled27351_driver::ScanPhase;

// Scan only CB1-CB6. The remaining rows get a higher duty, so lower the current.
driver.set_scan_phase(0, ScanPhase::Channels6).await?;
driver.set_current_tune(0, 0x66).await?;
```

## Timing

The driver waits using `embassy-time`, so your application must provide an
`embassy-time` driver. Embassy HALs do this for you.

## Minimum supported Rust version

Rust 1.85 (edition 2024). Development uses the dated nightly pinned in
`rust-toolchain.toml` for rustfmt and clippy.

## License

Licensed under either of [Apache License, Version 2.0](LICENSE-APACHE) or
[MIT license](LICENSE-MIT) at your option.
