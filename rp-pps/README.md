# rp-pps

[![crates.io](https://img.shields.io/crates/v/rp-pps.svg)](https://crates.io/crates/rp-pps)
[![docs.rs](https://img.shields.io/docsrs/rp-pps)](https://docs.rs/rp-pps)
[![License: MIT](https://img.shields.io/badge/license-MIT-blue.svg)](https://github.com/sksat/pico-gnss-rs/blob/main/LICENSE)

RP2040 / RP2350 **PIO building blocks for a GNSS 1PPS timebase**, plus NMEA time
ingestion. The device/receiver-facing companion to
[`gnssdo`](https://crates.io/crates/gnssdo): `gnssdo` is the HAL-agnostic discipline
core (timestamps + a UTC epoch → disciplined UTC), and `rp-pps` is what *produces*
those inputs on the RP2040 — it hardware-timestamps the PPS edge on the PIO
(~16 ns, free of the µs-scale jitter a software GPIO interrupt has on a Cortex-M0+),
emits a steerable 1PPS, and decodes the receiver's NMEA to pair each edge with its
UTC second.

## What it provides

**HAL-agnostic core** (always available, host-tested with `cargo test`):

- PIO programs (`pps_capture_program`, `pps_output_program`) and their FIFO-word
  contracts, built with `pio::pio_asm!` so every backend loads the same
  `pio::Program`.
- tick↔ns / period-word math (`interval_ns`, `output_period_cycles_ppb`, …),
  sub-cycle `OutputPeriodDither` (sigma-delta), and loopback phase
  (`loopback_phase_ns`, `calibrate_loopback_offset`).
- `PpsEdgeTimeline` — turns raw capture-counter values into timed edges.
- NMEA framing/parsing (`NmeaLineAssembler`, `parse_rmc_time_date`) and the
  PPS↔UTC-second pairing (`PpsTimeSync` → `SyncEpoch`).

**Backends** (thin, feature-gated): `embassy-rp` (async) and `rp2040-hal`
(blocking/IRQ). Each is a small concrete type — there is no unified HAL trait
(async vs blocking diverge). Two tiers per backend:

- *fine*: `PpsCapture` / `PpsOutput` (raw FIFO I/O).
- *easy*: `TimedPpsCapture` (`next_edge().await -> TimedEdge`) /
  `SteeredPpsOutput` (`set_next_period(freq_mppb, phase_corr_ns)`).

**`gnssdo` feature** (turn-key, requires `embassy-rp`): `PpsGpsdo` bundles
`gnssdo`'s discipline with the PPS↔NMEA pairing, and `run_capture` / `run_nmea` are
embassy runner tasks that drive it — so an app spawns the runners and reads
disciplined UTC.

## Scope

`rp-pps` owns the device/receiver **I/O and time ingestion**: capturing edges,
emitting pulses, and turning the receiver's NMEA + a PPS edge into a UTC epoch. It
deliberately does **not** own the discipline (frequency estimation, holdover, the
phase servo) — that is [`gnssdo`](https://crates.io/crates/gnssdo)'s job; feed it the
timestamps and epoch this crate produces.

## Example (sketch — turn-key GPSDO)

```rust,ignore
use rp_pps::PpsGpsdo;
use rp_pps::embassy::{run_capture, run_nmea, TimedPpsCapture};

static CLOCK: BlockingMutex<CriticalSectionRawMutex, RefCell<PpsGpsdo>> =
    BlockingMutex::new(RefCell::new(PpsGpsdo::new()));

// Spawn the two runners (your own #[task] wrappers); they discipline CLOCK on their own:
//   run_capture(capture, &CLOCK, || Instant::now().as_micros() * 1000)  // PPS
//   run_nmea(uart_rx, &CLOCK)                                           // NMEA
// then read disciplined UTC anywhere:
let utc_ns = CLOCK.lock(|g| g.borrow().now_from_query_ns(now_ns()));
```

See the `gpsdo` (drive `PpsGpsdo` by hand) and `gpsdo_runner` (spawn the runners)
examples in the [repository](https://github.com/sksat/pico-gnss-rs).

## Cargo features

- **`embassy-rp`** — the async embassy-rp backend. The downstream binary selects the
  chip (e.g. `embassy-rp/rp2040`); this crate stays chip-feature-agnostic.
- **`rp2040-hal`** — the blocking/IRQ rp2040-hal backend (RP2040 only).
- **`gnssdo`** — bundle the discipline core into `PpsGpsdo` + the runner tasks.
- **`external-nmea`** — parse RMC with the [`nmea`](https://docs.rs/nmea) crate
  instead of the zero-dependency built-in parser.

On [docs.rs](https://docs.rs/rp-pps) the `embassy-rp` backend (with the `rp2040`
chip), `gnssdo` and `external-nmea` are built; the `rp2040-hal` backend is not built
there (one backend per docs build) — read its source for that variant.
