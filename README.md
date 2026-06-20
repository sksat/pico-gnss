# pico-gnss-rs

A GNSS **PPS-disciplined clock (GPSDO/GNSSDO)** built on the RP2040, plus a reusable
`no_std` core crate and a real-time web dashboard.

The 1 Hz PPS edge from a GNSS receiver marks the UTC second boundary. This project
captures that edge with nanosecond resolution, estimates the local crystal's
frequency error, and keeps disciplined UTC — including **holdover** while the PPS
signal is lost.

## Workspace layout

| Path | What |
|---|---|
| [`gnssdo/`](gnssdo/) | **Core library** ([crate `gnssdo`](gnssdo/README.md)). `no_std`, HAL-agnostic, integer-only, zero-dependency. Frequency (ppb) estimation, holdover, PPS tracking, NMEA/PPS time sync. Host-testable. |
| [`firmware/`](firmware/) | RP2040 firmware (embassy-rp). PIO hardware PPS capture, clock discipline, disciplined PPS output. Embedded-only; uses `gnssdo`. |
| [`webapp/`](webapp/) | Real-time dashboard (React 19 + Vite + TypeScript), fed from the firmware's defmt/RTT output via a zero-dependency Node bridge. |
| [`report/`](report/) | On-hardware evaluation logs and figures. |
| [`NOTES.md`](NOTES.md) | Design decisions and hard-won gotchas. |

## Quick start

```sh
# Core library — runs on the host, no hardware needed:
cargo test -p gnssdo

# Firmware — needs a probe-rs-compatible probe + RP2040 and the embedded target:
rustup target add thumbv6m-none-eabi
cd firmware && cargo run --release        # builds, flashes, streams defmt logs
```

The workspace's `default-members` is `gnssdo`, so a bare `cargo build`/`cargo test`
from the root only touches the host-safe core. The firmware is embedded-only and is
built from within `firmware/` (where its `.cargo/config.toml` selects the
`thumbv6m-none-eabi` target and the probe-rs runner).

## Hardware

- **MCU**: RP2040 (Raspberry Pi Pico, Seeed XIAO RP2040, …).
- **GNSS module** with NMEA + 1PPS output, e.g. Akizuki `AE-GNSS-EXTANT` /
  GYSFFMANC (MediaTek MT3333), 9600 baud.
- **Wiring**: UART0 RX = GP1 (module TX), UART0 TX = GP0 (module RX), PPS = GP2.
  Common ground is required.

## Measured performance (firmware, RP2040 @ 125 MHz)

- PPS timestamping via PIO hardware capture: jitter ~10–16 ns (vs ~9 µs for a
  software GPIO-interrupt approach).
- Crystal frequency disciplined in ppb with multi-stage outlier gating + holdover.
- Disciplined PPS output phase-locked to UTC: σ ~35–48 ns (Smith-predictor servo).

See [`report/REPORT.md`](report/REPORT.md) for the methodology and figures.

## License

MIT — see [LICENSE](LICENSE).
