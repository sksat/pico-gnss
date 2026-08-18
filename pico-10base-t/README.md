# pico-10base-t

**10BASE-T Ethernet transmit from a Raspberry Pi Pico with three resistors and no PHY chip.**

`no_std`. HAL-agnostic core that runs under `cargo test` on the host, plus a thin `embassy-rp`
backend. Transmit only.

## Credit

This crate is a Rust port of **[kingyoPiyo/Pico-10BASE-T](https://github.com/kingyoPiyo/Pico-10BASE-T)**
by **kingyo** — the original C/PIO implementation that showed an RP2040 can drive 10BASE-T directly
from two GPIO pins. The PIO serialiser here follows that design closely: a `.side_set 2` program
clocked at 20 MHz whose `out pc, 2` treats each 2-bit symbol as a jump target, so the three
instruction slots *are* the three line states (idle / low / high) and the CPU only has to hand the
state machine a pre-encoded symbol stream.

That project is MIT licensed:

> MIT License
>
> Copyright (c) 2022 kingyo

This port is MIT licensed as well. The clever part is theirs; the bugs are ours.

The upstream project is transmit-only, and so is this one — receiving 10BASE-T needs hardware this
wiring does not have.

## Hardware

Following the upstream wiring:

| | |
|---|---|
| TX− | GP16 |
| TX+ | GP17 |
| Resistors | 2 × 47 Ω (one in series with each TX pin), 1 × 470 Ω (across the pair) |
| Connector | RJ45 — pins 1 and 2 (the receiver's RX pair) |

Upstream recommends a pulse transformer for isolation, and so do we: without one there is no
galvanic isolation between the Pico and whatever it is plugged into.

## Status

Early. See the crate docs for what is implemented.

## Why it exists here

This repository builds a GNSS-disciplined clock ([`gnssdo`](https://docs.rs/gnssdo)) and is turning
it into a Stratum-1 NTP time server. That needs a way to put packets on a wire; a Pico with three
resistors is a far more interesting way to do it than an Ethernet module.

| Concern | Crate |
|---|---|
| Disciplined UTC + holdover | [`gnssdo`](https://docs.rs/gnssdo) |
| NTP wire format + Stratum-1 policy | `ntp-refclock` |
| **Ethernet framing + 10BASE-T PHY** | **this crate** |
| Wiring it together on real hardware | `pico-ntp` |

## License

MIT
