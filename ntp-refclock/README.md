# ntp-refclock

`no_std`, integer-only, **zero-dependency** NTP ([RFC 5905](https://www.rfc-editor.org/info/rfc5905))
reference-clock server core.

This crate turns a **disciplined UTC instant (Unix nanoseconds)** into the bytes of an NTP packet,
and back. It knows nothing about Ethernet, IPv4, UDP or any HAL — only integer nanosecond timestamps
cross into it, which is what lets it run on the host under `cargo test` and on a microcontroller
unchanged.

It is built for the case where **you are the reference clock**: a GNSS/PPS-disciplined oscillator
serving Stratum 1, rather than a client asking someone else for the time.

## Status

Early. Implemented so far:

- `timestamp` — NTP's two fixed-point formats and their Unix-ns conversions:
  - `NtpTimestamp` (32.32, seconds since the 1900 prime epoch) with **era** handling, so decoding
    still works across the 2036-02-07 wrap.
  - `NtpShort` (16.16) for root delay / root dispersion, **saturating** rather than wrapping — a
    wrapped dispersion would advertise a better clock than you have.

Planned: 48-byte packet encode/decode, and Stratum-1 broadcast packet construction gated on the
clock's discipline state with holdover-aware root dispersion.

## Where the other pieces live

| Concern | Crate |
|---|---|
| Disciplined UTC + holdover (the time source) | [`gnssdo`](https://docs.rs/gnssdo) |
| Ethernet / IPv4 / UDP framing, 10BASE-T PHY | `pico-10base-t` |
| Wiring it together on real hardware | `pico-ntp` |

Keeping framing out of this crate is what lets the NTP layer stay L2/L3/L4-agnostic: the same packet
bytes go out over 10BASE-T here, or over anything else elsewhere.

## License

MIT
