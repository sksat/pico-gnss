# tiny-ntp

`no_std`, integer-only, **zero-dependency** NTP ([RFC 5905](https://www.rfc-editor.org/info/rfc5905))
for embedded targets. A Stratum-1 reference-clock server, and a client for both unicast and broadcast.

This crate turns a **disciplined UTC instant (Unix nanoseconds)** into the bytes of an NTP packet,
and back. It knows nothing about Ethernet, IPv4, UDP or any HAL — only integer nanosecond timestamps
cross into it, which is what lets it run on the host under `cargo test` and on a microcontroller
unchanged.

It is built for the case where **you are the reference clock**: a GNSS/PPS-disciplined oscillator
serving Stratum 1, rather than a client asking someone else for the time.

## What it does

- `timestamp` — NTP's two fixed-point formats and their Unix-ns conversions:
  - `NtpTimestamp` (32.32, seconds since the 1900 prime epoch) with **era** handling, so decoding
    still works across the 2036-02-07 wrap.
  - `NtpShort` (16.16) for root delay / root dispersion, **saturating** rather than wrapping — a
    wrapped dispersion would advertise a better clock than you have.
- `packet` — the 48-byte header, encoded and decoded.
- `server` — Stratum-1 policy. Both service modes:
  - `respond` for a unicast client exchange (mode 3 → 4), echoing the client's timestamps so it can
    separate offset from round-trip delay.
  - `broadcast` for one-way announcement (mode 5), for a link where nothing can be received.

  Both are gated on the clock's discipline state and grow root dispersion through a holdover.

The encoding is cross-checked against Wireshark's dissector rather than only against itself; see
`tests/wireshark_decode.rs`.

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
