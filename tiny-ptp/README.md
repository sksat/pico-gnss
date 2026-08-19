# tiny-ptp

An end-to-end two-step PTP exchange, and nothing else.

Two ports, fixed roles, one wire. What this implements is the part of IEEE 1588 that carries time
between them — `Sync`, `Follow_Up`, `Delay_Req`, `Delay_Resp`, and the offset and mean path delay
they produce — and it stops there. No best-master algorithm, no `Announce`, no management, no peer
delay: with two ports that were built together there is nothing to elect and nothing to discover.
Call it a static-role subset, not an ordinary clock.

`no_std`, integer-only, no dependencies. Nothing here knows about Ethernet, UDP, or where a
timestamp came from — the same boundary [`tiny-ntp`](../tiny-ntp) keeps.

## Why two-step

A transmit timestamp has to be written into a message before that message is checksummed and
encoded, so it is always a claim about a moment that has not happened yet. On the hardware this was
written for, that claim was wrong by hundreds of microseconds, and wrong by an amount that varied
with how long the encoding took — measured, on the NTP path this replaces.

Two-step is the standard's answer. Send the message, let the hardware say when it actually left,
and send that afterwards in a `Follow_Up`. The timestamp that counts is never inside the message it
describes.

## What it does not know

The halving in `mean_path_delay` is an assumption: that the two directions took the same time.
Whatever they did not share appears in the offset at half its size, and nothing in the exchange can
see it. That is the mechanism's central limitation, and there is a test that says so.

## Timescale

The standard's own timescale is TAI since 1970. This carries whatever the caller put in — UTC from
a GNSS receiver, in the case it was written for — and leaves the `ptpTimescale` flag clear to say
as much. That is a profile decision, and it is why two ends have to have been built together.
