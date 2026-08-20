#![no_std]
#![no_main]

//! **Stratum-1 NTP broadcast server on an RP2040.**
//!
//! A GNSS 1PPS-disciplined clock serving time over 10BASE-T Ethernet driven straight from two GPIO
//! pins and three resistors — no PHY chip, no MAC.
//!
//! ```text
//! GNSS module ──NMEA──> UART0 RX (GP1)  ──┐
//!             ──1PPS──> GP2 (PIO0 SM0)  ──┴─> PpsGpsdo ──> disciplined UTC
//!                                                            │
//!                                    tiny-ntp ───────────┘  (48-byte NTP packet)
//!                                          │
//!                                    pico-10base-t            (Ethernet/IPv4/UDP + Manchester)
//!                                          │
//!                                    PIO1 SM0 + DMA ──> GP16 (TX−) / GP17 (TX+)
//! ```
//!
//! # Wiring
//!
//! | Signal | Pin |
//! |---|---|
//! | GNSS NMEA TX → UART0 RX | GP1 (9600 baud) |
//! | GNSS 1PPS | GP2 |
//! | Ethernet TX− | GP16 |
//! | Ethernet TX+ | GP17 |
//!
//! 2 × 47 Ω in series with the TX pins and 1 × 470 Ω across the pair, into an RJ45's pins 1 and 2.
//! A pulse transformer is strongly recommended: without one there is no galvanic isolation between
//! the Pico and whatever it is plugged into.
//!
//! PIO0 belongs to the PPS capture and PIO1 to the Ethernet serialiser, so the two never contend.
//!
//! # Receiving this
//!
//! Broadcast (RFC 5905 mode 5) is one-way, which is all the present wiring can do — **not** where
//! this is meant to end up. The unicast exchange is the better protocol in every respect: it lets a
//! client measure the path instead of assuming it. [`tiny_ntp::server::respond`] already builds
//! those replies and is tested; what is missing is a receive path in the hardware.
//!
//! Meanwhile, note that **chrony and systemd-timesyncd do not implement broadcast client mode at
//! all** — the reference `ntpd` does, with `broadcastclient` (and, since we cannot answer the
//! calibration exchange it would like to make, `disable auth` and an explicit `broadcastdelay`).

#[cfg(feature = "swd-rx")]
mod swd_rx;

use core::cell::{Cell, RefCell};
use core::sync::atomic::{AtomicBool, Ordering};

use defmt::{info, warn};
use defmt_rtt as _;
use panic_probe as _;

use embassy_executor::Spawner;
use embassy_rp::bind_interrupts;
use embassy_rp::clocks::clk_sys_freq;
use embassy_rp::peripherals::{PIO0, PIO1, UART0};
use embassy_rp::pio::{InterruptHandler as PioInterruptHandler, Pio};
use embassy_rp::uart::{
    BufferedInterruptHandler, BufferedUart, BufferedUartRx, Config as UartConfig,
};
use embassy_sync::channel::Channel;
use embassy_sync::blocking_mutex::Mutex as BlockingMutex;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_time::{Duration, Instant, Timer, with_timeout};
use embedded_io_async::Read as _;
use static_cell::StaticCell;

use pico_10base_t::embassy::{Rx10BaseT, Tx10BaseT};
use pico_10base_t::frame::{
    Ipv4Addr, MacAddr, UdpFrameSpec, build_udp_frame, frame_len, parse_udp_frame,
};
use pico_10base_t::phy::{NLP_INTERVAL_US, encode_frame, encoded_words};
use pico_10base_t::rx::decode_frame;
use rp_pps::embassy::{
    PpsCapture, PpsOutput, TimedPpsCapture, run_capture, run_nmea, set_capture_polarity,
    start_in_sync,
};
use rp_pps::{
    PpsGpsdo, PpsPolarity, PpsSchedule, PpsScheduleConfig, output_high_cycles,
    output_period_cycles,
};
use tiny_ntp::packet::{NtpPacket, PACKET_LEN};
use tiny_ntp::server::{
    ClockState, LeapWarning, ServeDecision, ServerConfig, Source, broadcast, respond, silent_reason,
};

bind_interrupts!(struct Irqs {
    UART0_IRQ => BufferedInterruptHandler<UART0>;
    PIO0_IRQ_0 => PioInterruptHandler<PIO0>;
    PIO1_IRQ_0 => PioInterruptHandler<PIO1>;
    // The Ethernet serialiser's symbol feed, and the deserialiser's. One channel each.
    DMA_IRQ_0 => embassy_rp::dma::InterruptHandler<embassy_rp::peripherals::DMA_CH0>,
                 embassy_rp::dma::InterruptHandler<embassy_rp::peripherals::DMA_CH1>;
});

// --- Network identity -------------------------------------------------------------------------
//
// Fixed, because a transmit-only station cannot DHCP or answer ARP. A client identifies the source
// by its IP, so it has to be stable and it has to be on the client's subnet.

/// Locally-administered MAC (the `02:` prefix marks it as such, so it cannot collide with a real
/// assignment).
const SRC_MAC: MacAddr = MacAddr([0x02, 0x00, 0x00, 0xC0, 0xFF, 0xEE]);
/// Our address. **Change this to something on your LAN.** A client identifies the source by this,
/// and will ignore one that cannot belong to its own subnet.
const SRC_IP: Ipv4Addr = Ipv4Addr::new(192, 168, 0, 200);
/// Limited broadcast. Never forwarded by a router, and far easier to debug than multicast — no
/// IGMP, no switch snooping, no per-client group membership. `Ipv4Addr::NTP_MULTICAST` (224.0.1.1)
/// is the other legitimate choice once this works.
const DST_IP: Ipv4Addr = Ipv4Addr::BROADCAST;
const DST_MAC: MacAddr = MacAddr::BROADCAST;
/// Source port. Always 123: a client checks it, and it is what makes this an NTP server rather
/// than a host that happens to emit 48-byte datagrams.
const SRC_PORT: u16 = 123;
/// Destination port. 123 in production, or whatever `NTP_DST_PORT` said at build time.
///
/// Set it above 1024 to measure from an **unprivileged** listener: binding 123 needs root, which is
/// a real obstacle on a machine where the developer cannot elevate. Nothing about the frame changes
/// — same timestamps, same checksums, same line coding — so "did it arrive, and how far off was it"
/// is answered identically. Only a real NTP client needs 123, and that is the one thing a high port
/// cannot test.
///
/// ```sh
/// NTP_DST_PORT=10123 cargo run --release
/// ```
const DST_PORT: u16 = match option_env!("NTP_DST_PORT") {
    Some(s) => parse_port(s),
    None => 123,
};

/// Decimal `&str` to `u16` at compile time, so a typo in `NTP_DST_PORT` fails the build rather
/// than shipping a firmware that transmits somewhere unintended.
const fn parse_port(s: &str) -> u16 {
    let bytes = s.as_bytes();
    assert!(!bytes.is_empty(), "NTP_DST_PORT is empty");
    let mut value: u32 = 0;
    let mut i = 0;
    while i < bytes.len() {
        let digit = bytes[i];
        assert!(
            digit >= b'0' && digit <= b'9',
            "NTP_DST_PORT must be decimal"
        );
        value = value * 10 + (digit - b'0') as u32;
        assert!(value <= u16::MAX as u32, "NTP_DST_PORT is out of range");
        i += 1;
    }
    value as u16
}

// --- Server policy ----------------------------------------------------------------------------

/// How long after the second boundary the packet says it left: the whole path from the instant we
/// aim at to the first bit on the pair.
///
/// A broadcast client cannot sound the path, so whatever this misses lands directly in its clock.
/// Split in two because the two halves are measured by different instruments, and only one of them
/// is visible from inside.
///
/// Applied to the timestamp rather than by firing early. Firing early was tried and made things
/// worse: the residual moved 118.4 µs → 73.5 µs instead of to zero, its spread grew from 5.4 µs to
/// 26.5 µs, and 13% of seconds ran out of sleep and handed over immediately. The approach lands
/// anywhere within a 16 ms link-pulse interval, so taking 118 µs off what is left of it sometimes
/// takes all of it. Correcting the timestamp leaves the schedule alone.
const TX_LAG_NS: i64 = HANDOVER_LAG_NS + WIRE_LAG_NS;

/// Second boundary to the handover, from `residual_ns` below over 389 seconds: median 118.4 µs,
/// 102.4 to 133.4 µs, standard deviation 5.4 µs. The firmware can see this one because both ends
/// of it are clock reads it makes itself.
const HANDOVER_LAG_NS: i64 = 118_400;

/// Handover to the first bit on the pair. Nothing the firmware reads can show this: it starts at
/// the last instant the code can timestamp and ends outside the chip.
///
/// Measured on an oscilloscope with the GPS receiver's 1PPS as the reference, 60 single shots:
/// 236.19 µs mean from the second boundary to the first bit, standard deviation 6.87 µs. What each
/// shot caught was confirmed to be the frame and not a link pulse — the activity ran 81.8 µs, and
/// 102 byte at 10 Mbit/s is 81.6 µs. Subtracting the half above leaves this.
const WIRE_LAG_NS: i64 = 117_800;

const CFG: ServerConfig = ServerConfig {
    // log2 seconds of the resolution at which we can actually *timestamp* a transmission — not the
    // oscillator's. Over-claiming corrupts a client's source selection, so this covers the whole
    // observed spread rather than its standard deviation: with `TX_LAG_NS` carrying the median,
    // the residual runs −16.0 to +15.0 µs, and −15 is 30.5 µs.
    //
    // RFC 5905 defines precision as the system clock's, and strictly the transmit-timestamping
    // error is neither that nor root dispersion — the protocol has no field for it. Carrying it
    // here as a floor tells a client something true about the timestamps it is being handed.
    precision: -15,
    // Broadcast interval, log2 seconds: one per second.
    poll: 0,
    source: Source::ReferenceClock { id: *b"GPS\0" },
    // Our own uncertainty when freshly disciplined. Conservative for the same reason as `precision`.
    base_dispersion_ns: 1_000_000,
    // Bound on fractional frequency error during holdover. The crystal measures ~0.6 ppm off and
    // the estimator tracks it to the ppb level, so 1 ppm is a safe envelope.
    holdover_drift_ppb: 1_000,
    // Stop serving after an hour without a PPS edge.
    max_holdover_ns: 3_600 * 1_000_000_000,
};

const FRAME_LEN: usize = frame_len(PACKET_LEN);
const SYMBOL_WORDS: usize = encoded_words(FRAME_LEN);

/// How many outgoing frames to hexdump over RTT at start-up.
///
/// Debugging a transmit-only PHY is awkward: with no wired NIC to capture on and no scope on the
/// pair, the probe is the only window onto what was actually sent. Feeding these bytes to
/// `text2pcap` and `tshark` on the host checks the whole firmware path — real disciplined
/// timestamps, real framing, real checksums — with only the line coding and the wire left over.
const FRAME_DUMPS: u32 = 3;

/// Which GPIO carries the receiver's 1PPS.
const PPS_PIN: usize = 2;

/// Diagnostic only: ask the receiver for this 1PPS pulse width (ms) at boot, to find out which
/// excursion at the pin is the pulse.
///
/// `None` in normal builds — this changes the receiver's configuration, which outlives a reflash.
const PPS_WIDTH_EXPERIMENT: Option<u32> = None;

/// The receiver's power-on baud rate.
const GNSS_BOOT_BAUD: u32 = 9600;
/// What it would be raised to, to get the NMEA burst out of the way of the next PPS edge.
const GNSS_FAST_BAUD: u32 = 115_200;

/// Whether to raise the receiver's baud rate at boot.
///
/// On, and effectively required: at 9600 the NMEA burst lands on top of the next PPS edge (margin
/// mean 490 ms, sd 460 ms, **min 2 ms**), which makes the second-pairing a coin toss. Turning this
/// off leaves the receiver at 9600, and [`SOURCE_TRUSTED`] then keeps the server silent rather than
/// letting it announce a second it cannot vouch for.
///
/// An earlier attempt at this did fail — sending `PMTK251,115200` and switching the UART left the
/// link producing nothing but framing errors, because the module needs time after power-up before
/// it will take configuration and because the setting survives a reflash. Both are handled by
/// probing the port either side of the command rather than assuming; see `establish_gnss_link`.
///
/// `PMTK314` is the other route to the same margin, trimming the sentence set instead, and does not
/// require both ends to change rate in step. Untried.
const RAISE_BAUD: bool = true;

/// How the disciplined clock is configured, pinned rather than inherited.
///
/// Both values now match `rp-pps`'s own defaults, so `PpsGpsdo::new()` would do the same thing.
/// They are spelled out anyway: this is a time server, and the setting that decides *which UTC
/// second* a pulse is labelled with should not change under it because a library default moved.
///
/// # These were verified, not assumed
///
/// ZDA is the sentence the receiver defines against its pulse ("outputs the time associated with
/// the current 1PPS pulse … tells the time of the pulse that just occurred" — MT3333 NMEA
/// specification §2.2.7), and `SameSecond` is what that sentence therefore means. But the
/// specification-correct pair only *holds* if the sentence arrives comfortably before the following
/// edge, which at 9600 baud it does not:
///
/// ```text
/// margin from the time sentence to the next PPS edge, measured on this hardware
///   9600 baud    RMC  mean 490 ms, sd 460 ms, min   2 ms   (bimodal: either side of the edge)
///                ZDA  mean 866 ms, sd 218 ms, min   1 ms
///   115200 baud  RMC  mean 749 ms, sd  26 ms, min 718 ms
/// ```
///
/// So the firmware raises the receiver's baud rate at boot (see [`RAISE_BAUD`]); with that margin
/// in place these two constants are correct on their own, with no ±1 s correction anywhere.
/// Without it, either association is a coin toss that a longer NMEA burst can flip at runtime.
///
/// Verified against an NTP-synchronised host over the debug probe, so no network path could be
/// mistaken for clock error: `host − firmware = +0.11 … +0.20 s`, which is the probe's RTT polling
/// latency and not an offset. See `logs/20260818-ntp-bringup/`.
const PPS_NMEA: rp_pps::PpsNmeaAssociation = rp_pps::PpsNmeaAssociation::SameSecond;
/// Pair on ZDA — see [`PPS_NMEA`].
const TIME_SOURCE: rp_pps::NmeaTimeSource = rp_pps::NmeaTimeSource::Zda;

/// The disciplined clock. The two rp-pps runners write it; the NTP task reads it.
static CLOCK: BlockingMutex<CriticalSectionRawMutex, RefCell<PpsGpsdo>> =
    BlockingMutex::new(RefCell::new(PpsGpsdo::with_config(TIME_SOURCE, PPS_NMEA)));

/// Whether the receiver ended up in the configuration this server's correctness rests on.
///
/// Raising the baud rate is a *correctness* measure, not a throughput one: at 9600 the timing
/// sentence lands on the next PPS edge and the second can flip at runtime. A clock built on that
/// pairing may be a whole second out, and a stratum-1 server is believed — so if the upgrade did
/// not take, we keep the link alive and say nothing rather than announce a time we cannot vouch for.
static SOURCE_TRUSTED: AtomicBool = AtomicBool::new(false);

/// `Instant` as nanoseconds (µs resolution) — the query timebase for the disciplined clock.
fn now_ns() -> u64 {
    Instant::now().as_micros() * 1000
}

/// Software time of the instant the PIO counters were started, or `i64::MIN` before that.
///
/// The capture counter's zero is the write that enabled it, and that write is on the same line as
/// the read below it: no await between them, no other task in the way, so the two are a few
/// instructions apart rather than however long the executor takes to come round. Everything that
/// has a software time and wants to ask the disciplined clock about it converts through this.
///
/// This is the one place a clock read still decides anything, and it is read once, at a moment the
/// code chose rather than one it was handed.
static COUNTER_ORIGIN_NS: BlockingMutex<CriticalSectionRawMutex, Cell<i64>> =
    BlockingMutex::new(Cell::new(i64::MIN));

/// A software time, on the capture counter's scale.
fn capture_ns(software_ns: u64) -> Option<u64> {
    let origin = COUNTER_ORIGIN_NS.lock(|o| o.get());
    if origin == i64::MIN {
        return None;
    }
    (software_ns as i64).checked_sub(origin).map(|d| d as u64)
}

/// Now, on the capture counter's scale.
fn capture_now_ns() -> Option<u64> {
    capture_ns(now_ns())
}

/// The polarity of the 1PPS this firmware is built for.
///
/// [`rp_pps::pps_capture_program`] only ever watches for a rising edge, so an active-low receiver
/// has to be inverted into PIO. Without that the capture lands on the *end* of the pulse — one
/// pulse width past the second — and nothing on this board can tell: every interval is still
/// exactly one second and the frequency estimate is unaffected. It took a client on the other side
/// of the wire, which saw this server 100.23 ms slow (sd 0.55 ms, n=299).
///
/// The AE-GNSS-EXTANT carrier passes the module's 1PPS through one gate of its 74HC04, and its
/// manual gives the result as `1PPS 出力 : C-MOS ロジック (3.3V) レベル,
/// パルス幅 :100mS (アクティブ Low)`.
///
/// Configured rather than probed at boot. A probe needs a pulse to be running, and the receiver
/// drives 1PPS only once it has a fix, so at boot the pin is often resting — and a resting pin
/// still yields a majority level, which reads like a measurement and is a guess. On an unknown
/// board, sample the pin during bring-up with `rp_pps::PolarityProbe` and put the answer here.
const PPS_POLARITY: PpsPolarity = PpsPolarity::ActiveLow;

#[embassy_executor::task]
async fn pps_task(capture: TimedPpsCapture<'static, PIO0, 0>) {
    run_capture(capture, &CLOCK, now_ns).await
}

/// How far ahead a reply's departure is set when the reply is built (ns).
///
/// The transmit timestamp has to be written before the bytes it sits in are checksummed and
/// Manchester-encoded, so it is always a claim about a moment that has not happened. A broadcast
/// gets away with it by choosing the moment first and building into the wait. A reply can do the
/// same: pick a departure far enough ahead to build into, and then hold the frame until the clock
/// reaches it.
///
/// It has to clear the worst build, not the typical one - a reply that misses its own departure is
/// a reply that lies. Building was measured at 400-700 us on this board, so a millisecond.
///
/// Setting this from a measured average instead, and handing the frame over as soon as it was
/// ready, put the round-trip delay at -510 us on a link whose true delay is nanoseconds. The error
/// was not the average being wrong; it was that the build time varies by hundreds of microseconds
/// and no single number can stand for it.
const REPLY_LAG_NS: i64 = 1_000_000;

/// Words per capture on the receive side. See the client for the arithmetic; a request is smaller
/// than a reply, and this covers either.
const RX_CAPTURE_WORDS: usize = 256;

/// How long one capture covers (ns). The state machine starts on the frame's first bit and takes a
/// fixed number of samples, so the DMA finishes exactly this long after that bit arrived.
const RX_CAPTURE_NS: i64 = (RX_CAPTURE_WORDS as i64) * 16 * 25;

/// Everything between the capture ending and the timestamp taken for it (ns): the DMA interrupt,
/// the executor waking the task, and the clock read.
///
/// Measured on the client, which runs the same code on the same silicon — its 1PPS sat 21.41 us
/// (sd 3.68, n=24) past the second with nothing here. It is the receive-side counterpart of
/// `WIRE_LAG_NS`, and like it, a constant standing in for a timestamp nobody took.
const RX_LAG_NS: i64 = 21_400;

/// Largest frame the receive path will hold.
const RX_MAX_FRAME: usize = 256;

/// A request that arrived, and when.
///
/// The reply is not built here. Building it needs the transmitter, which belongs to [`ntp_task`],
/// so what crosses is the question and the moment it was asked - the two things that cannot be
/// recovered later.
struct PendingRequest {
    packet: NtpPacket,
    receive_unix_ns: i64,
    src_mac: MacAddr,
    src_ip: Ipv4Addr,
    src_port: u16,
}

/// Requests waiting for an answer.
///
/// Two deep. A client that asks faster than we answer is not owed a queue - it is owed the truth
/// about a recent moment, and a stale request answered late is worse than one dropped.
static REQUESTS: Channel<CriticalSectionRawMutex, PendingRequest, 2> = Channel::new();

/// Read the link and hand any request on it to [`ntp_task`].
#[embassy_executor::task]
async fn link_rx_task(mut rx: Rx10BaseT<'static, PIO1, 1>) {
    let mut words = [0u32; RX_CAPTURE_WORDS];
    let mut frame = [0u8; RX_MAX_FRAME];
    let mut seen: u32 = 0;
    let mut taken: u32 = 0;

    loop {
        rx.capture(&mut words).await;
        // Local, then UTC. `respond` puts this on the wire for a client to subtract from its own
        // timestamps, so it has to be in the client's units, not this board's uptime.
        let arrived_local = now_ns() as i64 - RX_CAPTURE_NS - RX_LAG_NS;
        seen = seen.wrapping_add(1);
        let Some(arrived_ns) = CLOCK.lock(|g| g.borrow().now_from_query_ns(arrived_local as u64))
        else {
            continue;
        };

        let Some(len) = decode_frame(&words, &mut frame) else {
            continue;
        };
        let Some(datagram) = parse_udp_frame(&frame[..len]) else {
            continue;
        };
        if datagram.dst_port != DST_PORT {
            continue;
        }
        let Some(packet) = NtpPacket::decode(datagram.payload) else {
            continue;
        };

        taken = taken.wrapping_add(1);
        let request = PendingRequest {
            packet,
            receive_unix_ns: arrived_ns,
            src_mac: datagram.src_mac,
            src_ip: datagram.src_ip,
            src_port: datagram.src_port,
        };
        // Never block the receiver on the transmitter: a full queue means the answer is already
        // late, and waiting here would make us miss the next question as well.
        if REQUESTS.try_send(request).is_err() {
            warn!("NTPRX {} dropped, still answering the last one", seen);
        }
        if taken.is_multiple_of(16) {
            info!("NTPRX seen={} requests={}", seen, taken);
        }
    }
}

/// Width of the 1PPS on GP6, matching the receiver's own so the two traces have the same shape.
const PPS_OUT_PULSE_NS: u32 = 100_000_000;

/// How early the word for the next edge is pushed, measured against the edge before it. See
/// [`rp_pps::PpsSchedule`]: what the state machine pulls before an edge is the interval that
/// follows, so the deadline is one edge earlier than the edge being positioned.
const PPS_OUT_LEAD_NS: i64 = 200_000_000;

/// Put the disciplined clock on GP6, so the thing this server is announcing can be seen.
///
/// The clock is already being read once a second for the packet; this reads it for a pin. The two
/// answers come from the same estimate, so an oscilloscope comparing GP6 against the receiver's own
/// 1PPS is measuring what the packets carry — including any constant this firmware is wrong by,
/// which is the point.
#[embassy_executor::task]
async fn pps_out_task(mut out: PpsOutput<'static, PIO0, 1>, mut schedule: PpsSchedule) {
    let mut edges: u32 = 0;

    loop {
        let push_at = schedule.edge_ns() - PPS_OUT_LEAD_NS;
        let now = now_ns() as i64;
        if push_at > now {
            Timer::after(Duration::from_micros(((push_at - now) / 1000) as u64)).await;
        }

        let predicted = schedule.predicted_edge_ns();
        let utc = CLOCK.lock(|g| g.borrow().now_from_capture_ns(predicted as u64));
        let trusted = SOURCE_TRUSTED.load(Ordering::Relaxed);

        let step = match utc {
            Some(utc) if trusted => schedule.advance(0, pps_lateness_ns(utc)),
            // Nothing to steer by yet: free-run at the nominal second so the FIFO stays fed.
            _ => schedule.step(0, 0),
        };
        if step.acquired {
            info!("PPSOUT placed, corr_ns={}", step.correction_ns);
        }

        if !out.set_period(step.period_word) {
            warn!("PPSOUT push dropped, FIFO full");
        }
        edges = edges.wrapping_add(1);
        if edges <= 8 || edges.is_multiple_of(16) {
            let landed = CLOCK
                .lock(|g| g.borrow().now_from_capture_ns(step.edge_ns as u64))
                .map(pps_lateness_ns)
                .unwrap_or(0);
            info!(
                "PPSOUT edges={} word={} corr_ns={} asked_late_ns={} landed_late_ns={}",
                edges,
                step.period_word,
                step.correction_ns,
                utc.map(pps_lateness_ns).unwrap_or(0),
                landed
            );
        }
    }
}

/// Signed distance from `utc_ns` to the nearest second boundary: positive when it is past one.
fn pps_lateness_ns(utc_ns: i64) -> i64 {
    let into = utc_ns.rem_euclid(1_000_000_000);
    if into > 500_000_000 {
        into - 1_000_000_000
    } else {
        into
    }
}

#[embassy_executor::task]
async fn nmea_task(rx: BufferedUartRx) {
    run_nmea(rx, &CLOCK).await
}

/// Read everything the server policy needs, in one lock so the values are mutually consistent.
fn clock_state() -> (Option<i64>, ClockState) {
    let q = now_ns();
    let c = capture_now_ns();
    CLOCK.lock(|g| {
        let g = g.borrow();
        // Time in the capture timebase, ageing in the software one. The first is what must not
        // carry a scheduling delay; the second only gates service and is a millisecond quantity.
        let now = c.and_then(|c| g.now_from_capture_ns(c));
        let holdover_ns = g.holdover_ns(q);
        (
            now,
            ClockState {
                // NTP's reference timestamp is when the clock was last corrected, which is exactly
                // `now` minus how long we have been extrapolating.
                last_update_unix_ns: now.map(|n| n - holdover_ns as i64),
                holdover_ns,
                frequency_locked: g.frequency_locked(),
                // The MT3333 does not surface the almanac's leap-second announcement, so we have
                // nothing to pass on. Clients see NoWarning, which is what we actually know.
                leap: LeapWarning::None,
            },
        )
    })
}

/// Build and send one reply.
///
/// The transmit timestamp is written before the frame is encoded, so it is a claim about a moment
/// that has not happened yet: `REPLY_LAG_NS` is what stands in for the encoding, the handover and
/// the wire. `build_ns` in the log is the part of that this firmware can see, and is what the
/// constant was set from.
async fn answer(
    tx: &mut Tx10BaseT<'static, PIO1, 0>,
    request: &PendingRequest,
    ip_id: u16,
    frame: &mut [u8; FRAME_LEN],
    symbols: &mut [u32; SYMBOL_WORDS],
) {
    let started = now_ns() as i64;
    let (now, state) = clock_state();
    let Some(now) = now else {
        return;
    };
    // Both timestamps go on the wire for a client to subtract from its own, so both are moved on
    // to the receiver's second. A common shift would cancel out of the client's round-trip delay
    // but not out of its offset, which is exactly the number it sets its clock by: without this the
    // client tracked this board's clock faithfully, 51.73 us (sd 7.76, n=16) behind the receiver.
    let decision = respond(
        &CFG,
        &state,
        &request.packet,
        request.receive_unix_ns,
        now + REPLY_LAG_NS,
    );
    let packet = match decision {
        ServeDecision::Silent(reason) => {
            warn!("NTP reply withheld: {}", defmt::Debug2Format(&reason));
            return;
        }
        ServeDecision::Serve(packet) => packet,
    };

    let payload = packet.encode();
    let spec = UdpFrameSpec {
        src_mac: SRC_MAC,
        dst_mac: request.src_mac,
        src_ip: SRC_IP,
        dst_ip: request.src_ip,
        src_port: DST_PORT,
        dst_port: request.src_port,
        ip_id,
        ttl: 1,
        payload: &payload,
    };
    let Some(len) = build_udp_frame(&spec, frame) else {
        warn!("reply does not fit the frame buffer");
        return;
    };
    let Some(words) = encode_frame(&frame[..len], symbols) else {
        warn!("reply does not fit the symbol buffer");
        return;
    };
    // The frame is ready; the moment it claims is not yet. Hold it until the clock is one wire lag
    // short of the departure written inside it, so that what goes out is what was promised.
    let departure = now + REPLY_LAG_NS;
    let handover_ns = now_ns() as i64 - started;
    let slack_ns = departure - WIRE_LAG_NS - (now + handover_ns);
    if slack_ns > 0 {
        Timer::after_micros((slack_ns / 1000) as u64).await;
    }
    tx.send(&symbols[..words]).await;

    info!(
        "NTPREPLY rx_ns={} claimed_tx_ns={} handover_ns={} slack_ns={} bytes={}",
        request.receive_unix_ns, departure, handover_ns, slack_ns, len
    );
    if slack_ns <= 0 {
        // The build overran the departure it had promised. Every reply from here is late by
        // whatever this says, and no client can tell.
        warn!("NTP reply missed its own departure by {} ns", -slack_ns);
    }
}

/// Broadcast one NTP packet per UTC second, and keep the link alive in between.
///
/// Both jobs live in one task because they share the transmitter. Interleaving them here also keeps
/// the ordering obvious: a link pulse is never emitted while a frame is going out.
#[embassy_executor::task]
async fn ntp_task(mut tx: Tx10BaseT<'static, PIO1, 0>) {
    let mut frame = [0u8; FRAME_LEN];
    let mut symbols = [0u32; SYMBOL_WORDS];
    let mut ip_id: u16 = 0;
    let mut sent: u32 = 0;
    let nlp_us = NLP_INTERVAL_US as u64;
    // The last second we actually served. Guards against emitting the same timestamp twice, which
    // is otherwise reachable: after transmitting we can still be a few hundred microseconds *before*
    // the boundary we aimed at, and would then aim at it again.
    let mut last_target_ns: i64 = i64::MIN;

    loop {
        // A question outranks the beacon. Answering is the whole point of having a receive path:
        // it is what lets a client measure the round trip instead of assuming it.
        if let Ok(request) = REQUESTS.try_receive() {
            ip_id = ip_id.wrapping_add(1);
            answer(&mut tx, &request, ip_id, &mut frame, &mut symbols).await;
            continue;
        }

        // The receiver never reached the rate the pairing depends on, so the second this clock is
        // built on may be wrong. Hold the link up — a dropped link is harder to diagnose than a
        // quiet one — and serve nothing. `main` has already said why on RTT.
        if !SOURCE_TRUSTED.load(Ordering::Relaxed) {
            tx.link_pulse();
            Timer::after_micros(nlp_us).await;
            continue;
        }

        let (now, state) = clock_state();

        // Without a UTC epoch there is no second boundary to aim at. Keep the link up and wait.
        let Some(now) = now else {
            tx.link_pulse();
            Timer::after_micros(nlp_us).await;
            continue;
        };

        // The next whole UTC second we have not already served.
        let mut target_unix_ns = (now.div_euclid(1_000_000_000) + 1) * 1_000_000_000;
        if target_unix_ns <= last_target_ns {
            target_unix_ns = last_target_ns + 1_000_000_000;
        }
        let wait_ns = target_unix_ns - now;

        // Sleep towards the boundary, but **never past it**: the sleep is capped at the link-pulse
        // interval rather than being a fixed tick. Polling on a fixed 16 ms period with a 16 ms
        // threshold means any jitter either steps over the boundary (dropping that second) or lands
        // short of it twice (sending it twice) — both were observed on hardware before this.
        if wait_ns > (nlp_us * 1000) as i64 {
            tx.link_pulse();
            Timer::after_micros(nlp_us).await;
            continue;
        }
        last_target_ns = target_unix_ns;

        // Build *before* sleeping. The transmit timestamp we are about to write says the frame
        // left at `target_unix_ns + TX_LAG_NS`, so anything between waking and handing the buffer
        // to the PIO that `TX_LAG_NS` does not already account for is a systematic lag added to
        // every packet we serve. Encoding 48 NTP bytes into a
        // ~90-byte frame and then into Manchester symbols is far more than the ~1 µs of timestamp
        // resolution `CFG.precision` advertises, so it cannot sit on that path.
        //
        // What this costs is freshness: the packet's contents are those of a moment up to one
        // link-pulse interval before it leaves. That is affordable and the gate below is not —
        // holdover grows dispersion by `holdover_drift_ppb` over that interval, single-digit
        // nanoseconds against a floor of a millisecond, whereas *whether we may speak at all* has
        // to be answered at transmission.
        let (_, state_now) = clock_state();
        let state = if state_now.last_update_unix_ns.is_some() {
            state_now
        } else {
            state
        };

        match broadcast(&CFG, &state, target_unix_ns + TX_LAG_NS) {
            ServeDecision::Silent(reason) => {
                // Deliberately quiet: an unsynchronised beacon is only noise to a client that
                // cannot interrogate us. Say why on RTT so the reason is visible.
                warn!("NTP silent: {}", defmt::Debug2Format(&reason));
                tx.link_pulse();
                Timer::after_micros(nlp_us).await;
            }
            ServeDecision::Serve(packet) => {
                ip_id = ip_id.wrapping_add(1);
                let payload = packet.encode();
                let spec = UdpFrameSpec {
                    src_mac: SRC_MAC,
                    dst_mac: DST_MAC,
                    src_ip: SRC_IP,
                    dst_ip: DST_IP,
                    src_port: SRC_PORT,
                    dst_port: DST_PORT,
                    ip_id,
                    // Broadcast is link-local; one hop is all it may take.
                    ttl: 1,
                    payload: &payload,
                };
                let Some(len) = build_udp_frame(&spec, &mut frame) else {
                    warn!("NTP frame buffer too small");
                    continue;
                };
                let Some(words) = encode_frame(&frame[..len], &mut symbols) else {
                    warn!("NTP symbol buffer too small");
                    continue;
                };

                // Only now sleep to the boundary, and re-read the clock to do it: the build above
                // consumed part of the interval, and `wait_ns` was measured before it. Sleeping the
                // stale figure would overshoot by exactly the build time — the error this ordering
                // exists to remove.
                let (before_wait, _) = clock_state();
                if let Some(before_wait) = before_wait {
                    let remaining_ns = target_unix_ns - before_wait;
                    Timer::after_micros((remaining_ns.max(0) / 1000) as u64).await;
                }

                // Disciplined UTC as close to the handover as we can read it. `residual_ns` below
                // is this minus where `HANDOVER_LAG_NS` says the handover should be, so zero means
                // that half is still calibrated. It says nothing about `WIRE_LAG_NS`, which starts
                // after this read. Taking it costs a lock and lands inside the number it reports,
                // so the figure errs high.
                let (utc_at_handover, state_at_handover) = clock_state();

                // The packet above was built from the clock as it stood before the sleep, so its
                // *eligibility* is that old too. Lock can drop and holdover can cross its limit in
                // the meantime, and the policy is about the state at transmission — otherwise
                // moving the build off the critical path would have bought accuracy by paying with
                // correctness. Cheaper than `broadcast()`: this asks the gate without rebuilding a
                // packet we would only discard.
                if let Some(reason) = silent_reason(&CFG, &state_at_handover) {
                    warn!("NTP silent at handover: {}", defmt::Debug2Format(&reason));
                    tx.link_pulse();
                    Timer::after_micros(nlp_us).await;
                    continue;
                }

                let before = now_ns();
                tx.send(&symbols[..words]).await;
                let after = now_ns();

                sent = sent.wrapping_add(1);
                // Dump the first few frames over RTT. This is the only way to inspect what actually
                // went out when there is no wired NIC to capture on and no scope on the pair: the
                // host can turn these bytes into a pcap (`text2pcap`) and let Wireshark judge them,
                // which covers everything except the Manchester coding and the pair itself.
                if sent <= FRAME_DUMPS {
                    info!("NTPFRAME n={} bytes={=[u8]:02x}", sent, &frame[..len]);
                }
                // Everything a host-side measurement needs to line its receive timestamps up with
                // what we believed we were sending, and to see the scheduling error separately from
                // the path delay.
                info!(
                    "NTPTX n={} target_unix_ns={} residual_ns={} tx_lag_ns={} dma_us={} bytes={} words={} disp_ns={} holdover_ns={}",
                    sent,
                    target_unix_ns,
                    utc_at_handover
                        .map(|u| u - target_unix_ns - HANDOVER_LAG_NS)
                        .unwrap_or(0),
                    TX_LAG_NS,
                    (after - before) / 1000,
                    len,
                    words,
                    tiny_ntp::server::root_dispersion_ns(&CFG, state.holdover_ns),
                    state.holdover_ns,
                );
            }
        }
    }
}

/// Send `$<payload>*<checksum>\r\n` to the receiver.
async fn send_pmtk<W: embedded_io_async::Write>(tx: &mut W, payload: &str) {
    let cs = rp_pps::nmea_checksum(payload.as_bytes());
    let mut line: heapless::String<64> = heapless::String::new();
    if core::fmt::Write::write_fmt(&mut line, format_args!("${payload}*{cs:02X}\r\n")).is_ok() {
        let _ = tx.write_all(line.as_bytes()).await;
    }
}

/// Read until a complete NMEA sentence arrives, or the deadline passes.
///
/// Used as the liveness probe on both sides of a baud change: the point is not the contents but
/// that framing works at all, which is exactly what a wrong rate destroys.
async fn await_nmea(uart: &mut BufferedUart, timeout: Duration) -> bool {
    let mut assembler = rp_pps::NmeaLineAssembler::new();
    let mut buf = [0u8; 64];
    let deadline = Instant::now() + timeout;
    loop {
        let Some(remaining) = deadline.checked_duration_since(Instant::now()) else {
            return false;
        };
        let Ok(read) = with_timeout(remaining, uart.read(&mut buf)).await else {
            return false;
        };
        // Framing and break errors are what a wrong baud rate looks like; keep waiting rather than
        // treating them as a verdict, since a few are normal right after a rate change.
        let Ok(n) = read else { continue };
        for &b in &buf[..n] {
            if let Some(sentence) = assembler.push(b)
                && sentence.starts_with(b"$G")
            {
                return true;
            }
        }
    }
}

/// Find the receiver, then get it onto [`GNSS_FAST_BAUD`] — verifying at every step.
///
/// Returns the rate the link ended up on.
///
/// # Why it probes instead of assuming
///
/// Two things make "open at 9600 and send PMTK251" wrong:
///
/// 1. **The rate survives a firmware reflash.** `PMTK251` reverts only on a full cold start or
///    standby (MT3333 NMEA specification §2.3.14), so after re-flashing the RP2040 the module is
///    still at whatever it was last set to. Assuming the power-on rate means finding silence and
///    concluding the receiver is dead — which is exactly what happened before this probed.
/// 2. **The change cannot be acknowledged.** `PMTK251` has no ACK and could not have one: the reply
///    would have to be sent at a rate one end has not adopted yet. Nothing in the specification says
///    how long the switch takes, either.
///
/// So the sleep below is not load-bearing. It only has to be *usually* long enough; if it is not,
/// the probe afterwards finds the module at whichever rate it actually settled on and follows it.
/// That is a much better property than a carefully tuned constant, because the constant would be
/// tuned against one module on one day.
async fn establish_gnss_link(uart: &mut BufferedUart) -> u32 {
    let Some(found) = probe_gnss_baud(uart).await else {
        warn!("no NMEA at either rate — check the wiring, or power-cycle the receiver");
        return GNSS_BOOT_BAUD;
    };
    if found == GNSS_FAST_BAUD {
        info!("GNSS already at {}", GNSS_FAST_BAUD);
        return found;
    }

    // At the slow rate, where the sentence burst (~640 ms) collides with the next PPS edge. Ask for
    // the fast one, which shrinks the burst to ~53 ms and turns the second-pairing from a coin toss
    // into a margin.
    send_pmtk(uart, "PMTK251,115200").await;
    // Enough for the command to clock out at 9600 (~21 ms) and for the module to finish the sentence
    // it is part-way through. Generous rather than tuned — see above, and note this is boot-time
    // only, against a clock that needs seconds to lock regardless.
    Timer::after_millis(500).await;

    match probe_gnss_baud(uart).await {
        Some(rate) if rate == GNSS_FAST_BAUD => {
            info!("GNSS baud raised to {}", GNSS_FAST_BAUD);
            rate
        }
        Some(rate) => {
            warn!("PMTK251 did not take; continuing at {}", rate);
            rate
        }
        None => {
            warn!(
                "lost the receiver after PMTK251; leaving the port at {}",
                GNSS_BOOT_BAUD
            );
            uart.set_baudrate(GNSS_BOOT_BAUD);
            GNSS_BOOT_BAUD
        }
    }
}

/// Try each supported rate until NMEA frames, leaving the port on the one that worked.
async fn probe_gnss_baud(uart: &mut BufferedUart) -> Option<u32> {
    for baud in [GNSS_BOOT_BAUD, GNSS_FAST_BAUD] {
        uart.set_baudrate(baud);
        if await_nmea(uart, Duration::from_secs(3)).await {
            return Some(baud);
        }
    }
    None
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());
    info!(
        "pico-ntp: Stratum-1 NTP broadcast (NMEA UART0/GP1 @9600, PPS in GP2, PPS out GP6, 10BASE-T GP16/GP17)"
    );

    // PIO0: the 1PPS capture that disciplines the clock.
    let Pio {
        mut common,
        sm0,
        sm1,
        ..
    } = Pio::new(p.PIO0, Irqs);
    // Stopped, and started later in the same write as the 1PPS output. The two counters are then
    // one timebase, which is what lets the output be placed without reading a software clock.
    let capture = TimedPpsCapture::new_stopped(
        &mut common,
        sm0,
        p.PIN_2,
        clk_sys_freq(),
        &rp_pps::pps_capture_program(),
    );
    // After the capture has claimed the pin, never before: assigning it to PIO rewrites the same
    // control register the inversion lives in, so an earlier setting is silently dropped. Found by
    // measuring — the offset came back at −98 ms with the log still saying it had inverted.
    set_capture_polarity(PPS_PIN, PPS_POLARITY);
    info!("PPS on GP{}: configured active low", PPS_PIN);

    // PIO1: the Ethernet serialiser. A separate PIO block so the two never contend for a state
    // machine or for instruction memory.
    let Pio {
        common: mut eth_common,
        sm0: eth_sm,
        sm1: eth_sm1,
        ..
    } = Pio::new(p.PIO1, Irqs);
    let dma = embassy_rp::dma::Channel::new(p.DMA_CH0, Irqs);
    let tx = Tx10BaseT::new(
        &mut eth_common,
        eth_sm,
        p.PIN_16, // TX−
        p.PIN_17, // TX+
        dma,
        clk_sys_freq(),
    );

    // The same block's second state machine reads the other pair, which is where a client's
    // question arrives. The serialiser is loaded first and is pinned to offset zero (`out pc`
    // indexes it by symbol value), so the deserialiser lands after it.
    let eth_rx_dma = embassy_rp::dma::Channel::new(p.DMA_CH1, Irqs);
    let eth_rx = Rx10BaseT::new(
        &mut eth_common,
        eth_sm1,
        p.PIN_18, // TX− from the other board
        p.PIN_19, // TX+
        eth_rx_dma,
        clk_sys_freq(),
    );

    // UART0: the receiver's NMEA.
    static TX_BUF: StaticCell<[u8; 32]> = StaticCell::new();
    static RX_BUF: StaticCell<[u8; 256]> = StaticCell::new();
    let mut config = UartConfig::default();
    config.baudrate = GNSS_BOOT_BAUD;
    let mut uart = BufferedUart::new(
        p.UART0,
        p.PIN_0,
        p.PIN_1,
        Irqs,
        TX_BUF.init([0; 32]),
        RX_BUF.init([0; 256]),
        config,
    );

    // Raise the receiver's baud rate before doing anything else.
    //
    // This is a *correctness* measure, not a throughput one. `rp-pps` pairs a PPS edge with the UTC
    // second from an NMEA sentence, and at 9600 baud the sentence burst runs ~640 ms and starts a
    // few hundred ms after the edge — so the timing sentence lands essentially on top of the *next*
    // pulse (measured margin: mean 490 ms, sd 460 ms, min 2 ms, bimodal either side of the edge).
    // Whichever association is configured, a longer burst — more satellites, more GSV — can flip
    // the pairing and move the clock by a whole second at runtime.
    //
    // Twelve times the baud makes the burst ~53 ms, which puts the sentence hundreds of
    // milliseconds clear of the next edge and turns a coin toss into a margin.
    //
    // Not persistent: the module reverts to 9600 on power loss, so this runs on every boot.
    // Whether that succeeded decides whether we may serve at all: the pairing this clock is built
    // on is only trustworthy at the fast rate, and every fallback below leaves us at the slow one.
    let gnss_baud = if RAISE_BAUD {
        establish_gnss_link(&mut uart).await
    } else {
        GNSS_BOOT_BAUD
    };

    // Diagnostic: ask the receiver for a specific 1PPS pulse width, so the pin can be watched to
    // see which excursion follows. The vendor documents the *rising* edge as the time mark, and
    // this pin measures idle-high — one of those has to give, and the width is the fingerprint that
    // says which. `PMTK285,Type,WidthMs`; type 4 keeps the pulse coming regardless of fix.
    if let Some(width_ms) = PPS_WIDTH_EXPERIMENT {
        let mut cmd: heapless::String<32> = heapless::String::new();
        if core::fmt::Write::write_fmt(&mut cmd, format_args!("PMTK285,4,{width_ms}")).is_ok() {
            send_pmtk(&mut uart, &cmd).await;
            info!("PPS width experiment: asked for {} ms", width_ms);
            Timer::after_millis(500).await;
        }
    }
    if gnss_baud == GNSS_FAST_BAUD {
        SOURCE_TRUSTED.store(true, Ordering::Relaxed);
    } else {
        warn!(
            "NTP disabled: receiver at {} baud, where the PPS-NMEA pairing can be a second out",
            gnss_baud
        );
    }

    let (_uart_tx, uart_rx) = uart.split();

    // The 1PPS out on GP6, on the capture's block. Built here rather than with the capture, and the
    // ordering is not cosmetic: the state machine starts running the moment it is enabled and wants
    // a fresh period every second, but its task cannot run until this function stops. Enabling it
    // before the baud negotiation above left it pulling an empty FIFO, which loads a spent counter
    // rather than the last period - one ~34 s interval, and the output was gone for half a minute.
    let pps_high = output_high_cycles(clk_sys_freq(), PPS_OUT_PULSE_NS);
    let pps_initial = output_period_cycles(clk_sys_freq(), pps_high);
    let pps_out = PpsOutput::new_stopped(&mut common, sm1, p.PIN_6, pps_high, pps_initial);
    // Zero, not a clock reading. Both state machines are enabled by the write below, so the moment
    // this schedule counts from is the moment the capture counter started, and the schedule's edges
    // and the captured edges are on one scale. Reading a clock here instead put whatever that read
    // cost onto every edge afterwards, which is what `CLOCK_OFFSET_NS` used to subtract.
    let pps_schedule = PpsSchedule::at_enable(
        clk_sys_freq(),
        pps_high,
        PpsScheduleConfig::default(),
        0,
        pps_initial,
    );
    start_in_sync(
        embassy_rp::pac::PIO0,
        PpsCapture::<PIO0, 0>::sm_mask() | PpsOutput::<PIO0, 1>::sm_mask(),
    );
    // Next line, deliberately: see `COUNTER_ORIGIN_NS`.
    COUNTER_ORIGIN_NS.lock(|o| o.set(now_ns() as i64));

    // Neither `Common` may be dropped. embassy-rp releases a PIO block's pins - resets their
    // FUNCSEL to NULL - once the `Common` and the state machines it handed out are all gone, and
    // `main` returning is enough to start that. The state machines live on in their tasks and keep
    // running; the pin they were driving quietly stops being connected to them.
    //
    // It cost an afternoon. GP6 read funcsel 7 at the last line of `main` and 31 two seconds later,
    // with the state machine still enabled and still consuming a period every second, driving a pad
    // that was no longer listening. Nothing in the firmware's own telemetry could see it.
    core::mem::forget(common);
    core::mem::forget(eth_common);

    spawner.spawn(pps_task(capture).unwrap());
    spawner.spawn(nmea_task(uart_rx).unwrap());
    spawner.spawn(ntp_task(tx).unwrap());
    spawner.spawn(link_rx_task(eth_rx).unwrap());
    spawner.spawn(pps_out_task(pps_out, pps_schedule).unwrap());
    // Debug only: a unicast exchange carried over the probe, so a real client can be measured
    // against a link that cannot receive. See `swd_rx`.
    #[cfg(feature = "swd-rx")]
    spawner.spawn(swd_rx::swd_rx_task().unwrap());

    // Report the disciplined clock once a second, independently of whether we are serving, so a
    // silent server can be told apart from a stopped one.
    loop {
        Timer::after_secs(1).await;
        let q = now_ns();
        let (now, ppb, locked, holdover) = CLOCK.lock(|g| {
            let g = g.borrow();
            (
                g.now_from_query_ns(q),
                g.freq_ppb(),
                g.frequency_locked(),
                g.holdover_ns(q),
            )
        });
        match now {
            Some(now) => info!(
                "TIME unix_ns={} ppb={} locked={} holdover_ms={}",
                now,
                ppb,
                locked as u8,
                holdover / 1_000_000
            ),
            None => info!("TIME waiting for a UTC epoch (no PPS+NMEA pairing yet)"),
        }
    }
}
