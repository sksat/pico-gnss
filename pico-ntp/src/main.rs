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

use core::cell::RefCell;
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
use embassy_sync::blocking_mutex::Mutex as BlockingMutex;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_time::{Duration, Instant, Timer, with_timeout};
use embedded_io_async::Read as _;
use static_cell::StaticCell;

use pico_10base_t::embassy::Tx10BaseT;
use pico_10base_t::frame::{Ipv4Addr, MacAddr, UdpFrameSpec, build_udp_frame, frame_len};
use pico_10base_t::phy::{NLP_INTERVAL_US, encode_frame, encoded_words};
use rp_pps::PpsGpsdo;
use rp_pps::embassy::{TimedPpsCapture, run_capture, run_nmea};
use tiny_ntp::packet::PACKET_LEN;
use tiny_ntp::server::{ClockState, LeapWarning, ServeDecision, ServerConfig, Source, broadcast};

bind_interrupts!(struct Irqs {
    UART0_IRQ => BufferedInterruptHandler<UART0>;
    PIO0_IRQ_0 => PioInterruptHandler<PIO0>;
    PIO1_IRQ_0 => PioInterruptHandler<PIO1>;
    // The Ethernet serialiser's symbol feed. This firmware uses exactly one DMA channel.
    DMA_IRQ_0 => embassy_rp::dma::InterruptHandler<embassy_rp::peripherals::DMA_CH0>;
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
/// Destination port. 123 in production.
///
/// Set it above 1024 to measure from an **unprivileged** listener: binding 123 needs root, which is
/// a real obstacle on a machine where the developer cannot elevate. Nothing about the frame changes
/// — same timestamps, same checksums, same line coding — so "did it arrive, and how far off was it"
/// is answered identically. Only a real NTP client needs 123, and that is the one thing a high port
/// cannot test.
const DST_PORT: u16 = 123;

// --- Server policy ----------------------------------------------------------------------------

/// How long it takes from `send()` returning control to the first bit reaching the wire.
///
/// **Currently a placeholder.** It has to be measured — a broadcast client cannot sound the path,
/// so this offset lands directly in every client's clock. Until then the transmit timestamp is
/// systematically early by whatever this really is, and `PRECISION` is set to say so.
const TX_LEAD_NS: i64 = 0;

const CFG: ServerConfig = ServerConfig {
    // log2 seconds of the resolution at which we can actually *timestamp* a transmission — not the
    // oscillator's. Deliberately pessimistic at ~1 µs until TX_LEAD_NS is measured; over-claiming
    // here corrupts a client's source selection, which is worse than under-claiming.
    precision: -20,
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

#[embassy_executor::task]
async fn pps_task(capture: TimedPpsCapture<'static, PIO0, 0>) {
    run_capture(capture, &CLOCK, now_ns).await
}

#[embassy_executor::task]
async fn nmea_task(rx: BufferedUartRx) {
    run_nmea(rx, &CLOCK).await
}

/// Read everything the server policy needs, in one lock so the values are mutually consistent.
fn clock_state() -> (Option<i64>, ClockState) {
    let q = now_ns();
    CLOCK.lock(|g| {
        let g = g.borrow();
        let now = g.now_from_query_ns(q);
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
        let wait_ns = target_unix_ns - now - TX_LEAD_NS;

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

        // Build *before* sleeping. The transmit timestamp we are about to write says the frame's
        // first bit is at `target_unix_ns`, so everything between waking and handing the buffer to
        // the PIO is a systematic lag added to every packet we serve. Encoding 48 NTP bytes into a
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

        match broadcast(&CFG, &state, target_unix_ns) {
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
                    let remaining_ns = target_unix_ns - before_wait - TX_LEAD_NS;
                    Timer::after_micros((remaining_ns.max(0) / 1000) as u64).await;
                }

                // Disciplined UTC as close to the handover as we can read it. `sched_ns` below is
                // this minus the instant we advertised, i.e. the residual lag that `TX_LEAD_NS`
                // exists to cancel — and it is what has to be measured before that constant can be
                // anything but zero. Reading it costs a lock and lands inside the number it
                // reports, so the figure errs high, which is the safe direction for a correction.
                let (utc_at_handover, _) = clock_state();
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
                    "NTPTX n={} target_unix_ns={} sched_ns={} tx_lead_ns={} dma_us={} bytes={} words={} disp_ns={} holdover_ns={}",
                    sent,
                    target_unix_ns,
                    utc_at_handover.map(|u| u - target_unix_ns).unwrap_or(0),
                    TX_LEAD_NS,
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
    info!("pico-ntp: Stratum-1 NTP broadcast (NMEA UART0/GP1 @9600, PPS GP2, 10BASE-T GP16/GP17)");

    // PIO0: the 1PPS capture that disciplines the clock.
    let Pio {
        mut common, sm0, ..
    } = Pio::new(p.PIO0, Irqs);
    let capture = TimedPpsCapture::new(&mut common, sm0, p.PIN_2, clk_sys_freq());

    // PIO1: the Ethernet serialiser. A separate PIO block so the two never contend for a state
    // machine or for instruction memory.
    let Pio {
        common: mut eth_common,
        sm0: eth_sm,
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
    if gnss_baud == GNSS_FAST_BAUD {
        SOURCE_TRUSTED.store(true, Ordering::Relaxed);
    } else {
        warn!(
            "NTP disabled: receiver at {} baud, where the PPS-NMEA pairing can be a second out",
            gnss_baud
        );
    }

    let (_uart_tx, uart_rx) = uart.split();

    spawner.spawn(pps_task(capture).unwrap());
    spawner.spawn(nmea_task(uart_rx).unwrap());
    spawner.spawn(ntp_task(tx).unwrap());

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
