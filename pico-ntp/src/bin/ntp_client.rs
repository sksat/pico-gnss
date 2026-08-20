//! The other board in the pair: take the time off the link and put it on a pin.
//!
//! ```text
//!   server GP16 (TX-) ──► GP18  PIO0 SM0 + DMA ──► frame ──► NTP ──┐
//!   server GP17 (TX+) ──► GP19                                     │
//!                                                                 ▼
//!                                                          NtpDiscipline
//!                                                                 │
//!                                          PIO1 SM0 ──► GP6 (1PPS out)
//! ```
//!
//! There is no receiver here and no crystal worth the name. Everything this board knows about the
//! time arrives over four wires, and the 1PPS on GP6 is the whole of what it has to say back — it
//! is the only output an oscilloscope can compare against the server's own, and against the GPS
//! receiver that both of them ultimately come from.
//!
//! It asks rather than listens. A broadcast can only be believed; a question and its answer carry
//! four timestamps, and four timestamps separate how far the clocks differ from how long the path
//! took. That the path here is two wires and a state machine does not change the argument — it
//! changes only how small the answer comes out.
//!
//! **The output edge is computed, not measured.** See [`rp_pps::PpsSchedule`]: the edges follow from
//! the period words, so nothing on this board watches GP6. That leaves one constant unknown, the
//! local timestamp of the moment the state machine was enabled, and no way to see it from in here.
//! It is a fixed offset on the output, and the scope is what it is for.

#![no_std]
#![no_main]

use core::cell::RefCell;

use defmt::{info, warn};
use embassy_executor::Spawner;
use embassy_rp::bind_interrupts;
use embassy_rp::clocks::clk_sys_freq;
use embassy_rp::peripherals::{PIO0, PIO1};
use embassy_rp::pio::{InterruptHandler as PioInterruptHandler, Pio};
use embassy_sync::blocking_mutex::Mutex as BlockingMutex;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_time::{Duration, Instant, Timer};

use pico_10base_t::embassy::{Rx10BaseT, Tx10BaseT};
use pico_10base_t::frame::{
    Ipv4Addr, MacAddr, UdpFrameSpec, build_udp_frame, frame_len, parse_udp_frame,
};
use pico_10base_t::phy::{encode_frame, encoded_words};
use pico_10base_t::rx::{decode_frame, symbols_of};
use rp_pps::embassy::PpsOutput;
use rp_pps::{PpsSchedule, PpsScheduleConfig, output_high_cycles, output_period_cycles};
use tiny_ntp::client::{accept_broadcast, measure, request};
use tiny_ntp::discipline::{DisciplineConfig, NtpDiscipline};
use tiny_ntp::packet::{Mode, NtpPacket, PACKET_LEN};

use defmt_rtt as _;
use panic_probe as _;

bind_interrupts!(struct Irqs {
    PIO0_IRQ_0 => PioInterruptHandler<PIO0>;
    PIO1_IRQ_0 => PioInterruptHandler<PIO1>;
    // One DMA channel for the deserialiser, one for the serialiser.
    DMA_IRQ_0 => embassy_rp::dma::InterruptHandler<embassy_rp::peripherals::DMA_CH0>,
                 embassy_rp::dma::InterruptHandler<embassy_rp::peripherals::DMA_CH1>;
});

/// Words per capture. A 94-byte frame is 1638 symbols with its preamble and TP_IDL, and at two
/// samples each that is 205 words; the rest is idle and the decoder ignores it.
const CAPTURE_WORDS: usize = 256;

/// How long one capture covers. The state machine starts on the first bit of the frame and takes a
/// fixed number of samples, so the DMA finishes exactly this long after that bit arrived — which is
/// how a receive timestamp is recovered from a completion that happens well after the fact.
const CAPTURE_NS: i64 = (CAPTURE_WORDS as i64) * 16 * 25;

/// Everything between the frame's last bit and the timestamp taken for it (ns).
///
/// The capture ends, the DMA raises its interrupt, the executor wakes this task, and only then is
/// the clock read. All of that is time the frame has already spent arriving, and none of it is
/// visible from in here: the firmware's own numbers are consistent with a clock that is out by any
/// constant at all. It took the oscilloscope. With this at zero, the client's 1PPS sat 21.41 us
/// (sd 3.68, n=24) past the receiver's second while the server's own sat 55.02 us (sd 1.13) past
/// it - and the server's transmit lag was calibrated against that same second, so its packets are
/// right and what was left over was this.
///
/// The counterpart of `TX_LAG_NS` on the server, arrived at the same way and no more satisfying:
/// a constant measured once, on a rig, at a temperature. Timestamping the edge in PIO is what
/// replaces it.
const RX_LAG_NS: i64 = 21_400;

/// Room for the largest frame this link carries.
const MAX_FRAME: usize = 256;

/// The port the server announces on. It has a build-time override for the same reason the server
/// does — a high port can be listened to without root — and the two have to agree, so they are set
/// the same way.
///
/// ```sh
/// NTP_DST_PORT=10123 cargo build --release --bin ntp_client
/// ```
const NTP_PORT: u16 = match option_env!("NTP_DST_PORT") {
    Some(s) => parse_port(s),
    None => 123,
};

/// Decimal `&str` to `u16` at compile time, so a typo fails the build rather than shipping a
/// firmware that listens somewhere unintended.
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

/// Width of the 1PPS on GP6. The same 100 ms a GPS module emits, so the two traces on the scope are
/// the same shape and the rising edges are what differ.
const PPS_PULSE_NS: u32 = 100_000_000;

/// How early the word for the *next* edge is pushed, measured against the edge before it.
///
/// The deadline is a second earlier than it looks. The state machine pulls a period three cycles
/// before an edge, and what it pulls there is the length of the interval that *follows* — so the
/// word positioning edge n+1 has to be in the FIFO before edge n, not before edge n+1. Pushing on
/// the later deadline leaves the FIFO empty at every pull, and an empty `pull noblock` loads a spent
/// counter rather than holding the last period: one ~34-second interval and the output is gone.
const PUSH_LEAD_NS: i64 = 200_000_000;

/// Who we are on the wire. Locally administered, and not the server's.
const SRC_MAC: MacAddr = MacAddr([0x02, 0x00, 0x00, 0xC0, 0xFF, 0xEF]);
/// Our address, and the server's. Both on the link's own subnet; nothing routes between them.
const SRC_IP: Ipv4Addr = Ipv4Addr::new(192, 168, 0, 201);
const SERVER_IP: Ipv4Addr = Ipv4Addr::new(192, 168, 0, 200);
/// The server's MAC, as it appears in what it sends.
const SERVER_MAC: MacAddr = MacAddr([0x02, 0x00, 0x00, 0xC0, 0xFF, 0xEE]);

/// Port we ask from. A real client picks something ephemeral; a fixed one is easier to find in a
/// capture, and there is exactly one client on this link.
const SRC_PORT: u16 = 50123;

/// How often to ask. Once a second, to match what the beacon offered, so the two are comparable.
const POLL_INTERVAL_S: u64 = 1;
/// The poll exponent this interval corresponds to, as RFC 5905 counts it.
const POLL_LOG2: i8 = 0;

/// How far ahead a request's departure is set when the request is built (ns).
///
/// The counterpart of the server's `REPLY_LAG_NS`, and the same reasoning: the departure is written
/// into the packet before the packet is checksummed and encoded, so it is a claim about a moment
/// that has not happened, and the frame is held until the clock reaches it.
const REQ_LAG_NS: i64 = 1_000_000;

/// From handing the first symbol to the DMA to that symbol reaching the pin (ns).
///
/// The server measured this on its own transmit path with an oscilloscope; the client runs the same
/// state machine at the same clock, so it is the same number.
const WIRE_LAG_NS: i64 = 117_800;

const REQ_FRAME_LEN: usize = frame_len(PACKET_LEN);
const REQ_SYMBOL_WORDS: usize = encoded_words(REQ_FRAME_LEN);

/// The question currently outstanding, and when it left.
///
/// One at a time. The reply is matched on the transmit timestamp it echoes, so a second question
/// asked before the first is answered would make the first unmatchable - and a client with two
/// unanswered questions has no more information than one with one.
static OUTSTANDING: BlockingMutex<CriticalSectionRawMutex, RefCell<Option<NtpPacket>>> =
    BlockingMutex::new(RefCell::new(None));

/// The estimate. The link task writes it, the 1PPS task reads it.
static CLOCK: BlockingMutex<CriticalSectionRawMutex, RefCell<NtpDiscipline>> =
    BlockingMutex::new(RefCell::new(NtpDiscipline::new(DisciplineConfig::DEFAULT)));

/// `Instant` as nanoseconds. The resolution is a microsecond, which is the timer's, and at this
/// stage it is also the floor on everything this board can know about when a packet arrived.
fn now_ns() -> i64 {
    Instant::now().as_micros() as i64 * 1000
}

/// Signed distance from `utc_ns` to the nearest second boundary: positive when it is past one.
fn lateness_ns(utc_ns: i64) -> i64 {
    let into = utc_ns.rem_euclid(1_000_000_000);
    if into > 500_000_000 {
        into - 1_000_000_000
    } else {
        into
    }
}

/// Watch the 1PPS pin from the inside, and say what fraction of the time it is high.
///
/// A pin assigned to PIO is still readable through SIO, so this is the output as the chip sees it.
/// It answers a question an oscilloscope cannot: when a probe shows nothing, whether the pin is
/// idle or the probe is. A 100 ms pulse once a second should read about a tenth.
#[embassy_executor::task]
async fn pin_watch_task() {
    const SAMPLES: u32 = 1000;
    loop {
        let mut high = 0u32;
        for _ in 0..SAMPLES {
            if embassy_rp::pac::SIO.gpio_in(0).read() & (1 << 6) != 0 {
                high += 1;
            }
            Timer::after(Duration::from_micros(2000)).await;
        }
        // The pin as the chip sees it is only half the answer: a PIO pin whose input enable is
        // off reads zero however hard it is being driven. The routing is the other half, and it is
        // the half that failed silently - see the note in `main` about keeping `Common`.
        let funcsel = embassy_rp::pac::IO_BANK0.gpio(6).ctrl().read().funcsel();
        info!("GP6 high {}/1000, funcsel {}", high, funcsel);
    }
}

/// Count a capture that went nowhere, and say so from time to time.
///
/// Every second brings another, so reporting each one would bury everything else; the count is what
/// matters, and the reason is worth seeing occasionally.
fn note(dropped: &mut u32, why: &str, seen: u32) {
    *dropped = dropped.wrapping_add(1);
    if *dropped <= 3 || (*dropped).is_multiple_of(64) {
        warn!("capture {} dropped ({}), {} so far", seen, why, dropped);
    }
}

/// Read the link, and hand every NTP packet on it to the estimate.
#[embassy_executor::task]
async fn link_task(mut rx: Rx10BaseT<'static, PIO0, 0>) {
    let mut words = [0u32; CAPTURE_WORDS];
    let mut frame = [0u8; MAX_FRAME];
    let mut seen: u32 = 0;
    let mut used: u32 = 0;
    let mut dropped: u32 = 0;
    let mut broadcasts: u32 = 0;

    loop {
        rx.capture(&mut words).await;
        // The capture began on the frame's first bit and ran for a fixed span, so that bit is one
        // capture behind the completion - less what it took to get from the completion to here.
        // On a link two boards long the propagation from the far pad to this one is nanoseconds,
        // and the software between the two is not.
        let arrived_ns = now_ns() - CAPTURE_NS - RX_LAG_NS;
        seen = seen.wrapping_add(1);

        // How much of the capture the frame actually took. A 94-byte frame is 1638 symbols with
        // its preamble and TP_IDL, so at two samples each this has to come out at 3276. Anything
        // else means the sampling clock is not what `CAPTURE_NS` assumes, and `CAPTURE_NS` is how
        // the arrival time above is worked out.
        if seen % 64 == 1 {
            let nonidle: u32 = words
                .iter()
                .map(|w| symbols_of(*w).into_iter().filter(|s| *s != 0).count() as u32)
                .sum();
            info!("capture {} held {} non-idle samples", seen, nonidle);
        }

        let Some(len) = decode_frame(&words, &mut frame) else {
            note(&mut dropped, "no frame", seen);
            continue;
        };
        let Some(datagram) = parse_udp_frame(&frame[..len]) else {
            note(&mut dropped, "not a UDP datagram", seen);
            continue;
        };
        // The beacon goes to the service port; an answer comes back to the port the question left
        // from. Both are ours, and nothing else on this link is.
        if datagram.dst_port != NTP_PORT && datagram.dst_port != SRC_PORT {
            note(&mut dropped, "wrong port", seen);
            continue;
        }
        let Some(packet) = NtpPacket::decode(datagram.payload) else {
            warn!("port {} but not an NTP packet", datagram.dst_port);
            continue;
        };

        // The hint is only there to place the NTP era, which wraps in 2036; before the first
        // measurement there is nothing to hint with, and zero puts it in the era that covers now.
        let hint = CLOCK.lock(|c| c.borrow().utc_at(arrived_ns)).unwrap_or(0);
        // The exchange we asked for, if this is its answer. `measure` matches on the transmit
        // timestamp the server echoes, so a reply to a question we did not ask is refused here
        // rather than believed.
        let measurement = match packet.mode {
            Mode::Server => {
                // Read the question without consuming it. A mode-4 packet is not proof of being
                // *our* answer - `measure` decides that, by the transmit timestamp the server
                // echoes - and taking the question first means a duplicate or a stale reply
                // throws away the state the real one needs. The question is only spent once
                // something has matched it.
                let Some(sent) = OUTSTANDING.lock(|o| *o.borrow()) else {
                    note(&mut dropped, "a reply to nothing", seen);
                    continue;
                };
                match measure(&sent, &packet, hint) {
                    Ok(m) => {
                        OUTSTANDING.lock(|o| *o.borrow_mut() = None);
                        m
                    }
                    Err(reason) => {
                        warn!("reply refused: {}", defmt::Debug2Format(&reason));
                        continue;
                    }
                }
            }
            // The beacon still goes out, and it is still true; it just cannot say how long it took
            // to arrive. Counted, not used - the exchange is what this client runs on, unless it
            // was built the other way round to measure what the difference is worth.
            Mode::Broadcast => {
                broadcasts = broadcasts.wrapping_add(1);
                if !cfg!(feature = "broadcast-client") {
                    continue;
                }
                // Nothing measured the path, so nothing is subtracted for it. On two jumper wires
                // the one-way time is nanoseconds; on any longer link this is where a client would
                // have to be told what to assume, and being told is the weakness of the mode.
                match accept_broadcast(&packet, hint, 0) {
                    Ok(m) => m,
                    Err(reason) => {
                        warn!("beacon refused: {}", defmt::Debug2Format(&reason));
                        continue;
                    }
                }
            }
            _ => {
                note(&mut dropped, "not an answer", seen);
                continue;
            }
        };

        // `measure` worked against the hint we handed it as our own receive time; what the estimate
        // wants is the offset against local time, which is the same number shifted by the hint.
        let offset_ns = measurement.offset_ns + (hint - arrived_ns);
        let update = CLOCK.lock(|c| c.borrow_mut().observe(arrived_ns, offset_ns));
        used = used.wrapping_add(1);

        if update.stepped || used.is_multiple_of(16) {
            info!(
                "NTPRX seen={} used={} beacons={} stratum={} step={} delay_ns={} resid_ns={} offset_ns={} drift_ppb={}",
                seen,
                used,
                broadcasts,
                measurement.stratum,
                update.stepped,
                measurement.delay_ns,
                update.residual_ns,
                update.offset_ns,
                update.drift_ppb
            );
        }
    }
}

/// Ask the server the time, once a poll interval.
///
/// The transmit timestamp is the whole of the client's state for an exchange: the server echoes it
/// back untouched and [`measure`] matches on it, so nothing else has to be remembered.
#[embassy_executor::task]
async fn ask_task(mut tx: Tx10BaseT<'static, PIO0, 1>) {
    let mut frame = [0u8; REQ_FRAME_LEN];
    let mut symbols = [0u32; REQ_SYMBOL_WORDS];
    let mut ip_id: u16 = 0;
    let mut asked: u32 = 0;

    loop {
        Timer::after(Duration::from_secs(POLL_INTERVAL_S)).await;

        let started = now_ns();
        // Before there is a clock, the departure time is a number with no meaning - but it is still
        // the tag the reply is matched on, so a monotonic one does the job and the offset it
        // produces is discarded by the step on the first measurement.
        let departure = CLOCK
            .lock(|c| c.borrow().utc_at(now_ns()))
            .unwrap_or_else(now_ns)
            + REQ_LAG_NS;
        let packet = request(departure, POLL_LOG2);

        let payload = packet.encode();
        ip_id = ip_id.wrapping_add(1);
        let spec = UdpFrameSpec {
            src_mac: SRC_MAC,
            dst_mac: SERVER_MAC,
            src_ip: SRC_IP,
            dst_ip: SERVER_IP,
            src_port: SRC_PORT,
            dst_port: NTP_PORT,
            ip_id,
            ttl: 1,
            payload: &payload,
        };
        let Some(len) = build_udp_frame(&spec, &mut frame) else {
            warn!("request does not fit the frame buffer");
            continue;
        };
        let Some(words) = encode_frame(&frame[..len], &mut symbols) else {
            warn!("request does not fit the symbol buffer");
            continue;
        };

        // Hold the frame until the clock is one wire lag short of the departure written inside it.
        let handover_ns = now_ns() - started;
        let slack_ns = REQ_LAG_NS - WIRE_LAG_NS - handover_ns;
        if slack_ns > 0 {
            Timer::after(Duration::from_micros((slack_ns / 1000) as u64)).await;
        }
        OUTSTANDING.lock(|o| *o.borrow_mut() = Some(packet));
        tx.send(&symbols[..words]).await;
        asked = asked.wrapping_add(1);

        if asked <= 3 || asked.is_multiple_of(16) {
            info!(
                "NTPREQ n={} departure_ns={} handover_ns={} slack_ns={} bytes={}",
                asked, departure, handover_ns, slack_ns, len
            );
        }
        if slack_ns <= 0 {
            warn!("NTP request missed its own departure by {} ns", -slack_ns);
        }
    }
}

/// Keep GP6 on the second.
#[embassy_executor::task]
async fn pps_task(mut out: PpsOutput<'static, PIO1, 0>, mut schedule: PpsSchedule) {
    let mut edges: u32 = 0;

    loop {
        // Wake in time to have the next word in the FIFO before the state machine reaches for it,
        // which is one edge before the edge that word positions.
        let push_at = schedule.edge_ns() - PUSH_LEAD_NS;
        let now = now_ns();
        if push_at > now {
            Timer::after(Duration::from_micros(((push_at - now) / 1000) as u64)).await;
        }

        let predicted = schedule.predicted_edge_ns();
        let state = CLOCK.lock(|c| {
            let c = c.borrow();
            c.utc_at(predicted)
                .map(|utc| (lateness_ns(utc), c.drift_ppb() * 1000, c.locked()))
        });

        let step = match state {
            // Nothing has arrived yet: free-run at the nominal second so the FIFO stays fed.
            None => schedule.step(0, 0),
            // Something has, but not enough of it to steer by.
            Some((_, freq_mppb, false)) => schedule.step(freq_mppb, 0),
            Some((late, freq_mppb, true)) => schedule.advance(freq_mppb, late),
        };
        if step.acquired {
            info!("PPS placed, corr_ns={}", step.correction_ns);
        }

        if !out.set_period(step.period_word) {
            // The output program does not hold the last period on an empty pull, so a dropped push
            // is a dropped pulse rather than a glitch. Say so: it means this task ran late.
            warn!("PPS push dropped, FIFO full");
        }
        edges = edges.wrapping_add(1);

        if edges.is_multiple_of(16) {
            let late = CLOCK.lock(|c| c.borrow().utc_at(step.edge_ns).map(lateness_ns));
            info!(
                "PPS edges={} word={} corr_ns={} late_ns={}",
                edges,
                step.period_word,
                step.correction_ns,
                late.unwrap_or(0)
            );
        }
    }
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());
    let clk = clk_sys_freq();
    info!(
        "ntp_client: asks on GP16/GP17, listens on GP18/GP19, 1PPS on GP6, clk {} Hz",
        clk
    );

    // PIO0: the link, both directions. The serialiser is loaded first because it is pinned to
    // offset zero - `out pc` indexes it by symbol value - so the deserialiser has to land after it.
    let Pio {
        mut common,
        sm0,
        sm1,
        ..
    } = Pio::new(p.PIO0, Irqs);
    let tx_dma = embassy_rp::dma::Channel::new(p.DMA_CH1, Irqs);
    let tx = Tx10BaseT::new(
        &mut common,
        sm1,
        p.PIN_16, // TX−
        p.PIN_17, // TX+
        tx_dma,
        clk,
    );
    let dma = embassy_rp::dma::Channel::new(p.DMA_CH0, Irqs);
    let rx = Rx10BaseT::new(&mut common, sm0, p.PIN_18, p.PIN_19, dma, clk);

    // PIO1: the 1PPS. The enable is the schedule's one tie to local time, so it is timestamped as
    // close to the call as this can be written.
    let Pio {
        common: mut pps_common,
        sm0: pps_sm,
        ..
    } = Pio::new(p.PIO1, Irqs);
    let high_cycles = output_high_cycles(clk, PPS_PULSE_NS);
    let initial_period = output_period_cycles(clk, high_cycles);
    let out = PpsOutput::new(
        &mut pps_common,
        pps_sm,
        p.PIN_6,
        high_cycles,
        initial_period,
    );
    let enabled_ns = now_ns();
    let schedule = PpsSchedule::at_enable(
        clk,
        high_cycles,
        PpsScheduleConfig::default(),
        enabled_ns,
        initial_period,
    );

    // Neither `Common` may be dropped. embassy-rp releases a PIO block's pins - resets their
    // FUNCSEL to NULL - once the `Common` and the state machines it handed out are all gone, and
    // `main` returning is enough to start that. The state machines live on in their tasks and keep
    // running; the pin they were driving quietly stops being connected to them.
    //
    // It cost an afternoon. GP6 read funcsel 7 at the last line of `main` and 31 two seconds later,
    // with the state machine still enabled and still consuming a period every second, driving a pad
    // that was no longer listening. Nothing in the firmware's own telemetry could see it.
    core::mem::forget(common);
    core::mem::forget(pps_common);

    spawner.spawn(link_task(rx).unwrap());
    spawner.spawn(ask_task(tx).unwrap());
    spawner.spawn(pps_task(out, schedule).unwrap());
    spawner.spawn(pin_watch_task().unwrap());
}
