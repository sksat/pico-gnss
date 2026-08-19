//! Carries a real NTP client's unicast exchange to `pico-ntp` over the debug probe.
//!
//! The 10BASE-T wiring transmits and nothing else, so the mode 3 → mode 4 exchange cannot happen
//! over the wire. `pico-ntp`'s `swd-rx` feature puts a mailbox in RAM; this binds a UDP socket on
//! the host, and for each request writes it into that mailbox, waits for the firmware to answer,
//! and sends the reply back. To the client it is an ordinary NTP server.
//!
//! ```text
//!   NTP client ──UDP──> this ──SWD──> pico-ntp (disciplined UTC) ──> reply ──> client
//! ```
//!
//! **Debug only.** The request travels over USB and the probe, so the round trip the client
//! measures describes that path rather than Ethernet. What this is for is the other question: does
//! a real implementation accept our packets, and where does its loop settle.
//!
//! ```sh
//! # once, to flash and then release the probe
//! cd pico-ntp && cargo run --release --features swd-rx   # then Ctrl-C
//! # then
//! cargo run -p swd-ntp-bridge --release -- target/thumbv6m-none-eabi/release/pico-ntp 10123
//! ```

use std::net::UdpSocket;
use std::time::{Duration, Instant};

use anyhow::{Context, Result, bail};
use probe_rs::probe::list::Lister;
use probe_rs::{MemoryInterface, Permissions, Session};

/// Layout of `pico_ntp::swd_rx::Mailbox`, by offset from the symbol. `#[repr(C)]` on that side is
/// what makes these fixed.
mod mailbox {
    pub const DOORBELL: u64 = 0;
    pub const DONE: u64 = 4;
    pub const RECV_NS: u64 = 8;
    pub const XMIT_NS: u64 = 16;
    pub const SILENT: u64 = 24;
    pub const REQUEST: u64 = 28;
    pub const REPLY: u64 = 28 + 48;
}

const PACKET_LEN: usize = 48;
/// How long to wait for the firmware to answer one request. Its poll interval is 2 ms.
const ANSWER_TIMEOUT: Duration = Duration::from_millis(500);

fn main() -> Result<()> {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let elf = args
        .first()
        .context("usage: swd-ntp-bridge <pico-ntp.elf> [udp-port | --duty <gpio> <seconds>]")?
        .clone();

    // `--duty` samples a pin instead of serving. Which edge of the receiver's 1PPS marks the second
    // decides whether the captured timestamp is the boundary or a pulse width past it, and the duty
    // cycle is what says which edge is the short one. An oscilloscope answers this directly; absent
    // one, the probe can still count.
    if args.get(1).map(String::as_str) == Some("--duty") {
        let pin: u32 = args.get(2).context("--duty needs a GPIO number")?.parse()?;
        let seconds: f64 = args.get(3).map_or(Ok(5.0), |s| s.parse())?;
        return duty(pin, seconds);
    }

    let port: u16 = args.get(1).map_or(Ok(10123), |s| s.parse())?;

    let base = mailbox_address(&elf)?;
    println!("mailbox at {base:#010x} (from {elf})");

    let lister = Lister::new();
    let probes = lister.list_all();
    let probe = probes
        .first()
        .context("no debug probe found")?
        .open()
        .context("could not open the probe")?;
    // Attach without resetting: the firmware is already running and its clock is already
    // disciplined. A reset here would throw away the very state being measured.
    let mut session = probe
        .attach("RP2040", Permissions::default())
        .context("could not attach — is another probe-rs holding it?")?;

    let socket = UdpSocket::bind(("0.0.0.0", port))?;
    println!("listening on udp/{port}; point an NTP client at this host");

    let mut seq: u32 = read_u32(&mut session, base + mailbox::DOORBELL)?;
    let mut buf = [0u8; 1024];
    loop {
        let (len, peer) = socket.recv_from(&mut buf)?;
        if len < PACKET_LEN {
            eprintln!("{peer}: {len} bytes is not an NTP packet, ignoring");
            continue;
        }

        let started = Instant::now();
        match exchange(&mut session, base, &mut seq, &buf[..PACKET_LEN]) {
            Ok(Some(answer)) => {
                socket.send_to(&answer.reply, peer)?;
                // One line per exchange, so the client's view and the instrument's overhead can be
                // told apart afterwards. The firmware's own T2/T3 bracket the work this server did;
                // everything else inside `probe_us` is the probe, and belongs to the measurement
                // rather than to the server being measured.
                println!(
                    "{},{},{},{},{}",
                    seq,
                    answer.recv_unix_ns,
                    answer.xmit_unix_ns,
                    answer.xmit_unix_ns - answer.recv_unix_ns,
                    started.elapsed().as_micros(),
                );
            }
            Ok(None) => eprintln!("{peer}: firmware stayed silent"),
            Err(e) => eprintln!("{peer}: {e:#}"),
        }
    }
}

/// What one mailbox round trip produced.
struct Answer {
    reply: [u8; PACKET_LEN],
    /// The firmware's own receive and transmit timestamps (T2, T3), by its disciplined clock.
    recv_unix_ns: i64,
    xmit_unix_ns: i64,
}

/// One request through the mailbox. `Ok(None)` means the firmware declined to answer.
fn exchange(
    session: &mut Session,
    base: u64,
    seq: &mut u32,
    request: &[u8],
) -> Result<Option<Answer>> {
    let mut core = session.core(0)?;
    core.write_8(base + mailbox::REQUEST, request)?;
    // The doorbell last, so the request it announces is already in place.
    *seq = seq.wrapping_add(1);
    core.write_word_32(base + mailbox::DOORBELL, *seq)?;

    let deadline = Instant::now() + ANSWER_TIMEOUT;
    while core.read_word_32(base + mailbox::DONE)? != *seq {
        if Instant::now() > deadline {
            bail!("timed out waiting for the firmware — is it built with --features swd-rx?");
        }
        std::thread::sleep(Duration::from_micros(500));
    }

    if core.read_word_32(base + mailbox::SILENT)? != 0 {
        return Ok(None);
    }
    let mut reply = [0u8; PACKET_LEN];
    core.read_8(base + mailbox::REPLY, &mut reply)?;
    Ok(Some(Answer {
        reply,
        recv_unix_ns: core.read_word_64(base + mailbox::RECV_NS)? as i64,
        xmit_unix_ns: core.read_word_64(base + mailbox::XMIT_NS)? as i64,
    }))
}

fn read_u32(session: &mut Session, address: u64) -> Result<u32> {
    Ok(session.core(0)?.read_word_32(address)?)
}

/// RP2040 SIO `GPIO_IN` — the raw state of every pin, readable while the core runs.
const SIO_GPIO_IN: u64 = 0xd000_0004;

/// Sample one pin as fast as the probe allows and report how much of the time it is high.
///
/// A 1PPS is one short pulse a second, and the short part is the one that marks the boundary. So
/// the fraction says which edge is the mark, without needing an instrument on the wire: 10% high
/// means a 100 ms pulse and a rising-edge mark, 90% high means the pulse is the *low* period and
/// the falling edge is the mark.
///
/// Also reports the longest run of consecutive same-value samples, which puts a number on the pulse
/// itself rather than only on the ratio.
fn duty(pin: u32, seconds: f64) -> Result<()> {
    let lister = Lister::new();
    let probes = lister.list_all();
    let probe = probes
        .first()
        .context("no debug probe found")?
        .open()
        .context("could not open the probe")?;
    let mut session = probe.attach("RP2040", Permissions::default())?;
    let mut core = session.core(0)?;

    let mask = 1u32 << pin;
    let started = Instant::now();
    let (mut high, mut total) = (0u64, 0u64);
    // Longest observed run of each level, in samples, and where the level last changed.
    let (mut run_high, mut run_low) = (0u64, 0u64);
    let (mut run, mut last) = (0u64, None::<bool>);

    while started.elapsed().as_secs_f64() < seconds {
        let is_high = core.read_word_32(SIO_GPIO_IN)? & mask != 0;
        total += 1;
        if is_high {
            high += 1;
        }
        match last {
            Some(prev) if prev == is_high => run += 1,
            Some(prev) => {
                if prev {
                    run_high = run_high.max(run);
                } else {
                    run_low = run_low.max(run);
                }
                run = 1;
            }
            None => run = 1,
        }
        last = Some(is_high);
    }

    let elapsed = started.elapsed().as_secs_f64();
    let per_sample_ms = elapsed / total as f64 * 1000.0;
    println!(
        "GP{pin}: {:.1}% high over {:.1} s ({} samples, {:.3} ms/sample)",
        high as f64 / total as f64 * 100.0,
        elapsed,
        total,
        per_sample_ms,
    );
    println!(
        "  longest run  high {:.1} ms   low {:.1} ms",
        run_high as f64 * per_sample_ms,
        run_low as f64 * per_sample_ms,
    );
    Ok(())
}

/// Find `NTP_SWD_MAILBOX` in the firmware's symbol table.
///
/// Read from the ELF rather than hard-coded: the address moves whenever the firmware's statics do,
/// and a stale constant would write into whatever now lives there.
fn mailbox_address(elf: &str) -> Result<u64> {
    let output = std::process::Command::new("nm")
        .arg(elf)
        .output()
        .context("could not run `nm`")?;
    if !output.status.success() {
        bail!("nm failed on {elf}");
    }
    let text = String::from_utf8_lossy(&output.stdout);
    for line in text.lines() {
        let mut parts = line.split_whitespace();
        let (Some(addr), Some(_kind), Some(name)) = (parts.next(), parts.next(), parts.next())
        else {
            continue;
        };
        if name == "NTP_SWD_MAILBOX" {
            return Ok(u64::from_str_radix(addr, 16)?);
        }
    }
    bail!("no NTP_SWD_MAILBOX in {elf} — build pico-ntp with --features swd-rx")
}
