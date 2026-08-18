//! **Hardware end-to-end**: a real third-party NTP client synchronising to the time this project's
//! RP2040 put on a 10BASE-T wire.
//!
//! Requires the hardware running and reachable. `#[ignore]`d, per this repo's convention for tests
//! that need a bench:
//!
//! ```text
//! # terminal 1 — with DST_PORT set to the measurement port
//! cd pico-ntp && cargo run --release
//! # terminal 2
//! cargo test -p tiny-ntp --test hardware_e2e -- --ignored --nocapture
//! ```
//!
//! # Why it is shaped like this
//!
//! The goal is a *real* NTP client accepting our time — not our own listener agreeing with itself.
//! Two obstacles:
//!
//! - No maintained Rust NTP client implements **broadcast** client mode, and neither do chrony or
//!   systemd-timesyncd; only the reference `ntpd` does. So the client has to be spoken to in
//!   unicast.
//! - The present wiring is **transmit-only**, so the hardware cannot answer a unicast request.
//!
//! So this receives the hardware's actual broadcast, takes the disciplined UTC out of it, and hands
//! that same time to [`rsntp`] — a third-party RFC 5905 implementation — through a unicast exchange
//! on a port of our choosing. What is being tested is that a real client validates and accepts
//! timestamps that originated in the GPSDO and travelled over the 10BASE-T link: the packet layout,
//! the epoch, the stratum and reference identifier, and the ~ms-level agreement with a
//! NTP-synchronised host clock.
//!
//! What it does **not** test is the last hop's timing: the relay adds its own scheduling, so the
//! offset here is an upper bound on the error, not a measurement of the link. `scripts/
//! ntp_broadcast_listen.py` measures that directly.

use std::net::UdpSocket;
use std::thread;
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

use tiny_ntp::packet::{Mode, NtpPacket};
use tiny_ntp::server::{ClockState, LeapWarning, ServeDecision, ServerConfig, Source, respond};

/// Where the firmware broadcasts while `DST_PORT` is set to the measurement port.
const HARDWARE_PORT: u16 = 10123;
/// How long to wait for the hardware to say something.
const RECEIVE_TIMEOUT: Duration = Duration::from_secs(20);

fn unix_ns_now() -> i64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .expect("system clock is after 1970")
        .as_nanos() as i64
}

/// One broadcast from the hardware: the UTC it claimed, and the local monotonic instant it arrived,
/// so the time can be carried forward without consulting the host's wall clock again.
struct HardwareTime {
    unix_ns: i64,
    received_at: Instant,
    stratum: u8,
    reference_id: [u8; 4],
    root_dispersion_ns: u64,
}

fn receive_one_broadcast() -> Option<HardwareTime> {
    let sock = UdpSocket::bind(("0.0.0.0", HARDWARE_PORT)).expect("bind the measurement port");
    sock.set_read_timeout(Some(RECEIVE_TIMEOUT)).unwrap();
    let mut buf = [0u8; 128];
    let (n, from) = sock.recv_from(&mut buf).ok()?;
    let received_at = Instant::now();
    let packet = NtpPacket::decode(&buf[..n])?;
    // Anything else on this port is not what we are here for.
    if packet.mode != Mode::Broadcast {
        return None;
    }
    println!(
        "received a broadcast from {from}: stratum {}, refid {:?}, transmit {}",
        packet.stratum,
        core::str::from_utf8(&packet.reference_id).unwrap_or("??"),
        packet.transmit_timestamp.to_unix_ns_near(unix_ns_now()),
    );
    Some(HardwareTime {
        unix_ns: packet.transmit_timestamp.to_unix_ns_near(unix_ns_now()),
        received_at,
        stratum: packet.stratum,
        reference_id: packet.reference_id,
        root_dispersion_ns: packet.root_dispersion.to_nanos(),
    })
}

/// Serve unicast NTP from the hardware's clock, carried forward on the local monotonic clock.
/// Returns the port to point a client at.
fn relay_hardware_time(hw: HardwareTime) -> u16 {
    let sock = UdpSocket::bind("127.0.0.1:0").expect("bind loopback");
    let port = sock.local_addr().unwrap().port();
    let cfg = ServerConfig {
        precision: -20,
        poll: 4,
        source: Source::ReferenceClock {
            id: hw.reference_id,
        },
        base_dispersion_ns: hw.root_dispersion_ns,
        holdover_drift_ppb: 1_000,
        max_holdover_ns: 3_600 * 1_000_000_000,
    };

    thread::spawn(move || {
        // The hardware's UTC, advanced by however long ago it arrived. Deliberately monotonic:
        // consulting the host's wall clock here would smuggle the host's time into the answer and
        // make the whole test circular.
        let now = |hw: &HardwareTime| hw.unix_ns + hw.received_at.elapsed().as_nanos() as i64;

        let mut buf = [0u8; 128];
        let (n, peer) = sock.recv_from(&mut buf).expect("receive a client request");
        let receive_unix_ns = now(&hw);
        let request = NtpPacket::decode(&buf[..n]).expect("a client request parses");
        let state = ClockState {
            last_update_unix_ns: Some(receive_unix_ns - 500_000_000),
            holdover_ns: 500_000_000,
            frequency_locked: true,
            leap: LeapWarning::None,
        };
        match respond(&cfg, &state, &request, receive_unix_ns, now(&hw)) {
            ServeDecision::Serve(reply) => {
                sock.send_to(&reply.encode(), peer).expect("send the reply");
            }
            ServeDecision::Silent(reason) => panic!("refused to serve: {reason:?}"),
        }
    });

    port
}

#[test]
#[ignore = "needs the hardware transmitting on the measurement port"]
fn a_real_ntp_client_synchronises_to_the_hardware() {
    let Some(hw) = receive_one_broadcast() else {
        panic!(
            "no NTP broadcast on UDP :{HARDWARE_PORT} within {RECEIVE_TIMEOUT:?} — is the firmware \
             running, and is DST_PORT set to {HARDWARE_PORT}?"
        );
    };

    assert_eq!(
        hw.stratum, 1,
        "the hardware should claim a primary reference"
    );
    let hardware_utc = hw.unix_ns;
    let host_utc_at_receipt = unix_ns_now();

    let port = relay_hardware_time(hw);

    let mut client = rsntp::SntpClient::new();
    client.set_timeout(Duration::from_secs(5));
    let result = client
        .synchronize(format!("127.0.0.1:{port}"))
        .expect("a third-party RFC 5905 client accepts the hardware's time");

    // What the client concluded, from time that came off the wire.
    let offset_s = result.clock_offset().as_secs_f64();
    println!(
        "rsntp synchronised: stratum {}, offset {:+.1} ms from this host's clock",
        result.stratum(),
        offset_s * 1000.0
    );

    assert_eq!(result.stratum(), 1, "client sees a primary reference");
    assert_eq!(
        result.reference_identifier().to_string(),
        "GPS",
        "client sees a GPS-disciplined source"
    );
    assert_eq!(
        result.leap_indicator(),
        rsntp::LeapIndicator::NoWarning,
        "a leap indicator of 3 would mean the client must not use this source"
    );

    // The hardware and this host both claim to know UTC; they should agree to well within a second.
    // A whole second apart is the PPS-NMEA pairing failure this project has already been bitten by,
    // so the bound is deliberately tight enough to catch it.
    let disagreement_s = (hardware_utc - host_utc_at_receipt).abs() as f64 / 1e9;
    assert!(
        disagreement_s < 0.5,
        "hardware UTC and host UTC disagree by {disagreement_s:.3} s — a value near 1.0 means the \
         PPS/NMEA second pairing is off by one"
    );
    assert!(
        offset_s.abs() < 0.5,
        "client computed a {offset_s:.3} s offset, which is too large to be path delay"
    );
}
