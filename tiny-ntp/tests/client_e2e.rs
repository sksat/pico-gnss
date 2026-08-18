//! End-to-end against an **independent NTP client implementation**.
//!
//! The Wireshark test proves a dissector can read our bytes. This proves something stronger and
//! more useful: that a real RFC 5905 client, written by someone else, will *accept our replies and
//! set its clock from them*. It runs the actual server policy over a real loopback UDP socket and
//! asks [`rsntp`] what time it thinks it is.
//!
//! What this can and cannot cover:
//!
//! - **Covered.** Packet layout, epoch conversion, the origin/receive/transmit timestamp protocol,
//!   and the fields a client checks before it will trust a source at all (stratum, leap indicator,
//!   mode, root dispersion).
//! - **Not covered.** Everything below the datagram — Ethernet framing, Manchester coding, the
//!   PHY — and broadcast mode, which no maintained Rust client implements. Those need the hardware
//!   and a `ntpd broadcastclient`.
//!
//! Note this exercises *unicast* (mode 3 → 4), which the current transmit-only wiring cannot do on
//! the wire. It is the same policy and the same encoder either way, so testing it here is the
//! cheapest place to catch a mistake in both.

use std::net::UdpSocket;
use std::thread;
use std::time::{Duration, SystemTime, UNIX_EPOCH};

use tiny_ntp::packet::{Mode, NtpPacket};
use tiny_ntp::server::{ClockState, ServeDecision, ServerConfig, respond};

fn unix_ns_now() -> i64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .expect("system clock is after 1970")
        .as_nanos() as i64
}

fn config() -> ServerConfig {
    ServerConfig {
        precision: -20,
        poll: 4,
        reference_id: *b"GPS\0",
        base_dispersion_ns: 1_000_000,
        holdover_drift_ppb: 1_000,
        max_holdover_ns: 3_600 * 1_000_000_000,
    }
}

/// Serve exactly one request, pretending our disciplined clock reads `offset_ns` away from the
/// host's. Returns the port to point a client at.
///
/// The offset is the trick that makes this a real test: if the client comes back reporting that
/// offset, it decoded our timestamps correctly — whereas serving the true time would also "pass"
/// for an encoder that accidentally echoed the client's own clock back at it.
fn serve_once(offset_ns: i64) -> u16 {
    let sock = UdpSocket::bind("127.0.0.1:0").expect("bind loopback");
    let port = sock.local_addr().expect("local addr").port();

    thread::spawn(move || {
        let mut buf = [0u8; 128];
        let (n, peer) = sock.recv_from(&mut buf).expect("receive a request");
        let receive_unix_ns = unix_ns_now() + offset_ns;

        let request = NtpPacket::decode(&buf[..n]).expect("a client request parses");
        assert_eq!(request.mode, Mode::Client, "rsntp should send mode 3");

        // A plausible freshly-disciplined clock: locked, half a second since the last PPS edge.
        let state = ClockState {
            last_update_unix_ns: Some(receive_unix_ns - 500_000_000),
            holdover_ns: 500_000_000,
            frequency_locked: true,
        };

        let transmit_unix_ns = unix_ns_now() + offset_ns;
        match respond(
            &config(),
            &state,
            &request,
            receive_unix_ns,
            transmit_unix_ns,
        ) {
            ServeDecision::Serve(reply) => {
                sock.send_to(&reply.encode(), peer).expect("send the reply");
            }
            ServeDecision::Silent(reason) => panic!("refused to serve: {reason:?}"),
        }
    });

    port
}

fn synchronize(port: u16) -> rsntp::SynchronizationResult {
    let mut client = rsntp::SntpClient::new();
    client.set_timeout(Duration::from_secs(5));
    client
        .synchronize(format!("127.0.0.1:{port}"))
        .expect("an independent RFC 5905 client accepts our reply")
}

#[test]
fn an_independent_client_reads_the_time_we_served() {
    // An hour ahead of the host clock — far outside any plausible measurement noise, so the
    // assertion cannot pass by accident.
    const OFFSET_NS: i64 = 3_600 * 1_000_000_000;
    let port = serve_once(OFFSET_NS);

    let result = synchronize(port);
    let offset = result.clock_offset().as_secs_f64();

    assert!(
        (offset - 3600.0).abs() < 1.0,
        "client should see us an hour ahead, saw {offset} s"
    );
}

#[test]
fn an_independent_client_agrees_with_an_unshifted_clock() {
    // Serving the true time should produce an offset indistinguishable from zero. This is the case
    // that would catch a constant epoch error — an hour of deliberate offset would still "work"
    // with a 70-year epoch mistake, but this would not.
    let port = serve_once(0);

    let result = synchronize(port);
    let offset = result.clock_offset().as_secs_f64();

    assert!(
        offset.abs() < 0.1,
        "loopback offset should be milliseconds, saw {offset} s"
    );
}

#[test]
fn the_client_sees_us_as_a_stratum_1_gps_reference() {
    // The fields a client uses to decide whether a source is worth listening to. A client that
    // computed a sane offset but read stratum 0 would discard us anyway.
    let port = serve_once(0);

    let result = synchronize(port);

    assert_eq!(result.stratum(), 1, "primary reference");
    assert_eq!(
        result.reference_identifier().to_string(),
        "GPS",
        "source code"
    );
    assert_eq!(
        result.leap_indicator(),
        rsntp::LeapIndicator::NoWarning,
        "a leap indicator of 3 would mean 'do not use me'"
    );
}
