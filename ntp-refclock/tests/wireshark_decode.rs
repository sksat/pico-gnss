//! Cross-check our wire encoding against an **independent** NTP implementation: Wireshark's
//! dissector.
//!
//! Unit tests can only prove that the encoder agrees with the encoder's own author. This feeds the
//! bytes `NtpPacket::encode` produces to `text2pcap` (which wraps them in dummy Ethernet/IPv4/UDP
//! headers) and then to `tshark`, and asserts that a third party reads back the fields we meant to
//! write. It catches exactly the class of bug a hand-written golden vector cannot: a wrong epoch
//! offset, a byte-order slip, or a signed field decoded as unsigned.
//!
//! The hex is generated from `encode()` — never typed by hand. A hand-typed vector only tests the
//! typist.
//!
//! Requires `wireshark-cli` (`tshark` + `text2pcap`). The test **skips with a warning** when they
//! are absent rather than failing, so a checkout without them still builds green.

use std::path::PathBuf;
use std::process::Command;

use ntp_refclock::packet::{LeapIndicator, Mode, NtpPacket};
use ntp_refclock::timestamp::{NtpShort, NtpTimestamp};

fn have(tool: &str) -> bool {
    Command::new(tool)
        .arg("--version")
        .output()
        .map(|o| o.status.success())
        .unwrap_or(false)
}

/// Format the 48 bytes the way `text2pcap` wants: an offset column, then hex octets.
fn hexdump(bytes: &[u8]) -> String {
    let mut s = String::new();
    for (i, chunk) in bytes.chunks(16).enumerate() {
        s.push_str(&format!("{:06x} ", i * 16));
        for b in chunk {
            s.push_str(&format!(" {b:02x}"));
        }
        s.push('\n');
    }
    s
}

/// Run the packet through text2pcap + tshark and return the requested field values, in order.
fn dissect(packet: &NtpPacket, fields: &[&str], tag: &str) -> Vec<String> {
    let dir = std::env::temp_dir().join(format!("ntp-refclock-{}-{tag}", std::process::id()));
    std::fs::create_dir_all(&dir).expect("create temp dir");
    let hex: PathBuf = dir.join("ntp.hex");
    let pcap: PathBuf = dir.join("ntp.pcap");

    std::fs::write(&hex, hexdump(&packet.encode())).expect("write hex");

    // -u 123,123 synthesises the UDP/IPv4/Ethernet headers around our payload, so this test stays
    // independent of the framing crate (which lives in `pico-10base-t`, not here).
    let out = Command::new("text2pcap")
        .args(["-u", "123,123"])
        .arg(&hex)
        .arg(&pcap)
        .output()
        .expect("run text2pcap");
    assert!(
        out.status.success(),
        "text2pcap failed: {}",
        String::from_utf8_lossy(&out.stderr)
    );

    let mut cmd = Command::new("tshark");
    cmd.arg("-r").arg(&pcap).args(["-T", "fields"]);
    for f in fields {
        cmd.args(["-e", f]);
    }
    let out = cmd.output().expect("run tshark");
    assert!(
        out.status.success(),
        "tshark failed: {}",
        String::from_utf8_lossy(&out.stderr)
    );

    let stdout = String::from_utf8(out.stdout).expect("tshark output is utf-8");
    let line = stdout.lines().next().unwrap_or_default().to_string();
    let _ = std::fs::remove_dir_all(&dir);
    line.split('\t').map(|s| s.to_string()).collect()
}

/// The Stratum-1 GPS broadcast packet this crate exists to produce.
fn stratum1_broadcast(transmit_unix_ns: i64, reference_unix_ns: i64) -> NtpPacket {
    NtpPacket {
        leap: LeapIndicator::NoWarning,
        version: 4,
        mode: Mode::Broadcast,
        stratum: 1,
        poll: 4,
        precision: -26,
        root_delay: NtpShort::ZERO,
        root_dispersion: NtpShort::from_nanos(1_000_000),
        reference_id: *b"GPS\0",
        reference_timestamp: NtpTimestamp::from_unix_ns(reference_unix_ns),
        origin_timestamp: NtpTimestamp::ZERO,
        receive_timestamp: NtpTimestamp::ZERO,
        transmit_timestamp: NtpTimestamp::from_unix_ns(transmit_unix_ns),
    }
}

#[test]
fn wireshark_reads_back_every_field_we_wrote() {
    if !have("tshark") || !have("text2pcap") {
        eprintln!("SKIP: wireshark-cli (tshark/text2pcap) not installed");
        return;
    }

    // 2026-08-18T02:42:47Z, a UTC second taken from a real GNSS fix on this hardware.
    const REF: i64 = 1_787_020_967 * 1_000_000_000;
    let packet = stratum1_broadcast(REF + 500_000_000, REF);

    let got = dissect(
        &packet,
        &[
            "ntp.flags.li",
            "ntp.flags.vn",
            "ntp.flags.mode",
            "ntp.stratum",
            "ntp.ppoll",
            "ntp.precision",
            "ntp.rootdelay",
            "ntp.refid",
            "ntp.reftime",
            "ntp.xmt",
        ],
        "fields",
    );

    assert_eq!(got[0], "0", "leap indicator");
    assert_eq!(got[1], "4", "version");
    assert_eq!(got[2], "5", "mode = broadcast");
    assert_eq!(got[3], "1", "stratum = primary reference");
    assert_eq!(got[4], "4", "poll");
    // The one that catches a u8/i8 slip: -26 read as unsigned would come back as 230.
    assert_eq!(got[5], "-26", "precision stays signed");
    assert_eq!(got[6], "0", "root delay");
    assert_eq!(got[7], "47505300", "refid is ASCII \"GPS\\0\"");
    // The one that catches a wrong prime-epoch offset or byte order.
    assert_eq!(
        got[8], "2026-08-18T02:42:47.000000000Z",
        "reference timestamp"
    );
    assert_eq!(
        got[9], "2026-08-18T02:42:47.500000000Z",
        "transmit timestamp"
    );
}

#[test]
fn wireshark_agrees_on_a_post_2036_timestamp() {
    if !have("tshark") || !have("text2pcap") {
        eprintln!("SKIP: wireshark-cli (tshark/text2pcap) not installed");
        return;
    }

    // 2040-01-01T00:00:00Z is in NTP era 1: the 32-bit seconds field has wrapped. The era is not on
    // the wire, so Wireshark resolves it with the RFC's own convention — an independent check that
    // our era arithmetic puts the right 32 bits out.
    const UNIX_2040: i64 = 2_208_988_800 * 1_000_000_000;
    assert_eq!(
        NtpTimestamp::era_of_unix_ns(UNIX_2040),
        1,
        "2040 must be era 1, or this test is not testing the wrap"
    );

    let packet = stratum1_broadcast(UNIX_2040, UNIX_2040);
    let got = dissect(&packet, &["ntp.xmt"], "era1");
    assert_eq!(got[0], "2040-01-01T00:00:00.000000000Z");
}
