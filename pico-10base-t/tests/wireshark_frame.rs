//! Cross-check the framing against Wireshark's dissectors.
//!
//! Checksums are the part of framing where "it compiles and the bytes look plausible" is worth
//! nothing: an IPv4 or UDP checksum that is subtly wrong produces a frame every switch will forward
//! and every host will silently drop. So rather than assert our own arithmetic against itself, this
//! hands the frame to `tshark` with checksum validation switched on and asks whether *it* is happy —
//! including the Ethernet FCS, which nothing else in this crate can verify independently.
//!
//! The hex is generated from `build_udp_frame`, never typed by hand.
//!
//! Requires `wireshark-cli` (`tshark` + `text2pcap`). Skips with a warning when they are absent.

use std::process::Command;

use pico_10base_t::frame::{Ipv4Addr, MacAddr, UdpFrameSpec, build_udp_frame};

fn have(tool: &str) -> bool {
    Command::new(tool)
        .arg("--version")
        .output()
        .map(|o| o.status.success())
        .unwrap_or(false)
}

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

/// Dissect a raw Ethernet frame (FCS included) and return the requested fields.
fn dissect(frame: &[u8], fields: &[&str], tag: &str) -> Vec<String> {
    let dir = std::env::temp_dir().join(format!("pico-10base-t-{}-{tag}", std::process::id()));
    std::fs::create_dir_all(&dir).expect("create temp dir");
    let hex = dir.join("frame.hex");
    let pcap = dir.join("frame.pcap");

    std::fs::write(&hex, hexdump(frame)).expect("write hex");

    // No -u here (unlike the tiny-ntp test): these bytes are already a complete Ethernet frame,
    // so text2pcap's default Ethernet linktype is exactly right.
    let out = Command::new("text2pcap")
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
    cmd.arg("-r").arg(&pcap);
    // Checksum validation is off by default, and the Ethernet dissector only *guesses* whether a
    // trailing FCS is present. All of these have to be switched on, or the status fields come back
    // "unverified" and this test would pass while checking nothing.
    // (`eth.assume_fcs` is the old spelling and is rejected as obsolete by current Wireshark.)
    cmd.args(["-o", "eth.fcs:always"]);
    cmd.args(["-o", "eth.check_fcs:TRUE"]);
    cmd.args(["-o", "ip.check_checksum:TRUE"]);
    cmd.args(["-o", "udp.check_checksum:TRUE"]);
    cmd.args(["-T", "fields"]);
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

const SRC_MAC: MacAddr = MacAddr([0x02, 0x00, 0x00, 0xC0, 0xFF, 0xEE]);

fn broadcast_frame(payload: &[u8], out: &mut [u8]) -> usize {
    let spec = UdpFrameSpec {
        src_mac: SRC_MAC,
        dst_mac: MacAddr::BROADCAST,
        src_ip: Ipv4Addr::new(192, 168, 1, 200),
        dst_ip: Ipv4Addr::BROADCAST,
        src_port: 123,
        dst_port: 123,
        ip_id: 0x1234,
        ttl: 64,
        payload,
    };
    build_udp_frame(&spec, out).expect("buffer is big enough")
}

/// tshark reports checksum/FCS status as an enum; 1 is "Good".
const GOOD: &str = "1";

#[test]
fn wireshark_validates_every_checksum_in_a_broadcast_frame() {
    if !have("tshark") || !have("text2pcap") {
        eprintln!("SKIP: wireshark-cli (tshark/text2pcap) not installed");
        return;
    }

    let mut buf = [0u8; 128];
    let n = broadcast_frame(&[0xAB; 48], &mut buf);

    let got = dissect(
        &buf[..n],
        &[
            "eth.dst",
            "eth.src",
            "eth.type",
            "ip.proto",
            "ip.checksum.status",
            "udp.srcport",
            "udp.dstport",
            "udp.checksum.status",
            "eth.fcs.status",
        ],
        "broadcast",
    );

    assert_eq!(got[0], "ff:ff:ff:ff:ff:ff", "destination is broadcast");
    assert_eq!(got[1], "02:00:00:c0:ff:ee", "source MAC");
    assert_eq!(got[2], "0x0800", "EtherType IPv4");
    assert_eq!(got[3], "17", "protocol UDP");
    assert_eq!(got[4], GOOD, "IPv4 header checksum: {got:?}");
    assert_eq!(got[5], "123", "source port");
    assert_eq!(got[6], "123", "destination port");
    assert_eq!(got[7], GOOD, "UDP checksum: {got:?}");
    assert_eq!(got[8], GOOD, "Ethernet FCS: {got:?}");
}

#[test]
fn wireshark_validates_a_padded_minimum_length_frame() {
    if !have("tshark") || !have("text2pcap") {
        eprintln!("SKIP: wireshark-cli (tshark/text2pcap) not installed");
        return;
    }

    // One payload byte, so the frame is padded to the 64-byte floor. The padding must not be
    // counted by the IP total length nor by the UDP checksum — get either wrong and the status
    // fields below stop saying "Good".
    let mut buf = [0u8; 128];
    let n = broadcast_frame(&[0x5A], &mut buf);
    assert_eq!(n, 64, "padded to the Ethernet minimum");

    let got = dissect(
        &buf[..n],
        &[
            "ip.len",
            "ip.checksum.status",
            "udp.length",
            "udp.checksum.status",
            "eth.fcs.status",
        ],
        "padded",
    );

    assert_eq!(
        got[0], "29",
        "IP total length is 20 + 8 + 1, not the padded size"
    );
    assert_eq!(got[1], GOOD, "IPv4 header checksum: {got:?}");
    assert_eq!(got[2], "9", "UDP length is 8 + 1");
    assert_eq!(got[3], GOOD, "UDP checksum over the datagram only: {got:?}");
    assert_eq!(got[4], GOOD, "Ethernet FCS over the padded frame: {got:?}");
}

#[test]
fn wireshark_accepts_the_ntp_multicast_destination() {
    if !have("tshark") || !have("text2pcap") {
        eprintln!("SKIP: wireshark-cli (tshark/text2pcap) not installed");
        return;
    }

    let dst_ip = Ipv4Addr::NTP_MULTICAST;
    let spec = UdpFrameSpec {
        src_mac: SRC_MAC,
        dst_mac: MacAddr::for_ipv4_multicast(dst_ip),
        src_ip: Ipv4Addr::new(192, 168, 1, 200),
        dst_ip,
        src_port: 123,
        dst_port: 123,
        ip_id: 1,
        ttl: 1,
        payload: &[0u8; 48],
    };
    let mut buf = [0u8; 128];
    let n = build_udp_frame(&spec, &mut buf).unwrap();

    let got = dissect(
        &buf[..n],
        &["eth.dst", "ip.dst", "ip.checksum.status", "eth.fcs.status"],
        "multicast",
    );

    // RFC 1112 §6.4 mapping, which Wireshark also knows: it flags a mismatch as a malformed frame.
    assert_eq!(got[0], "01:00:5e:00:01:01");
    assert_eq!(got[1], "224.0.1.1");
    assert_eq!(got[2], GOOD, "IPv4 header checksum: {got:?}");
    assert_eq!(got[3], GOOD, "Ethernet FCS: {got:?}");
}
