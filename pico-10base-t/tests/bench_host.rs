//! Host-side cost measurement for the transmit path, swept across frame sizes.
//!
//! **Not** a substitute for an on-target benchmark — an x86 host has a barrel shifter, a data cache
//! and a branch predictor the Cortex-M0+ does not. Treat the absolute numbers as an upper bound on
//! how good things can look, and the *ratios* as the transferable part.
//!
//! Run manually (`#[ignore]`d so it never slows normal test runs):
//!
//! ```text
//! cargo test -p pico-10base-t --test bench_host --release -- --ignored --nocapture
//! ```
//!
//! # What the numbers mean
//!
//! 10BASE-T puts one bit on the wire every 100 ns — that rate is fixed and nothing here can raise
//! it. Two separate ceilings decide what a caller actually gets:
//!
//! - **Protocol limit.** Every frame carries a 7+1 byte preamble/SFD, 46 bytes of headers and FCS,
//!   and is followed by a 96-bit-time (12 byte) interframe gap. Those are pure overhead, so UDP
//!   goodput tops out well under 10 Mbit/s and collapses for small payloads.
//! - **CPU limit.** Whether the RP2040 can *prepare* frames faster than the wire *drains* them. If
//!   it cannot, the link idles waiting and the protocol limit is unreachable.
//!
//! The `CPU%` column is preparation time as a percentage of wire time; under 100% means line rate
//! is reachable given double buffering, and the second table says which ceiling binds.

use std::time::Instant;

use pico_10base_t::frame::{Ipv4Addr, MacAddr, UdpFrameSpec, build_udp_frame, crc32, frame_len};
use pico_10base_t::phy::{encode_frame, encoded_words, manchester_word};

/// Preamble + SFD, which the PHY emits ahead of every frame.
const PREAMBLE_LEN: usize = 8;
/// Interframe gap: 96 bit times, i.e. 12 bytes of silence between frames.
const IFG_LEN: usize = 12;
/// 10BASE-T: 10 Mbit/s, so 100 ns per bit.
const NS_PER_BIT: f64 = 100.0;

/// A 256-entry table CRC-32, built at compile time — the alternative `crc32` deliberately is not.
const CRC_TABLE: [u32; 256] = {
    let mut table = [0u32; 256];
    let mut i = 0;
    while i < 256 {
        let mut c = i as u32;
        let mut k = 0;
        while k < 8 {
            c = if c & 1 != 0 {
                0xEDB8_8320 ^ (c >> 1)
            } else {
                c >> 1
            };
            k += 1;
        }
        table[i] = c;
        i += 1;
    }
    table
};

fn crc32_table(data: &[u8]) -> u32 {
    let mut crc = 0xFFFF_FFFFu32;
    for &b in data {
        crc = CRC_TABLE[((crc ^ b as u32) & 0xFF) as usize] ^ (crc >> 8);
    }
    !crc
}

/// The Manchester equivalent of upstream's `tbl_manchester`, for the same comparison.
const MANCHESTER_TABLE: [u32; 256] = {
    let mut table = [0u32; 256];
    let mut i = 0;
    while i < 256 {
        table[i] = manchester_word(i as u8);
        i += 1;
    }
    table
};

fn encode_frame_table(frame: &[u8], out: &mut [u32]) -> usize {
    let mut i = 0;
    for b in pico_10base_t::phy::PREAMBLE_SFD {
        out[i] = MANCHESTER_TABLE[b as usize];
        i += 1;
    }
    for &b in frame {
        out[i] = MANCHESTER_TABLE[b as usize];
        i += 1;
    }
    out[i] = pico_10base_t::phy::TP_IDL_WORD;
    i + 1
}

fn spec(payload: &[u8]) -> UdpFrameSpec<'_> {
    UdpFrameSpec {
        src_mac: MacAddr([0x02, 0x00, 0x00, 0xC0, 0xFF, 0xEE]),
        dst_mac: MacAddr::BROADCAST,
        src_ip: Ipv4Addr::new(192, 168, 1, 200),
        dst_ip: Ipv4Addr::BROADCAST,
        src_port: 123,
        dst_port: 123,
        ip_id: 0x1234,
        ttl: 64,
        payload,
    }
}

fn time_it(iters: u32, mut f: impl FnMut()) -> f64 {
    f(); // one untimed pass so a cold miss does not land in the measurement
    let t0 = Instant::now();
    for _ in 0..iters {
        f();
    }
    t0.elapsed().as_secs_f64() / iters as f64
}

/// Wire occupancy of a frame of `frame_bytes` (FCS included, preamble excluded), in seconds.
fn wire_time(frame_bytes: usize) -> f64 {
    (frame_bytes + PREAMBLE_LEN) as f64 * 8.0 * NS_PER_BIT * 1e-9
}

#[test]
fn the_two_crc32_implementations_agree() {
    // A benchmark comparing implementations is worthless if they compute different things.
    for len in [0usize, 9, 90, 1500] {
        let data = vec![0x5A; len];
        assert_eq!(crc32(&data), crc32_table(&data), "len={len}");
    }
    assert_eq!(crc32(b"123456789"), crc32_table(b"123456789"));
}

#[test]
fn the_two_manchester_encoders_agree() {
    let frame = vec![0xA5u8; 94];
    let mut a = vec![0u32; 256];
    let mut b = vec![0u32; 256];
    let n = encode_frame(&frame, &mut a).unwrap();
    assert_eq!(n, encode_frame_table(&frame, &mut b));
    assert_eq!(a[..n], b[..n]);
}

#[test]
#[ignore = "benchmark; run with --ignored --nocapture"]
fn transmit_path_cost_and_achievable_rate() {
    // UDP payload sizes: the smallest that still pads, an NTP packet, and up to the largest that
    // fits a 1500-byte IP MTU (1500 - 20 IPv4 - 8 UDP).
    const PAYLOADS: [usize; 6] = [1, 48, 256, 512, 1024, 1472];
    let mut out = vec![0u8; 2048];
    let mut sym = vec![0u32; 2048];

    println!();
    println!("prepare cost vs wire time (CPU% under 100 means line rate is reachable)");
    println!(
        "{:>8} {:>6} {:>10} {:>10} {:>9} {:>7} {:>8}",
        "payload", "frame", "build", "encode", "wire", "CPU%", "symRAM"
    );
    for &n in &PAYLOADS {
        let payload = vec![0xA5u8; n];
        let frame = frame_len(n);
        let iters = if n > 512 { 20_000 } else { 200_000 };
        let build = time_it(iters, || {
            std::hint::black_box(build_udp_frame(
                std::hint::black_box(&spec(&payload)),
                &mut out,
            ));
        });
        let encode = time_it(iters, || {
            std::hint::black_box(encode_frame(std::hint::black_box(&out[..frame]), &mut sym));
        });
        let wire = wire_time(frame);
        let symbol_ram = encoded_words(frame) * 4;
        println!(
            "{n:>8} {frame:>6} {:>7.0} ns {:>7.0} ns {:>6.1} us {:>6.2}% {symbol_ram:>6} B",
            build * 1e9,
            encode * 1e9,
            wire * 1e6,
            (build + encode) / wire * 100.0
        );
    }

    println!();
    println!("achievable UDP goodput — which ceiling binds?");
    println!(
        "{:>8} {:>9} {:>12} {:>12} {:>11}",
        "payload", "on-wire B", "proto Mbit/s", "CPU Mbit/s", "bottleneck"
    );
    for &n in &PAYLOADS {
        let payload = vec![0xA5u8; n];
        let iters = if n > 512 { 20_000 } else { 200_000 };
        let cpu = time_it(iters, || {
            let len = build_udp_frame(std::hint::black_box(&spec(&payload)), &mut out).unwrap();
            std::hint::black_box(encode_frame(&out[..len], &mut sym));
        });
        // Every frame also costs a preamble and the interframe gap.
        let on_wire = frame_len(n) + PREAMBLE_LEN + IFG_LEN;
        let proto = n as f64 * 8.0 / (on_wire as f64 * 8.0 * NS_PER_BIT * 1e-9) / 1e6;
        let cpu_mbps = n as f64 * 8.0 / cpu / 1e6;
        let bottleneck = if cpu_mbps < proto { "CPU" } else { "wire" };
        println!("{n:>8} {on_wire:>9} {proto:>11.2} {cpu_mbps:>11.0} {bottleneck:>11}");
    }

    // --- Would a lookup table help? ---
    println!();
    let frame_mtu = frame_len(1472);
    build_udp_frame(&spec(&vec![0xA5u8; 1472]), &mut out).unwrap();
    let enc_computed = time_it(20_000, || {
        std::hint::black_box(encode_frame(
            std::hint::black_box(&out[..frame_mtu]),
            &mut sym,
        ));
    });
    let enc_table = time_it(20_000, || {
        std::hint::black_box(encode_frame_table(
            std::hint::black_box(&out[..frame_mtu]),
            &mut sym,
        ));
    });
    let big = vec![0x5Au8; 1500];
    let crc_bitwise = time_it(20_000, || {
        std::hint::black_box(crc32(std::hint::black_box(&big)));
    });
    let crc_tab = time_it(20_000, || {
        std::hint::black_box(crc32_table(std::hint::black_box(&big)));
    });
    println!("lookup tables, at MTU (1 KB of flash each):");
    println!(
        "  manchester  computed {:>7.0} ns   table {:>7.0} ns   {:.2}x",
        enc_computed * 1e9,
        enc_table * 1e9,
        enc_computed / enc_table
    );
    println!(
        "  crc32       bitwise  {:>7.0} ns   table {:>7.0} ns   {:.2}x",
        crc_bitwise * 1e9,
        crc_tab * 1e9,
        crc_bitwise / crc_tab
    );
    println!(
        "  (a crc32 ratio near 1.0 is not an error: at -O3 LLVM recognises the bitwise CRC\n   \
         idiom and rewrites it. Re-run without --release to see the ~4.3x algorithmic ratio.)"
    );
}
