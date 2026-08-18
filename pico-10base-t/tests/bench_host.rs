//! Host-side cost measurement for the framing path, swept across frame sizes.
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
//! 10BASE-T puts one bit on the wire every 100 ns, so a frame of `N` bytes (plus the 8-byte
//! preamble/SFD) occupies the wire for `(N + 8) * 800 ns`. The question that decides whether this
//! crate can stream back-to-back frames is whether the CPU can *prepare* a frame faster than the
//! wire *drains* one. The `wire%` column below is exactly that: CPU time as a percentage of wire
//! time. Under 100% means line rate is reachable with double buffering; over 100% means the CPU is
//! the bottleneck and the link idles.
//!
//! This measures framing only (headers, checksums, FCS). The Manchester symbol encoding is a
//! separate and larger cost; it belongs in this sweep once it exists.

use std::time::Instant;

use pico_10base_t::frame::{
    FCS_LEN, IPV4_HEADER_LEN, Ipv4Addr, MacAddr, UDP_HEADER_LEN, UdpFrameSpec, build_udp_frame,
    crc32, frame_len,
};

/// Preamble + SFD, which the PHY emits ahead of every frame and which therefore counts against the
/// wire-time budget even though `build_udp_frame` never sees it.
const PREAMBLE_LEN: usize = 8;
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
#[ignore = "benchmark; run with --ignored --nocapture"]
fn framing_cost_across_frame_sizes() {
    // UDP payload sizes: the smallest that still pads, an NTP packet, and up to the largest that
    // fits a 1500-byte IP MTU (1500 - 20 IPv4 - 8 UDP).
    const PAYLOADS: [usize; 6] = [1, 48, 256, 512, 1024, 1472];
    let mut out = vec![0u8; 2048];

    println!();
    println!(
        "{:>8} {:>7} {:>11} {:>11} {:>8} {:>9}",
        "payload", "frame", "build", "wire", "wire%", "symbolRAM"
    );
    for &n in &PAYLOADS {
        let payload = vec![0xA5u8; n];
        let frame = frame_len(n);
        let iters = if n > 512 { 20_000 } else { 200_000 };
        let t = time_it(iters, || {
            std::hint::black_box(build_udp_frame(
                std::hint::black_box(&spec(&payload)),
                &mut out,
            ));
        });
        let wire = wire_time(frame);
        // Each data bit becomes two line states, each a 2-bit symbol: 4 bits of buffer per bit.
        let symbol_ram = (frame + PREAMBLE_LEN) * 4;
        println!(
            "{n:>8} {frame:>7} {:>9.0} ns {:>9.1} us {:>7.2}% {symbol_ram:>7} B",
            t * 1e9,
            wire * 1e6,
            t / wire * 100.0
        );
    }

    println!();
    let big = vec![0x5Au8; 1500];
    let bitwise = time_it(20_000, || {
        std::hint::black_box(crc32(std::hint::black_box(&big)));
    });
    let table = time_it(20_000, || {
        std::hint::black_box(crc32_table(std::hint::black_box(&big)));
    });
    println!("crc32 over 1500 B — bitwise {:>7.0} ns", bitwise * 1e9);
    println!("crc32 over 1500 B — table   {:>7.0} ns", table * 1e9);
    println!(
        "table/bitwise speedup: {:.1}x for 1 KB of flash",
        bitwise / table
    );
    println!(
        "  (a speedup near 1.0x here is not a measurement error: at -O3 LLVM recognises the\n   \
         bitwise CRC idiom and rewrites it, erasing the difference. Re-run without --release\n   \
         to see the algorithmic ratio, which is ~4.3x.)"
    );
    println!(
        "crc32 is {:.0}% of build_udp_frame's work at MTU",
        bitwise
            / time_it(20_000, || {
                std::hint::black_box(build_udp_frame(
                    std::hint::black_box(&spec(&vec![0xA5u8; 1472])),
                    &mut out,
                ));
            })
            * 100.0
    );

    println!();
    println!(
        "note: an MTU frame is {} B of headers+FCS over {} B of payload",
        IPV4_HEADER_LEN + UDP_HEADER_LEN + FCS_LEN + 14,
        1472
    );
}
