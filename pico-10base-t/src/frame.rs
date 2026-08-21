//! Ethernet II / IPv4 / UDP framing and the Ethernet FCS.
//!
//! Pure integer logic — no HAL, no PIO, no allocation. The caller supplies the output buffer, so a
//! frame can be built straight into whatever the PHY will serialise from.
//!
//! Everything here is what a transmit-only station needs and nothing more: there is no ARP (we only
//! ever address broadcast or multicast, whose MACs are computable), and no IP fragmentation (an NTP
//! datagram is 76 bytes).

/// Ethernet II header: destination MAC, source MAC, EtherType.
pub const ETHERNET_HEADER_LEN: usize = 14;
/// IPv4 header with no options.
pub const IPV4_HEADER_LEN: usize = 20;
/// UDP header.
pub const UDP_HEADER_LEN: usize = 8;
/// Frame Check Sequence (CRC-32).
pub const FCS_LEN: usize = 4;
/// Minimum Ethernet payload; shorter ones are zero-padded so the frame reaches 64 bytes with FCS.
pub const MIN_ETHERNET_PAYLOAD: usize = 46;

const ETHERTYPE_IPV4: u16 = 0x0800;
const IP_PROTO_UDP: u8 = 17;

/// A 48-bit MAC address.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub struct MacAddr(pub [u8; 6]);

impl MacAddr {
    /// `ff:ff:ff:ff:ff:ff`.
    pub const BROADCAST: Self = Self([0xFF; 6]);

    /// The multicast MAC an IPv4 multicast group maps to (RFC 1112 §6.4): `01:00:5e` followed by
    /// the low 23 bits of the address.
    ///
    /// Note the lost bit — the group address has 28 significant bits but only 23 survive, so 32
    /// IPv4 groups share each MAC. That is the protocol's problem, not ours.
    pub const fn for_ipv4_multicast(ip: Ipv4Addr) -> Self {
        Self([0x01, 0x00, 0x5e, ip.0[1] & 0x7F, ip.0[2], ip.0[3]])
    }
}

/// An IPv4 address.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub struct Ipv4Addr(pub [u8; 4]);

impl Ipv4Addr {
    /// `255.255.255.255` — the limited broadcast address, never forwarded by a router.
    pub const BROADCAST: Self = Self([255; 4]);
    /// `224.0.1.1` — the IANA-assigned NTP multicast group.
    pub const NTP_MULTICAST: Self = Self([224, 0, 1, 1]);

    pub const fn new(a: u8, b: u8, c: u8, d: u8) -> Self {
        Self([a, b, c, d])
    }

    /// Whether this is in `224.0.0.0/4`.
    pub const fn is_multicast(self) -> bool {
        self.0[0] & 0xF0 == 0xE0
    }
}

/// Everything needed to build one UDP-over-IPv4-over-Ethernet frame.
#[derive(Clone, Copy, Debug)]
pub struct UdpFrameSpec<'a> {
    pub src_mac: MacAddr,
    pub dst_mac: MacAddr,
    pub src_ip: Ipv4Addr,
    pub dst_ip: Ipv4Addr,
    pub src_port: u16,
    pub dst_port: u16,
    /// IPv4 identification field. Only meaningful for fragmentation, which we never do; vary it
    /// anyway so captures are easier to follow.
    pub ip_id: u16,
    pub ttl: u8,
    pub payload: &'a [u8],
}

/// The Ethernet FCS: CRC-32 (reflected, polynomial `0xEDB88320`), as used by IEEE 802.3.
///
/// Table-driven, for 1 KB of flash — a decision made by the target, not the host.
///
/// **The state it was added to fix.** This was a bitwise loop, eight iterations per byte, and it
/// accounted for ~94% of the cost of building a frame. Together with the Manchester encoding that
/// put frame preparation at 97% of the time a frame occupies the wire on an RP2040, making the CPU
/// the bottleneck rather than the 10 Mbit/s link.
///
/// **What was done, and what it bought.** A 256-entry table, built in a `const` block so there is
/// no runtime setup to get wrong. Prepare time at MTU fell from 1194 µs to 527 µs, CPU cost from
/// 97% to 43% of wire time, and the effective transmit rate from 4.85 to 6.69 Mbit/s. See the
/// Manchester table in [`crate::phy`] for the full before/after — the two were changed together
/// because either alone leaves the other dominating.
///
/// **Why the host benchmark said not to bother.** It reported framing at 0.26% of wire time and no
/// measurable difference between the implementations — because at `-O3` LLVM recognises the bitwise
/// CRC idiom and rewrites it into a table, so the host was timing a table against a table. At `-O0`
/// the same comparison shows 4.3x. The host was wrong in direction, not merely in magnitude, and
/// following it would have left the transmit rate at half of what the wire allows.
pub fn crc32(data: &[u8]) -> u32 {
    let mut crc = 0xFFFF_FFFFu32;
    for &byte in data {
        crc = CRC32_TABLE[((crc ^ byte as u32) & 0xFF) as usize] ^ (crc >> 8);
    }
    !crc
}

/// Reflected CRC-32 table, generated at compile time (1 KB of flash, no runtime setup).
const CRC32_TABLE: [u32; 256] = {
    let mut table = [0u32; 256];
    let mut i = 0;
    while i < 256 {
        let mut c = i as u32;
        let mut bit = 0;
        while bit < 8 {
            // Reflected algorithm: shift right, and fold the polynomial in when a 1 falls out.
            c = if c & 1 != 0 {
                (c >> 1) ^ 0xEDB8_8320
            } else {
                c >> 1
            };
            bit += 1;
        }
        table[i] = c;
        i += 1;
    }
    table
};

/// The ones' complement checksum used by IPv4 and UDP (RFC 1071).
///
/// `initial` carries a pre-summed pseudo-header (UDP needs one); pass 0 for IPv4. The result is the
/// *complement* of the end-around-carry sum, so running this over a block that already contains a
/// correct checksum yields zero — which is how a receiver verifies one.
pub fn ones_complement_checksum(data: &[u8], initial: u32) -> u16 {
    let mut sum = initial;
    let mut chunks = data.chunks_exact(2);
    for c in &mut chunks {
        sum += u16::from_be_bytes([c[0], c[1]]) as u32;
    }
    // An odd trailing byte is the *high* half of a notional final word, padded on the right.
    if let [last] = chunks.remainder() {
        sum += u16::from_be_bytes([*last, 0]) as u32;
    }
    // Fold the carries back in until none are left.
    while sum >> 16 != 0 {
        sum = (sum & 0xFFFF) + (sum >> 16);
    }
    !(sum as u16)
}

/// Total frame length (including FCS, excluding preamble/SFD) for a given payload.
pub const fn frame_len(payload_len: usize) -> usize {
    let ip_total = IPV4_HEADER_LEN + UDP_HEADER_LEN + payload_len;
    let eth_payload = if ip_total < MIN_ETHERNET_PAYLOAD {
        MIN_ETHERNET_PAYLOAD
    } else {
        ip_total
    };
    ETHERNET_HEADER_LEN + eth_payload + FCS_LEN
}

/// Build the complete Ethernet frame — headers, payload, padding and FCS — into `out`.
///
/// Returns the number of bytes written, or `None` if `out` is too small. The preamble and SFD are
/// **not** included: those belong to the PHY, which generates them as line symbols.
pub fn build_udp_frame(spec: &UdpFrameSpec, out: &mut [u8]) -> Option<usize> {
    let udp_len = UDP_HEADER_LEN + spec.payload.len();
    let ip_total = IPV4_HEADER_LEN + udp_len;
    let total = frame_len(spec.payload.len());
    if out.len() < total || ip_total > u16::MAX as usize {
        return None;
    }
    let out = &mut out[..total];
    out.fill(0); // padding is zeros; writing everything else over it keeps the code linear

    // --- Ethernet II ---
    out[0..6].copy_from_slice(&spec.dst_mac.0);
    out[6..12].copy_from_slice(&spec.src_mac.0);
    out[12..14].copy_from_slice(&ETHERTYPE_IPV4.to_be_bytes());

    // --- IPv4 ---
    const IP: usize = ETHERNET_HEADER_LEN;
    out[IP] = 0x45; // version 4, IHL 5 (no options)
    out[IP + 1] = 0; // DSCP / ECN
    out[IP + 2..IP + 4].copy_from_slice(&(ip_total as u16).to_be_bytes());
    out[IP + 4..IP + 6].copy_from_slice(&spec.ip_id.to_be_bytes());
    out[IP + 6..IP + 8].copy_from_slice(&0u16.to_be_bytes()); // no flags, no fragment offset
    out[IP + 8] = spec.ttl;
    out[IP + 9] = IP_PROTO_UDP;
    out[IP + 10..IP + 12].copy_from_slice(&0u16.to_be_bytes()); // checksum, filled in below
    out[IP + 12..IP + 16].copy_from_slice(&spec.src_ip.0);
    out[IP + 16..IP + 20].copy_from_slice(&spec.dst_ip.0);
    let ip_ck = ones_complement_checksum(&out[IP..IP + IPV4_HEADER_LEN], 0);
    out[IP + 10..IP + 12].copy_from_slice(&ip_ck.to_be_bytes());

    // --- UDP ---
    const UDP: usize = ETHERNET_HEADER_LEN + IPV4_HEADER_LEN;
    out[UDP..UDP + 2].copy_from_slice(&spec.src_port.to_be_bytes());
    out[UDP + 2..UDP + 4].copy_from_slice(&spec.dst_port.to_be_bytes());
    out[UDP + 4..UDP + 6].copy_from_slice(&(udp_len as u16).to_be_bytes());
    out[UDP + 6..UDP + 8].copy_from_slice(&0u16.to_be_bytes()); // checksum, filled in below
    out[UDP + UDP_HEADER_LEN..UDP + udp_len].copy_from_slice(spec.payload);

    // The UDP checksum covers a pseudo-header of src, dst, zero, protocol and UDP length, then the
    // real header and payload — but *not* the Ethernet padding, which is not part of the datagram.
    let mut pseudo = 0u32;
    for c in spec
        .src_ip
        .0
        .chunks_exact(2)
        .chain(spec.dst_ip.0.chunks_exact(2))
    {
        pseudo += u16::from_be_bytes([c[0], c[1]]) as u32;
    }
    pseudo += IP_PROTO_UDP as u32 + udp_len as u32;
    let udp_ck = ones_complement_checksum(&out[UDP..UDP + udp_len], pseudo);
    // A computed zero is transmitted as 0xFFFF: in IPv4, zero means "no checksum here".
    let udp_ck = if udp_ck == 0 { 0xFFFF } else { udp_ck };
    out[UDP + 6..UDP + 8].copy_from_slice(&udp_ck.to_be_bytes());

    // --- FCS over everything above, little-endian on the wire ---
    let fcs = crc32(&out[..total - FCS_LEN]);
    out[total - FCS_LEN..].copy_from_slice(&fcs.to_le_bytes());

    Some(total)
}

/// One UDP datagram, as found inside a received frame.
///
/// The payload borrows the frame, so nothing is copied and the caller keeps the buffer.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct UdpDatagram<'a> {
    pub src_mac: MacAddr,
    pub dst_mac: MacAddr,
    pub src_ip: Ipv4Addr,
    pub dst_ip: Ipv4Addr,
    pub src_port: u16,
    pub dst_port: u16,
    pub payload: &'a [u8],
}

/// Find the UDP datagram in `frame`, which is a frame with its FCS already checked and removed.
///
/// The inverse of [`build_udp_frame`], and stricter than it needs to be on purpose: both header
/// checksums are verified, so a frame that survived its FCS but was built wrong is still refused.
///
/// The payload is bounded by the IP total length, not by the frame. A frame shorter than
/// [`MIN_ETHERNET_PAYLOAD`] is padded to reach it, and that padding is not payload.
pub fn parse_udp_frame(frame: &[u8]) -> Option<UdpDatagram<'_>> {
    const IP: usize = ETHERNET_HEADER_LEN;
    if frame.len() < IP + IPV4_HEADER_LEN + UDP_HEADER_LEN {
        return None;
    }
    if u16::from_be_bytes([frame[12], frame[13]]) != ETHERTYPE_IPV4 {
        return None;
    }

    // --- IPv4 ---
    if frame[IP] >> 4 != 4 {
        return None;
    }
    // Options are legal and this build never sends them, but a receiver that assumed their absence
    // would read the UDP header out of the middle of one.
    let ihl = (frame[IP] & 0x0F) as usize * 4;
    if ihl < IPV4_HEADER_LEN {
        return None;
    }
    let ip_total = u16::from_be_bytes([frame[IP + 2], frame[IP + 3]]) as usize;
    // Room for the UDP header, not merely for the IP one. `ihl` may be as much as 60 bytes, so a
    // packet that stops at the end of its own header passes `ip_total >= ihl` and still leaves
    // nothing to read — and the frame it came in may be no longer than the header either.
    if ip_total < ihl + UDP_HEADER_LEN || IP + ip_total > frame.len() {
        return None;
    }
    if frame[IP + 9] != IP_PROTO_UDP {
        return None;
    }
    // A one's-complement sum over a header that already carries its own checksum comes to zero.
    if ones_complement_checksum(&frame[IP..IP + ihl], 0) != 0 {
        return None;
    }
    // Fragments carry only a piece of the datagram, and the UDP header is in the first one alone.
    let frag = u16::from_be_bytes([frame[IP + 6], frame[IP + 7]]);
    if frag & 0x3FFF != 0 {
        return None;
    }

    // --- UDP ---
    let udp = IP + ihl;
    let udp_len = u16::from_be_bytes([frame[udp + 4], frame[udp + 5]]) as usize;
    if udp_len < UDP_HEADER_LEN || udp_len > ip_total - ihl {
        return None;
    }
    let datagram = &frame[udp..udp + udp_len];
    let carried = u16::from_be_bytes([frame[udp + 6], frame[udp + 7]]);
    // Zero means the sender declined to compute one, which IPv4 allows.
    if carried != 0 {
        let mut pseudo = 0u32;
        for c in frame[IP + 12..IP + 20].chunks_exact(2) {
            pseudo += u16::from_be_bytes([c[0], c[1]]) as u32;
        }
        pseudo += IP_PROTO_UDP as u32 + udp_len as u32;
        if ones_complement_checksum(datagram, pseudo) != 0 {
            return None;
        }
    }

    Some(UdpDatagram {
        src_mac: MacAddr(frame[6..12].try_into().ok()?),
        dst_mac: MacAddr(frame[0..6].try_into().ok()?),
        src_ip: Ipv4Addr(frame[IP + 12..IP + 16].try_into().ok()?),
        dst_ip: Ipv4Addr(frame[IP + 16..IP + 20].try_into().ok()?),
        src_port: u16::from_be_bytes([frame[udp], frame[udp + 1]]),
        dst_port: u16::from_be_bytes([frame[udp + 2], frame[udp + 3]]),
        payload: &datagram[UDP_HEADER_LEN..],
    })
}

#[cfg(test)]
mod tests {

    /// The spec `pico-ntp` actually sends, with a payload the caller chooses.
    fn ntp_spec(payload: &[u8]) -> UdpFrameSpec<'_> {
        UdpFrameSpec {
            src_mac: MacAddr([0x02, 0x00, 0x00, 0xC0, 0xFF, 0xEE]),
            dst_mac: MacAddr::BROADCAST,
            src_ip: Ipv4Addr::new(192, 168, 0, 200),
            dst_ip: Ipv4Addr::BROADCAST,
            src_port: 123,
            dst_port: 123,
            ip_id: 0x1234,
            ttl: 1,
            payload,
        }
    }

    /// Build a frame and hand back what a receiver would have: no FCS.
    fn received(spec: &UdpFrameSpec, out: &mut [u8]) -> usize {
        let len = build_udp_frame(spec, out).expect("frame fits");
        len - FCS_LEN
    }

    #[test]
    fn a_frame_parses_back_into_what_was_put_in_it() {
        let payload: [u8; 48] = core::array::from_fn(|i| i as u8);
        let spec = ntp_spec(&payload);
        let mut buf = [0u8; 128];
        let len = received(&spec, &mut buf);

        let got = parse_udp_frame(&buf[..len]).expect("parses");
        assert_eq!(got.src_mac, spec.src_mac);
        assert_eq!(got.dst_mac, spec.dst_mac);
        assert_eq!(got.src_ip, spec.src_ip);
        assert_eq!(got.dst_ip, spec.dst_ip);
        assert_eq!(got.src_port, spec.src_port);
        assert_eq!(got.dst_port, spec.dst_port);
        assert_eq!(got.payload, &payload[..]);
    }

    #[test]
    fn the_padding_on_a_short_frame_is_not_payload() {
        // Four bytes of payload, which is far under the 46-byte minimum: the rest is padding, and
        // a parser that trusted the frame length would hand back 18 bytes of zeros as well.
        let payload = [0xDE, 0xAD, 0xBE, 0xEF];
        let spec = ntp_spec(&payload);
        let mut buf = [0u8; 128];
        let len = received(&spec, &mut buf);
        assert!(
            len >= ETHERNET_HEADER_LEN + MIN_ETHERNET_PAYLOAD,
            "frame was padded"
        );

        let got = parse_udp_frame(&buf[..len]).expect("parses");
        assert_eq!(got.payload, &payload[..]);
    }

    #[test]
    fn a_frame_that_is_not_ipv4_is_refused() {
        let payload = [0u8; 48];
        let spec = ntp_spec(&payload);
        let mut buf = [0u8; 128];
        let len = received(&spec, &mut buf);
        buf[12] = 0x86;
        buf[13] = 0xDD;
        assert_eq!(parse_udp_frame(&buf[..len]), None);
    }

    #[test]
    fn a_frame_that_is_not_udp_is_refused() {
        let payload = [0u8; 48];
        let spec = ntp_spec(&payload);
        let mut buf = [0u8; 128];
        let len = received(&spec, &mut buf);
        // Protocol byte, and the IP checksum recomputed so only the protocol is wrong.
        let ip = ETHERNET_HEADER_LEN;
        buf[ip + 9] = 6;
        buf[ip + 10] = 0;
        buf[ip + 11] = 0;
        let sum = ones_complement_checksum(&buf[ip..ip + IPV4_HEADER_LEN], 0);
        buf[ip + 10..ip + 12].copy_from_slice(&sum.to_be_bytes());
        assert_eq!(parse_udp_frame(&buf[..len]), None);
    }

    #[test]
    fn a_frame_with_a_broken_ip_checksum_is_refused() {
        let payload = [0u8; 48];
        let spec = ntp_spec(&payload);
        let mut buf = [0u8; 128];
        let len = received(&spec, &mut buf);
        buf[ETHERNET_HEADER_LEN + 10] ^= 0xFF;
        assert_eq!(parse_udp_frame(&buf[..len]), None);
    }

    #[test]
    fn a_frame_with_a_broken_udp_checksum_is_refused() {
        let payload = [0u8; 48];
        let spec = ntp_spec(&payload);
        let mut buf = [0u8; 128];
        let len = received(&spec, &mut buf);
        let udp = ETHERNET_HEADER_LEN + IPV4_HEADER_LEN;
        buf[udp + 6] ^= 0xFF;
        assert_eq!(parse_udp_frame(&buf[..len]), None);
    }

    #[test]
    fn a_frame_cut_short_is_refused() {
        let payload = [0u8; 48];
        let spec = ntp_spec(&payload);
        let mut buf = [0u8; 128];
        let len = received(&spec, &mut buf);
        for cut in [
            0,
            1,
            ETHERNET_HEADER_LEN,
            ETHERNET_HEADER_LEN + IPV4_HEADER_LEN,
            len - 1,
        ] {
            assert_eq!(parse_udp_frame(&buf[..cut]), None, "cut at {cut}");
        }
    }
    use super::*;

    const NTP_PAYLOAD: [u8; 48] = [0xAB; 48];

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

    // --- CRC-32 / checksum primitives ---

    #[test]
    fn crc32_matches_the_standard_check_value() {
        // The CRC-32/ISO-HDLC check value, i.e. what IEEE 802.3 computes over "123456789".
        // Confirmed against an independent implementation rather than from memory:
        //   $ uv run --no-project python -c "import zlib; print(hex(zlib.crc32(b'123456789')))"
        //   0xcbf43926
        assert_eq!(crc32(b"123456789"), 0xCBF4_3926);
    }

    #[test]
    fn crc32_of_nothing_is_zero() {
        assert_eq!(crc32(b""), 0);
    }

    #[test]
    fn ones_complement_checksum_of_a_known_header() {
        // RFC 1071's own worked example. The end-around-carry *sum* of these bytes is 0xddf2; the
        // checksum is its ones' complement, 0x220d. Returning the complement is what makes a
        // correct header verify to zero.
        let data = [0x00, 0x01, 0xf2, 0x03, 0xf4, 0xf5, 0xf6, 0xf7];
        assert_eq!(ones_complement_checksum(&data, 0), 0x220d);
    }

    #[test]
    fn ones_complement_checksum_handles_an_odd_length() {
        // The final odd byte is padded on the right, not the left.
        assert_eq!(
            ones_complement_checksum(&[0x12], 0),
            ones_complement_checksum(&[0x12, 0x00], 0)
        );
    }

    // --- Address derivation ---

    #[test]
    fn broadcast_mac_is_all_ones() {
        assert_eq!(MacAddr::BROADCAST.0, [0xFF; 6]);
    }

    #[test]
    fn ntp_multicast_maps_to_the_expected_mac() {
        // 224.0.1.1 -> 01:00:5e:00:01:01 (RFC 1112 §6.4).
        assert_eq!(
            MacAddr::for_ipv4_multicast(Ipv4Addr::NTP_MULTICAST),
            MacAddr([0x01, 0x00, 0x5e, 0x00, 0x01, 0x01])
        );
    }

    #[test]
    fn multicast_mapping_drops_the_25th_bit() {
        // 224.128.1.1 and 224.0.1.1 differ above the low 23 bits, so they share a MAC. Encoding
        // that in a test because it looks like a bug when first observed in a capture.
        assert_eq!(
            MacAddr::for_ipv4_multicast(Ipv4Addr::new(224, 128, 1, 1)),
            MacAddr::for_ipv4_multicast(Ipv4Addr::new(224, 0, 1, 1))
        );
    }

    #[test]
    fn multicast_detection_covers_the_whole_class_d_range() {
        assert!(Ipv4Addr::new(224, 0, 0, 0).is_multicast());
        assert!(Ipv4Addr::new(239, 255, 255, 255).is_multicast());
        assert!(!Ipv4Addr::new(223, 255, 255, 255).is_multicast());
        assert!(!Ipv4Addr::BROADCAST.is_multicast());
    }

    // --- Frame layout ---

    #[test]
    fn frame_length_for_an_ntp_payload() {
        // 14 Ethernet + 20 IPv4 + 8 UDP + 48 NTP + 4 FCS.
        assert_eq!(frame_len(48), 94);
    }

    #[test]
    fn short_payloads_pad_to_the_ethernet_minimum() {
        // 20 + 8 + 1 = 29 bytes of IP, below the 46-byte floor, so the frame is 14 + 46 + 4.
        assert_eq!(frame_len(1), 64);
    }

    #[test]
    fn build_writes_exactly_frame_len_bytes() {
        let mut out = [0u8; 128];
        let n = build_udp_frame(&spec(&NTP_PAYLOAD), &mut out).expect("buffer is big enough");
        assert_eq!(n, frame_len(NTP_PAYLOAD.len()));
    }

    #[test]
    fn build_refuses_a_buffer_that_is_too_small() {
        let mut out = [0u8; 93];
        assert_eq!(build_udp_frame(&spec(&NTP_PAYLOAD), &mut out), None);
    }

    #[test]
    fn ethernet_header_carries_the_macs_and_the_ipv4_ethertype() {
        let mut out = [0u8; 128];
        let s = spec(&NTP_PAYLOAD);
        build_udp_frame(&s, &mut out).unwrap();
        assert_eq!(&out[0..6], &s.dst_mac.0);
        assert_eq!(&out[6..12], &s.src_mac.0);
        assert_eq!(&out[12..14], &ETHERTYPE_IPV4.to_be_bytes());
    }

    #[test]
    fn ipv4_header_is_a_20_byte_udp_datagram() {
        let mut out = [0u8; 128];
        let s = spec(&NTP_PAYLOAD);
        build_udp_frame(&s, &mut out).unwrap();
        let ip = &out[14..34];
        assert_eq!(ip[0], 0x45, "version 4, IHL 5 (no options)");
        assert_eq!(
            u16::from_be_bytes([ip[2], ip[3]]),
            (IPV4_HEADER_LEN + UDP_HEADER_LEN + 48) as u16,
            "total length counts IP header onward, never the Ethernet header"
        );
        assert_eq!(u16::from_be_bytes([ip[4], ip[5]]), 0x1234, "id");
        assert_eq!(ip[8], 64, "ttl");
        assert_eq!(ip[9], IP_PROTO_UDP);
        assert_eq!(&ip[12..16], &s.src_ip.0);
        assert_eq!(&ip[16..20], &s.dst_ip.0);
    }

    #[test]
    fn ipv4_header_checksum_verifies() {
        // A receiver checks by summing the header *including* the checksum field and expecting the
        // ones' complement to be zero.
        let mut out = [0u8; 128];
        build_udp_frame(&spec(&NTP_PAYLOAD), &mut out).unwrap();
        assert_eq!(ones_complement_checksum(&out[14..34], 0), 0);
    }

    #[test]
    fn udp_header_carries_the_ports_and_its_own_length() {
        let mut out = [0u8; 128];
        build_udp_frame(&spec(&NTP_PAYLOAD), &mut out).unwrap();
        let udp = &out[34..42];
        assert_eq!(u16::from_be_bytes([udp[0], udp[1]]), 123);
        assert_eq!(u16::from_be_bytes([udp[2], udp[3]]), 123);
        assert_eq!(
            u16::from_be_bytes([udp[4], udp[5]]),
            (UDP_HEADER_LEN + 48) as u16,
            "UDP length includes its own header"
        );
    }

    #[test]
    fn udp_checksum_is_present_and_verifies_over_the_pseudo_header() {
        let mut out = [0u8; 128];
        let s = spec(&NTP_PAYLOAD);
        build_udp_frame(&s, &mut out).unwrap();
        let udp = &out[34..42 + 48];
        assert_ne!(
            u16::from_be_bytes([udp[6], udp[7]]),
            0,
            "zero means 'no checksum' in IPv4 UDP; we always compute one"
        );
        // Pseudo-header: src, dst, zero, protocol, UDP length.
        let udp_len = (UDP_HEADER_LEN + 48) as u32;
        let mut sum = 0u32;
        for chunk in s.src_ip.0.chunks(2).chain(s.dst_ip.0.chunks(2)) {
            sum += u16::from_be_bytes([chunk[0], chunk[1]]) as u32;
        }
        sum += IP_PROTO_UDP as u32 + udp_len;
        assert_eq!(ones_complement_checksum(udp, sum), 0);
    }

    #[test]
    fn payload_lands_after_the_headers() {
        let mut out = [0u8; 128];
        build_udp_frame(&spec(&NTP_PAYLOAD), &mut out).unwrap();
        assert_eq!(&out[42..90], &NTP_PAYLOAD);
    }

    #[test]
    fn padding_is_zero_and_not_counted_by_the_ip_header() {
        let mut out = [0u8; 128];
        let s = spec(&[0x5A]);
        let n = build_udp_frame(&s, &mut out).unwrap();
        assert_eq!(n, 64);
        assert_eq!(out[42], 0x5A, "the one payload byte");
        assert!(
            out[43..60].iter().all(|&b| b == 0),
            "padding to the 46-byte floor must be zeros"
        );
        let ip = &out[14..34];
        assert_eq!(
            u16::from_be_bytes([ip[2], ip[3]]),
            (IPV4_HEADER_LEN + UDP_HEADER_LEN + 1) as u16,
            "IP total length describes the datagram, not the padded Ethernet payload"
        );
    }

    /// A header long enough to satisfy every length check, and a total length that leaves nothing
    /// after it. `ip_total >= ihl` holds and `IP + ip_total` is inside the frame, so the parser
    /// used to reach the UDP header and index past the end of what it was given.
    #[test]
    fn an_ipv4_packet_with_no_room_for_a_udp_header_is_refused() {
        const IHL_WORDS: u8 = 15; // 60 bytes: the largest an IPv4 header may be
        const IHL: usize = IHL_WORDS as usize * 4;
        let mut frame = [0u8; ETHERNET_HEADER_LEN + IHL];
        frame[12..14].copy_from_slice(&ETHERTYPE_IPV4.to_be_bytes());
        let ip = ETHERNET_HEADER_LEN;
        frame[ip] = 0x40 | IHL_WORDS;
        // Total length stops at the end of the header: a legal encoding of an empty payload.
        frame[ip + 2..ip + 4].copy_from_slice(&(IHL as u16).to_be_bytes());
        frame[ip + 9] = IP_PROTO_UDP;
        let checksum = ones_complement_checksum(&frame[ip..ip + IHL], 0);
        frame[ip + 10..ip + 12].copy_from_slice(&checksum.to_be_bytes());

        assert!(parse_udp_frame(&frame).is_none());
    }

    #[test]
    fn fcs_is_the_crc_of_everything_before_it_little_endian() {
        let mut out = [0u8; 128];
        let n = build_udp_frame(&spec(&NTP_PAYLOAD), &mut out).unwrap();
        let expected = crc32(&out[..n - FCS_LEN]);
        assert_eq!(&out[n - FCS_LEN..n], &expected.to_le_bytes());
    }
}
