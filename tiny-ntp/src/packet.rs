//! The 48-byte NTP packet header (RFC 5905 §7.3) and its wire encoding.

use crate::timestamp::{NtpShort, NtpTimestamp};

/// Length of the NTP header on the wire. Extension fields, if any, follow it; this crate does not
/// use them.
pub const PACKET_LEN: usize = 48;

/// Leap Indicator (RFC 5905 §7.3), the top 2 bits of byte 0.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum LeapIndicator {
    /// No leap second pending.
    NoWarning = 0,
    /// The last minute of the day has 61 seconds.
    LastMinute61 = 1,
    /// The last minute of the day has 59 seconds.
    LastMinute59 = 2,
    /// Clock not synchronised — clients must not use this source.
    Unsynchronized = 3,
}

/// Association mode (RFC 5905 §7.3), the low 3 bits of byte 0.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Mode {
    Reserved = 0,
    SymmetricActive = 1,
    SymmetricPassive = 2,
    /// A client asking a server for the time.
    Client = 3,
    /// A server answering a client.
    Server = 4,
    /// One-way broadcast/multicast from a server. What a transmit-only PHY can do.
    Broadcast = 5,
    ControlMessage = 6,
    Private = 7,
}

/// A decoded NTP header.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub struct NtpPacket {
    pub leap: LeapIndicator,
    /// Protocol version, 3 bits. 4 is current.
    pub version: u8,
    pub mode: Mode,
    /// 1 = a primary (reference-clock) server. 0 and 16..=255 mean unsynchronised/reserved.
    pub stratum: u8,
    /// log2 of the poll interval in seconds. For broadcast, the broadcast interval.
    pub poll: i8,
    /// log2 of the clock's *timestamping* resolution in seconds. Signed, and negative in practice.
    pub precision: i8,
    /// Round-trip delay to the reference source. Zero for a primary server.
    pub root_delay: NtpShort,
    /// Maximum error relative to the reference source. Grows during holdover.
    pub root_dispersion: NtpShort,
    /// Stratum 1: a four-character ASCII source code, e.g. `GPS\0`.
    pub reference_id: [u8; 4],
    /// When the clock was last set or corrected.
    pub reference_timestamp: NtpTimestamp,
    /// The client's transmit timestamp, echoed back. Unused in broadcast.
    pub origin_timestamp: NtpTimestamp,
    /// When the request arrived. Unused in broadcast.
    pub receive_timestamp: NtpTimestamp,
    /// When this packet left. The one that matters for a one-way broadcast.
    pub transmit_timestamp: NtpTimestamp,
}

impl LeapIndicator {
    /// From the 2-bit wire value. Total, since every 2-bit pattern is defined.
    const fn from_bits(bits: u8) -> Self {
        match bits & 0b11 {
            0 => Self::NoWarning,
            1 => Self::LastMinute61,
            2 => Self::LastMinute59,
            _ => Self::Unsynchronized,
        }
    }
}

impl Mode {
    /// From the 3-bit wire value. Total, since every 3-bit pattern is defined.
    const fn from_bits(bits: u8) -> Self {
        match bits & 0b111 {
            0 => Self::Reserved,
            1 => Self::SymmetricActive,
            2 => Self::SymmetricPassive,
            3 => Self::Client,
            4 => Self::Server,
            5 => Self::Broadcast,
            6 => Self::ControlMessage,
            _ => Self::Private,
        }
    }
}

/// Read a big-endian `NtpShort` from `buf[at..at + 4]`.
fn short_at(buf: &[u8], at: usize) -> NtpShort {
    let mut b = [0u8; 4];
    b.copy_from_slice(&buf[at..at + 4]);
    NtpShort::from_bits(u32::from_be_bytes(b))
}

/// Read a big-endian `NtpTimestamp` from `buf[at..at + 8]`.
fn timestamp_at(buf: &[u8], at: usize) -> NtpTimestamp {
    let mut b = [0u8; 8];
    b.copy_from_slice(&buf[at..at + 8]);
    NtpTimestamp::from_bits(u64::from_be_bytes(b))
}

impl NtpPacket {
    /// Serialise to the 48 wire bytes.
    pub fn encode(&self) -> [u8; PACKET_LEN] {
        let mut b = [0u8; PACKET_LEN];
        b[0] = ((self.leap as u8) << 6) | ((self.version & 0b111) << 3) | (self.mode as u8);
        b[1] = self.stratum;
        // `poll` and `precision` are signed log2 exponents; two's complement is just the bit cast.
        b[2] = self.poll as u8;
        b[3] = self.precision as u8;
        b[4..8].copy_from_slice(&self.root_delay.to_bits().to_be_bytes());
        b[8..12].copy_from_slice(&self.root_dispersion.to_bits().to_be_bytes());
        b[12..16].copy_from_slice(&self.reference_id);
        b[16..24].copy_from_slice(&self.reference_timestamp.to_bits().to_be_bytes());
        b[24..32].copy_from_slice(&self.origin_timestamp.to_bits().to_be_bytes());
        b[32..40].copy_from_slice(&self.receive_timestamp.to_bits().to_be_bytes());
        b[40..48].copy_from_slice(&self.transmit_timestamp.to_bits().to_be_bytes());
        b
    }

    /// Parse the 48 wire bytes. `None` if the buffer is too short. A longer buffer is accepted and
    /// anything past byte 48 (extension fields, MAC) is ignored — this crate does not use them.
    pub fn decode(buf: &[u8]) -> Option<Self> {
        if buf.len() < PACKET_LEN {
            return None;
        }
        let mut reference_id = [0u8; 4];
        reference_id.copy_from_slice(&buf[12..16]);
        Some(Self {
            leap: LeapIndicator::from_bits(buf[0] >> 6),
            version: (buf[0] >> 3) & 0b111,
            mode: Mode::from_bits(buf[0]),
            stratum: buf[1],
            poll: buf[2] as i8,
            precision: buf[3] as i8,
            root_delay: short_at(buf, 4),
            root_dispersion: short_at(buf, 8),
            reference_id,
            reference_timestamp: timestamp_at(buf, 16),
            origin_timestamp: timestamp_at(buf, 24),
            receive_timestamp: timestamp_at(buf, 32),
            transmit_timestamp: timestamp_at(buf, 40),
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A Stratum-1 GPS-disciplined broadcast packet, of the shape this crate exists to produce.
    fn sample() -> NtpPacket {
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
            reference_timestamp: NtpTimestamp::from_unix_ns(1_787_020_967 * 1_000_000_000),
            origin_timestamp: NtpTimestamp::ZERO,
            receive_timestamp: NtpTimestamp::ZERO,
            transmit_timestamp: NtpTimestamp::from_unix_ns(
                1_787_020_967 * 1_000_000_000 + 500_000_000,
            ),
        }
    }

    #[test]
    fn header_is_48_bytes() {
        // Pin the constant itself: asserting `encode().len()` would be a tautology, since the
        // return type is `[u8; PACKET_LEN]` and would follow any change to the constant.
        assert_eq!(PACKET_LEN, 48);
    }

    #[test]
    fn byte_zero_packs_leap_version_and_mode() {
        // LI(2) | VN(3) | Mode(3) — LI=0, VN=4, Mode=5 (broadcast) = 0b00_100_101.
        assert_eq!(sample().encode()[0], 0x25);
        // An unsynchronised server flips the top two bits.
        let mut p = sample();
        p.leap = LeapIndicator::Unsynchronized;
        assert_eq!(p.encode()[0], 0xE5);
    }

    #[test]
    fn stratum_poll_and_precision_follow_byte_zero() {
        let b = sample().encode();
        assert_eq!(b[1], 1); // stratum 1 = primary reference
        assert_eq!(b[2], 4); // poll = 2^4 s
        assert_eq!(b[3], 0xE6); // precision -26, two's complement
    }

    #[test]
    fn root_delay_and_dispersion_are_big_endian_shorts() {
        let b = sample().encode();
        assert_eq!(&b[4..8], &[0, 0, 0, 0]);
        let disp = NtpShort::from_nanos(1_000_000).to_bits();
        assert_eq!(&b[8..12], &disp.to_be_bytes());
    }

    #[test]
    fn reference_id_is_four_ascii_bytes() {
        assert_eq!(&sample().encode()[12..16], b"GPS\0");
    }

    #[test]
    fn the_four_timestamps_are_big_endian_at_16_24_32_40() {
        let p = sample();
        let b = p.encode();
        assert_eq!(&b[16..24], &p.reference_timestamp.to_bits().to_be_bytes());
        assert_eq!(&b[24..32], &p.origin_timestamp.to_bits().to_be_bytes());
        assert_eq!(&b[32..40], &p.receive_timestamp.to_bits().to_be_bytes());
        assert_eq!(&b[40..48], &p.transmit_timestamp.to_bits().to_be_bytes());
    }

    #[test]
    fn encode_decode_round_trip_preserves_every_field() {
        let p = sample();
        assert_eq!(NtpPacket::decode(&p.encode()), Some(p));
    }

    #[test]
    fn decode_rejects_a_short_buffer() {
        assert_eq!(NtpPacket::decode(&[0u8; PACKET_LEN - 1]), None);
    }

    #[test]
    fn decode_accepts_a_longer_buffer_ignoring_extension_fields() {
        // Extension fields / MAC may follow the header; the first 48 bytes still parse.
        let p = sample();
        let mut buf = [0u8; PACKET_LEN + 20];
        buf[..PACKET_LEN].copy_from_slice(&p.encode());
        assert_eq!(NtpPacket::decode(&buf), Some(p));
    }

    #[test]
    fn decode_reads_a_client_request_from_the_wire() {
        // A minimal NTPv4 client request: LI=0 VN=4 Mode=3, everything else zero.
        let mut raw = [0u8; PACKET_LEN];
        raw[0] = 0x23;
        let p = NtpPacket::decode(&raw).expect("48 bytes must parse");
        assert_eq!(p.mode, Mode::Client);
        assert_eq!(p.version, 4);
        assert_eq!(p.leap, LeapIndicator::NoWarning);
        assert_eq!(p.stratum, 0);
    }

    #[test]
    fn negative_precision_survives_the_wire_as_signed() {
        let mut p = sample();
        p.precision = -20;
        let decoded = NtpPacket::decode(&p.encode()).unwrap();
        assert_eq!(decoded.precision, -20);
    }
}
