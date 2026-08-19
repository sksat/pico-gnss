//! The four messages an end-to-end two-step exchange needs, on the wire.
//!
//! IEEE 1588 puts a 34-byte header in front of every message and a body behind it. Everything here
//! is that header and the four bodies; nothing here knows how the bytes travel.

/// Bytes in the common header (IEEE 1588-2008 §13.3).
pub const HEADER_LEN: usize = 34;

/// Bytes in a `Timestamp` on the wire: 48-bit seconds, then 32-bit nanoseconds.
pub const TIMESTAMP_LEN: usize = 10;

/// Bytes in a `PortIdentity`: an 8-byte clock identity, then a 16-bit port number.
pub const PORT_IDENTITY_LEN: usize = 10;

/// The version this speaks. There is no negotiation; a message in another version is refused.
pub const VERSION: u8 = 2;

/// The longest message this crate builds — a `Delay_Resp`, which carries a port identity as well
/// as a timestamp.
pub const MAX_MESSAGE_LEN: usize = HEADER_LEN + TIMESTAMP_LEN + PORT_IDENTITY_LEN;

/// A point in time as IEEE 1588 writes it: unsigned seconds and nanoseconds within the second.
///
/// **The epoch is the caller's.** The standard's own timescale is TAI since 1970, and this pair of
/// boards has no TAI — what they have is UTC from a GNSS receiver. So the timescale here is
/// whatever went in, the `ptpTimescale` flag is left clear to say as much, and the two ends agree
/// because they were built together. That is a profile decision and not a standard one.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Timestamp {
    /// Seconds. 48 bits on the wire, so anything above that cannot be represented.
    pub seconds: u64,
    /// Nanoseconds within the second, below 1e9.
    pub nanos: u32,
}

impl Timestamp {
    /// The largest second the wire format can carry.
    pub const MAX_SECONDS: u64 = (1 << 48) - 1;

    /// Split a nanosecond count since the epoch. Negative times have no representation and clamp
    /// to zero, which is what the format allows and not a number to build a clock on.
    pub fn from_ns(ns: i64) -> Self {
        if ns <= 0 {
            return Self::default();
        }
        Self {
            seconds: (ns as u64) / 1_000_000_000,
            nanos: ((ns as u64) % 1_000_000_000) as u32,
        }
    }

    /// Back to a nanosecond count. Saturates rather than wrapping: a 48-bit seconds field can hold
    /// more than an `i64` of nanoseconds.
    pub fn to_ns(self) -> i64 {
        self.seconds
            .saturating_mul(1_000_000_000)
            .saturating_add(self.nanos as u64)
            .min(i64::MAX as u64) as i64
    }
}

/// Eight bytes naming a clock. Derived from a MAC address in the usual way, or chosen.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct ClockIdentity(pub [u8; 8]);

impl ClockIdentity {
    /// The EUI-64 form of a MAC address: the standard's own suggestion, and it makes the identity
    /// on the wire traceable to the board that sent it.
    pub fn from_mac(mac: [u8; 6]) -> Self {
        Self([mac[0], mac[1], mac[2], 0xFF, 0xFE, mac[3], mac[4], mac[5]])
    }
}

/// A clock and one of its ports. What a message says it came from, and what a `Delay_Resp` says it
/// is answering.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct PortIdentity {
    pub clock: ClockIdentity,
    pub port: u16,
}

/// The message types this crate speaks. The others exist and are refused.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum MessageType {
    Sync,
    DelayReq,
    FollowUp,
    DelayResp,
}

impl MessageType {
    /// The low nibble of the first octet.
    pub fn code(self) -> u8 {
        match self {
            Self::Sync => 0x0,
            Self::DelayReq => 0x1,
            Self::FollowUp => 0x8,
            Self::DelayResp => 0x9,
        }
    }

    fn from_code(code: u8) -> Option<Self> {
        match code {
            0x0 => Some(Self::Sync),
            0x1 => Some(Self::DelayReq),
            0x8 => Some(Self::FollowUp),
            0x9 => Some(Self::DelayResp),
            _ => None,
        }
    }

    /// The `controlField`, which version 2 still carries and version 2.1 deprecates. Written
    /// because a receiver of the older version may check it.
    fn control(self) -> u8 {
        match self {
            Self::Sync => 0,
            Self::DelayReq => 1,
            Self::FollowUp => 2,
            Self::DelayResp => 3,
        }
    }
}

/// What follows the header.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Body {
    /// `Sync`. In a two-step exchange this timestamp is a placeholder — the one that counts arrives
    /// in the `Follow_Up`.
    Sync(Timestamp),
    /// `Follow_Up`: when the `Sync` it names actually left.
    FollowUp(Timestamp),
    /// `Delay_Req`. Its timestamp is a placeholder for the same reason: what counts is when it
    /// left, and only the sender's own hardware knows that.
    DelayReq(Timestamp),
    /// `Delay_Resp`: when the `Delay_Req` arrived, and whose it was.
    DelayResp {
        receive: Timestamp,
        requesting: PortIdentity,
    },
}

impl Body {
    fn message_type(&self) -> MessageType {
        match self {
            Self::Sync(_) => MessageType::Sync,
            Self::FollowUp(_) => MessageType::FollowUp,
            Self::DelayReq(_) => MessageType::DelayReq,
            Self::DelayResp { .. } => MessageType::DelayResp,
        }
    }

    fn len(&self) -> usize {
        match self {
            Self::DelayResp { .. } => TIMESTAMP_LEN + PORT_IDENTITY_LEN,
            _ => TIMESTAMP_LEN,
        }
    }
}

/// One message, header and body.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Message {
    pub domain: u8,
    /// Set on a `Sync` whose real departure will follow. Clear on everything else.
    pub two_step: bool,
    /// Residence and asymmetry corrections, in nanoseconds scaled by 2¹⁶ — the standard's own
    /// unit, kept rather than converted so that nothing is lost passing a message on.
    pub correction_sub_ns: i64,
    pub source: PortIdentity,
    pub sequence: u16,
    /// `logMessageInterval`: the base-two logarithm of the interval this kind of message is sent
    /// at. A `Delay_Req` has none and writes 0x7F.
    pub log_interval: i8,
    pub body: Body,
}

impl Message {
    /// Bytes this message occupies on the wire.
    pub fn len(&self) -> usize {
        HEADER_LEN + self.body.len()
    }

    /// Never empty; present because clippy asks for it beside `len`.
    pub fn is_empty(&self) -> bool {
        false
    }

    /// Correction in whole nanoseconds, rounded towards zero.
    pub fn correction_ns(&self) -> i64 {
        self.correction_sub_ns / 65_536
    }
}

fn write_timestamp(out: &mut [u8], ts: Timestamp) {
    let seconds = ts.seconds.to_be_bytes();
    // Six of the eight, big end first: the field is 48 bits.
    out[0..6].copy_from_slice(&seconds[2..8]);
    out[6..10].copy_from_slice(&ts.nanos.to_be_bytes());
}

fn read_timestamp(buf: &[u8]) -> Timestamp {
    let mut seconds = [0u8; 8];
    seconds[2..8].copy_from_slice(&buf[0..6]);
    Timestamp {
        seconds: u64::from_be_bytes(seconds),
        nanos: u32::from_be_bytes([buf[6], buf[7], buf[8], buf[9]]),
    }
}

fn write_port_identity(out: &mut [u8], id: PortIdentity) {
    out[0..8].copy_from_slice(&id.clock.0);
    out[8..10].copy_from_slice(&id.port.to_be_bytes());
}

fn read_port_identity(buf: &[u8]) -> Option<PortIdentity> {
    Some(PortIdentity {
        clock: ClockIdentity(buf[0..8].try_into().ok()?),
        port: u16::from_be_bytes([buf[8], buf[9]]),
    })
}

/// Write `msg` into `out`, returning how many bytes it took.
pub fn encode(msg: &Message, out: &mut [u8]) -> Option<usize> {
    let len = msg.len();
    if out.len() < len {
        return None;
    }
    let out = &mut out[..len];
    out.fill(0);

    let kind = msg.body.message_type();
    // The high nibble is transportSpecific, which this profile leaves at zero.
    out[0] = kind.code() & 0x0F;
    out[1] = VERSION & 0x0F;
    out[2..4].copy_from_slice(&(len as u16).to_be_bytes());
    out[4] = msg.domain;
    // flagField. Only two bits are ours to set: twoStep, and the timescale — which stays clear,
    // because the times in here are UTC from a receiver and not the standard's TAI.
    if msg.two_step {
        out[6] |= 0x02;
    }
    out[8..16].copy_from_slice(&msg.correction_sub_ns.to_be_bytes());
    write_port_identity(&mut out[20..30], msg.source);
    out[30..32].copy_from_slice(&msg.sequence.to_be_bytes());
    out[32] = kind.control();
    out[33] = msg.log_interval as u8;

    match msg.body {
        Body::Sync(ts) | Body::FollowUp(ts) | Body::DelayReq(ts) => {
            write_timestamp(&mut out[HEADER_LEN..HEADER_LEN + TIMESTAMP_LEN], ts);
        }
        Body::DelayResp {
            receive,
            requesting,
        } => {
            write_timestamp(&mut out[HEADER_LEN..HEADER_LEN + TIMESTAMP_LEN], receive);
            write_port_identity(&mut out[HEADER_LEN + TIMESTAMP_LEN..len], requesting);
        }
    }
    Some(len)
}

/// Read one message. `None` for anything this crate does not speak, including a version it does
/// not know and a length that disagrees with the type.
pub fn decode(buf: &[u8]) -> Option<Message> {
    if buf.len() < HEADER_LEN {
        return None;
    }
    if buf[1] & 0x0F != VERSION {
        return None;
    }
    let kind = MessageType::from_code(buf[0] & 0x0F)?;
    let body_len = match kind {
        MessageType::DelayResp => TIMESTAMP_LEN + PORT_IDENTITY_LEN,
        _ => TIMESTAMP_LEN,
    };
    let len = HEADER_LEN + body_len;
    // Both directions. A declared length that disagrees with the type is a message built wrong,
    // and one longer than what arrived is a message cut short.
    if u16::from_be_bytes([buf[2], buf[3]]) as usize != len || buf.len() < len {
        return None;
    }

    let ts = read_timestamp(&buf[HEADER_LEN..HEADER_LEN + TIMESTAMP_LEN]);
    let body = match kind {
        MessageType::Sync => Body::Sync(ts),
        MessageType::FollowUp => Body::FollowUp(ts),
        MessageType::DelayReq => Body::DelayReq(ts),
        MessageType::DelayResp => Body::DelayResp {
            receive: ts,
            requesting: read_port_identity(&buf[HEADER_LEN + TIMESTAMP_LEN..len])?,
        },
    };

    Some(Message {
        domain: buf[4],
        two_step: buf[6] & 0x02 != 0,
        correction_sub_ns: i64::from_be_bytes(buf[8..16].try_into().ok()?),
        source: read_port_identity(&buf[20..30])?,
        sequence: u16::from_be_bytes([buf[30], buf[31]]),
        log_interval: buf[33] as i8,
        body,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    fn identity() -> PortIdentity {
        PortIdentity {
            clock: ClockIdentity::from_mac([0x02, 0x00, 0x00, 0xC0, 0xFF, 0xEE]),
            port: 1,
        }
    }

    fn sync() -> Message {
        Message {
            domain: 0,
            two_step: true,
            correction_sub_ns: 0,
            source: identity(),
            sequence: 0x1234,
            log_interval: 0,
            body: Body::Sync(Timestamp::default()),
        }
    }

    #[test]
    fn a_mac_becomes_a_clock_identity_the_standard_way() {
        // EUI-64: the first three octets, then 0xFF 0xFE, then the last three.
        assert_eq!(
            ClockIdentity::from_mac([0x02, 0x00, 0x00, 0xC0, 0xFF, 0xEE]).0,
            [0x02, 0x00, 0x00, 0xFF, 0xFE, 0xC0, 0xFF, 0xEE]
        );
    }

    #[test]
    fn the_header_puts_each_field_where_the_standard_says() {
        let mut buf = [0u8; MAX_MESSAGE_LEN];
        let mut msg = sync();
        msg.domain = 7;
        msg.correction_sub_ns = -3 * 65_536;
        msg.log_interval = -4;
        let len = encode(&msg, &mut buf).expect("encodes");

        assert_eq!(
            buf[0] & 0x0F,
            0x00,
            "octet 0 low nibble is the message type"
        );
        assert_eq!(buf[0] >> 4, 0, "octet 0 high nibble is transportSpecific");
        assert_eq!(buf[1] & 0x0F, VERSION, "octet 1 low nibble is versionPTP");
        assert_eq!(
            u16::from_be_bytes([buf[2], buf[3]]) as usize,
            len,
            "octets 2-3 are messageLength"
        );
        assert_eq!(buf[4], 7, "octet 4 is domainNumber");
        assert_eq!(buf[6] & 0x02, 0x02, "octet 6 bit 1 is twoStepFlag");
        assert_eq!(
            i64::from_be_bytes(buf[8..16].try_into().unwrap()),
            -3 * 65_536,
            "octets 8-15 are correctionField"
        );
        assert_eq!(
            &buf[20..28],
            &identity().clock.0,
            "octets 20-27 are the clock"
        );
        assert_eq!(
            u16::from_be_bytes([buf[28], buf[29]]),
            1,
            "octets 28-29 are the port number"
        );
        assert_eq!(
            u16::from_be_bytes([buf[30], buf[31]]),
            0x1234,
            "octets 30-31 are sequenceId"
        );
        assert_eq!(buf[32], 0, "octet 32 is controlField, and Sync's is zero");
        assert_eq!(buf[33] as i8, -4, "octet 33 is logMessageInterval");
    }

    #[test]
    fn a_timestamp_is_six_bytes_of_seconds_and_four_of_nanoseconds() {
        let mut buf = [0u8; MAX_MESSAGE_LEN];
        let mut msg = sync();
        msg.body = Body::Sync(Timestamp {
            seconds: 0x0000_1234_5678_9ABC & Timestamp::MAX_SECONDS,
            nanos: 123_456_789,
        });
        encode(&msg, &mut buf).expect("encodes");
        assert_eq!(&buf[34..40], &[0x12, 0x34, 0x56, 0x78, 0x9A, 0xBC]);
        assert_eq!(
            u32::from_be_bytes(buf[40..44].try_into().unwrap()),
            123_456_789
        );
    }

    #[test]
    fn each_kind_is_as_long_as_the_standard_makes_it() {
        let mut buf = [0u8; MAX_MESSAGE_LEN];
        for (body, want) in [
            (Body::Sync(Timestamp::default()), 44),
            (Body::FollowUp(Timestamp::default()), 44),
            (Body::DelayReq(Timestamp::default()), 44),
            (
                Body::DelayResp {
                    receive: Timestamp::default(),
                    requesting: identity(),
                },
                54,
            ),
        ] {
            let msg = Message { body, ..sync() };
            assert_eq!(encode(&msg, &mut buf), Some(want), "{body:?}");
            assert_eq!(msg.len(), want);
        }
    }

    #[test]
    fn what_was_written_reads_back() {
        let mut buf = [0u8; MAX_MESSAGE_LEN];
        for body in [
            Body::Sync(Timestamp {
                seconds: 1_787_180_000,
                nanos: 1,
            }),
            Body::FollowUp(Timestamp {
                seconds: 1_787_180_000,
                nanos: 236_200,
            }),
            Body::DelayReq(Timestamp::default()),
            Body::DelayResp {
                receive: Timestamp {
                    seconds: 1_787_180_001,
                    nanos: 999_999_999,
                },
                requesting: PortIdentity {
                    clock: ClockIdentity([1, 2, 3, 4, 5, 6, 7, 8]),
                    port: 0xBEEF,
                },
            },
        ] {
            let msg = Message {
                correction_sub_ns: 42 * 65_536,
                two_step: matches!(body, Body::Sync(_)),
                body,
                ..sync()
            };
            let len = encode(&msg, &mut buf).expect("encodes");
            assert_eq!(decode(&buf[..len]), Some(msg), "{body:?}");
        }
    }

    #[test]
    fn a_buffer_too_small_is_refused_rather_than_truncated() {
        let msg = sync();
        let mut small = [0u8; 43];
        assert_eq!(encode(&msg, &mut small), None);
    }

    #[test]
    fn a_version_this_does_not_speak_is_refused() {
        let mut buf = [0u8; MAX_MESSAGE_LEN];
        let len = encode(&sync(), &mut buf).expect("encodes");
        buf[1] = (buf[1] & 0xF0) | 1;
        assert_eq!(decode(&buf[..len]), None);
    }

    #[test]
    fn a_message_type_this_does_not_speak_is_refused() {
        let mut buf = [0u8; MAX_MESSAGE_LEN];
        let len = encode(&sync(), &mut buf).expect("encodes");
        // 0x0B is Announce, which this crate has no use for and must not pretend to read.
        buf[0] = (buf[0] & 0xF0) | 0x0B;
        assert_eq!(decode(&buf[..len]), None);
    }

    #[test]
    fn a_length_that_disagrees_with_the_type_is_refused() {
        let mut buf = [0u8; MAX_MESSAGE_LEN];
        let len = encode(&sync(), &mut buf).expect("encodes");
        // A Sync claiming a Delay_Resp's length, and a Sync cut short.
        buf[2..4].copy_from_slice(&54u16.to_be_bytes());
        assert_eq!(decode(&buf[..len]), None);
        buf[2..4].copy_from_slice(&(len as u16).to_be_bytes());
        assert_eq!(decode(&buf[..len - 1]), None);
    }

    #[test]
    fn a_delay_request_says_it_has_no_interval() {
        let mut buf = [0u8; MAX_MESSAGE_LEN];
        let msg = Message {
            body: Body::DelayReq(Timestamp::default()),
            log_interval: 0x7F,
            two_step: false,
            ..sync()
        };
        encode(&msg, &mut buf).expect("encodes");
        assert_eq!(buf[33], 0x7F);
    }

    #[test]
    fn nanoseconds_survive_the_round_trip_through_a_timestamp() {
        for ns in [
            0i64,
            1,
            999_999_999,
            1_000_000_000,
            1_787_180_641_176_545_455,
        ] {
            assert_eq!(Timestamp::from_ns(ns).to_ns(), ns, "{ns}");
        }
        // A time before the epoch has no representation, and saying so is better than wrapping.
        assert_eq!(Timestamp::from_ns(-1), Timestamp::default());
    }
}
