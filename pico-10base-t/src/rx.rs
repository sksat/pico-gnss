//! Turning the line back into bytes.
//!
//! The transmitter drives two pins as a pair and the receiver reads the same two, so what arrives
//! is a stream of the three states [`crate::phy::SYMBOL_IDLE`], [`crate::phy::SYMBOL_LOW`] and
//! [`crate::phy::SYMBOL_HIGH`] — one every 50 ns at 10 Mbit/s. This module takes that stream and
//! gives back the frame.
//!
//! What it does not do is recover the clock. It assumes each symbol it is handed is one symbol
//! period on the line, which is what a PIO state machine clocked at [`crate::phy::PIO_CLOCK_HZ`]
//! delivers. Two RP2040 crystals differ by tens of ppm, which over the 81.6 µs of a 102-byte frame
//! is a few nanoseconds against a 50 ns symbol — far from enough to slip. Alignment comes from the
//! frame itself: the first non-idle symbol after silence is the first half of the first bit.

use crate::phy::{SYMBOL_HIGH, SYMBOL_IDLE, SYMBOL_LOW};

/// Start of Frame Delimiter. The preamble alternates; this is where it stops.
const SFD: u8 = 0xD5;

/// What one symbol did to the decoder.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum RxEvent {
    /// Nothing completed.
    Nothing,
    /// The SFD went by: what follows is the frame.
    Start,
    /// One byte of the frame, in the order it arrived.
    Byte(u8),
    /// The line went idle. Everything between [`RxEvent::Start`] and here was the frame.
    End,
}

/// Where the decoder is in a frame.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
enum State {
    /// Waiting for the line to leave idle.
    Idle,
    /// Reading bits, looking for the SFD in the last eight of them.
    Hunting,
    /// Past the SFD: every eight bits is a byte.
    Framing,
}

/// Decodes the line one symbol at a time.
///
/// Feed it every symbol, including the idle ones — leaving idle is how it learns where a bit
/// begins, and returning to idle is how it learns the frame ended.
#[derive(Clone, Copy, Debug)]
pub struct FrameDecoder {
    state: State,
    /// The first half of the bit being read, if a half is outstanding.
    pending: Option<u32>,
    /// The last eight bits, newest in the high bit — the shape the SFD is compared against.
    window: u8,
    /// Bits collected into the byte being assembled, and how many.
    byte: u8,
    bits: u8,
}

impl Default for FrameDecoder {
    fn default() -> Self {
        Self::new()
    }
}

impl FrameDecoder {
    pub const fn new() -> Self {
        Self {
            state: State::Idle,
            pending: None,
            window: 0,
            byte: 0,
            bits: 0,
        }
    }

    /// Forget everything and wait for the line to go idle again.
    pub fn reset(&mut self) {
        *self = Self::new();
    }

    /// Feed one symbol.
    pub fn push(&mut self, symbol: u32) -> RxEvent {
        if symbol == SYMBOL_IDLE {
            return self.finish();
        }
        if self.state == State::Idle {
            // Leaving idle is what fixes the alignment: this symbol is the first half of a bit.
            self.state = State::Hunting;
            self.pending = Some(symbol);
            return RxEvent::Nothing;
        }
        let Some(first) = self.pending.take() else {
            self.pending = Some(symbol);
            return RxEvent::Nothing;
        };
        match (first, symbol) {
            (SYMBOL_HIGH, SYMBOL_LOW) => self.absorb(0),
            (SYMBOL_LOW, SYMBOL_HIGH) => self.absorb(1),
            // Not a Manchester pair. The transmitter ends every frame this way — TP_IDL is six
            // half-bits of HIGH — so this is the ordinary end of a frame rather than an error.
            _ => self.finish(),
        }
    }

    /// The frame is over, whether the line went idle or the coding stopped making sense.
    fn finish(&mut self) -> RxEvent {
        let framing = self.state == State::Framing;
        self.reset();
        if framing {
            RxEvent::End
        } else {
            RxEvent::Nothing
        }
    }

    /// One decoded bit. Bits arrive least significant first, both in the SFD and in every byte.
    fn absorb(&mut self, bit: u8) -> RxEvent {
        match self.state {
            State::Idle => RxEvent::Nothing,
            State::Hunting => {
                self.window = (self.window >> 1) | (bit << 7);
                if self.window == SFD {
                    self.state = State::Framing;
                    self.byte = 0;
                    self.bits = 0;
                    RxEvent::Start
                } else {
                    RxEvent::Nothing
                }
            }
            State::Framing => {
                self.byte = (self.byte >> 1) | (bit << 7);
                self.bits += 1;
                if self.bits == 8 {
                    let done = self.byte;
                    self.byte = 0;
                    self.bits = 0;
                    RxEvent::Byte(done)
                } else {
                    RxEvent::Nothing
                }
            }
        }
    }
}

/// Expand the words [`crate::phy::encode_frame`] produces back into the symbols they play out as.
///
/// The PIO shifts its OSR right, so the symbol that leaves first is the low two bits of the word.
/// Tests use this to drive a decoder with exactly what the wire would carry.
pub fn symbols_of(word: u32) -> [u32; 16] {
    let mut out = [0u32; 16];
    let mut i = 0;
    while i < 16 {
        out[i] = (word >> (2 * i)) & 0b11;
        i += 1;
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::frame::{MacAddr, UdpFrameSpec, build_udp_frame, crc32};
    use crate::phy::{encode_frame, encoded_words};

    /// The largest frame these tests build, and the symbols it turns into.
    const MAX_FRAME: usize = 128;
    const MAX_WORDS: usize = encoded_words(MAX_FRAME);

    /// Play a word stream through a decoder and collect the frame it hands back.
    fn receive(words: &[u32], out: &mut [u8]) -> (usize, bool) {
        let mut decoder = FrameDecoder::new();
        let mut len = 0;
        let mut started = false;
        let mut ended = false;
        // A little idle in front, so the decoder sees the line leave idle.
        let stream = core::iter::repeat_n(SYMBOL_IDLE, 8)
            .chain(words.iter().flat_map(|w| symbols_of(*w)))
            .chain(core::iter::repeat_n(SYMBOL_IDLE, 8));
        for symbol in stream {
            match decoder.push(symbol) {
                RxEvent::Start => started = true,
                RxEvent::Byte(b) => {
                    out[len] = b;
                    len += 1;
                }
                RxEvent::End => ended = true,
                RxEvent::Nothing => {}
            }
        }
        assert!(started, "the decoder never found the SFD");
        (len, ended)
    }

    /// Build a frame the size `pico-ntp` actually sends, and encode it.
    fn ntp_sized(frame: &mut [u8; MAX_FRAME], symbols: &mut [u32; MAX_WORDS]) -> (usize, usize) {
        let spec = UdpFrameSpec {
            src_mac: MacAddr([0x02, 0x00, 0x00, 0xC0, 0xFF, 0xEE]),
            dst_mac: MacAddr::BROADCAST,
            src_ip: crate::frame::Ipv4Addr::new(192, 168, 0, 200),
            dst_ip: crate::frame::Ipv4Addr::BROADCAST,
            src_port: 123,
            dst_port: 123,
            ip_id: 1,
            ttl: 1,
            payload: &[0xA5; 48],
        };
        let len = build_udp_frame(&spec, frame).expect("frame fits");
        let words = encode_frame(&frame[..len], symbols).expect("encodes");
        (len, words)
    }

    #[test]
    fn what_the_transmitter_sent_is_what_the_receiver_reads() {
        let mut frame = [0u8; MAX_FRAME];
        let mut symbols = [0u32; MAX_WORDS];
        let (len, words) = ntp_sized(&mut frame, &mut symbols);
        let mut got = [0u8; MAX_FRAME];
        let (got_len, ended) = receive(&symbols[..words], &mut got);
        assert_eq!(
            &got[..got_len],
            &frame[..len],
            "the bytes come back unchanged"
        );
        assert!(ended, "returning to idle has to end the frame");
    }

    #[test]
    fn the_frame_that_comes_back_still_checks_out() {
        // The FCS is over everything before it, and it is the last four bytes. A decoder that drops
        // or doubles a bit anywhere would show up here even if the length happened to survive.
        let mut frame = [0u8; MAX_FRAME];
        let mut symbols = [0u32; MAX_WORDS];
        let (_, words) = ntp_sized(&mut frame, &mut symbols);
        let mut got = [0u8; MAX_FRAME];
        let (got_len, _) = receive(&symbols[..words], &mut got);
        let split = got_len - 4;
        let carried = u32::from_le_bytes(got[split..got_len].try_into().expect("four bytes"));
        assert_eq!(crc32(&got[..split]), carried, "FCS must verify");
    }

    #[test]
    fn a_line_that_never_leaves_idle_yields_nothing() {
        let mut decoder = FrameDecoder::new();
        for _ in 0..1000 {
            assert_eq!(decoder.push(SYMBOL_IDLE), RxEvent::Nothing);
        }
    }

    #[test]
    fn the_two_half_bits_of_a_bit_are_what_decides_it() {
        // 0 is HIGH then LOW, 1 is LOW then HIGH. Nothing else about the pair matters.
        let mut zero = FrameDecoder::new();
        let mut one = FrameDecoder::new();
        assert_eq!(zero.push(SYMBOL_HIGH), RxEvent::Nothing);
        assert_eq!(zero.push(SYMBOL_LOW), RxEvent::Nothing);
        assert_eq!(one.push(SYMBOL_LOW), RxEvent::Nothing);
        assert_eq!(one.push(SYMBOL_HIGH), RxEvent::Nothing);
        // Neither has reached eight bits, so neither has produced a byte yet; the point is that
        // both accepted the pair rather than resetting.
        assert_ne!(zero.state, State::Idle);
        assert_ne!(one.state, State::Idle);
    }
}
