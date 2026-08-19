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

use crate::frame::crc32;
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

/// Samples the deserialiser takes per symbol on the line.
pub const OVERSAMPLE: usize = 2;

/// Decode one capture into `out`, and say how many bytes the frame carried.
///
/// A capture holds two samples per symbol and does not say which of them is the one inside the
/// symbol — see [`crate::phy::RX_PIO_CLOCK_HZ`] for why nothing can. Both are tried, and the FCS
/// decides: a frame read at the wrong phase does not survive its own checksum. The returned length
/// covers the frame without the four FCS bytes, which have already been checked.
pub fn decode_frame(words: &[u32], out: &mut [u8]) -> Option<usize> {
    let mut phase = 0;
    while phase < OVERSAMPLE {
        if let Some(len) = decode_at_phase(words, phase, out) {
            return Some(len);
        }
        phase += 1;
    }
    None
}

/// Read the capture taking every [`OVERSAMPLE`]th sample from `phase`, and return the frame if the
/// bytes that came out check against the FCS they carry.
fn decode_at_phase(words: &[u32], phase: usize, out: &mut [u8]) -> Option<usize> {
    let mut decoder = FrameDecoder::new();
    let mut len = 0usize;
    let mut ended = false;
    let mut index = 0usize;
    'capture: for word in words {
        for symbol in symbols_of(*word) {
            let mine = index % OVERSAMPLE == phase;
            index += 1;
            if !mine {
                continue;
            }
            match decoder.push(symbol) {
                RxEvent::Byte(byte) => {
                    if len == out.len() {
                        return None;
                    }
                    out[len] = byte;
                    len += 1;
                }
                RxEvent::End => {
                    ended = true;
                    break 'capture;
                }
                RxEvent::Start | RxEvent::Nothing => {}
            }
        }
    }
    // Four bytes of FCS and something for it to cover.
    if !ended || len < 5 {
        return None;
    }
    let split = len - 4;
    let carried = u32::from_le_bytes([out[split], out[split + 1], out[split + 2], out[split + 3]]);
    if crc32(&out[..split]) != carried {
        return None;
    }
    Some(split)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::frame::{MacAddr, UdpFrameSpec, build_udp_frame};
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

    /// Expand an encoded stream into what the deserialiser captures: two samples per symbol,
    /// with the first `offset` of them already gone.
    ///
    /// `offset` is the sampling phase the hardware happened to land on. It is not a choice, and the
    /// receiver cannot see which one it got.
    fn oversample(words: &[u32], offset: usize, out: &mut [u32]) -> usize {
        let mut count = 0;
        let mut word = 0u32;
        let mut held = 0;
        let mut index = 0;
        for w in words {
            for symbol in symbols_of(*w) {
                for _ in 0..OVERSAMPLE {
                    if index >= offset {
                        word |= symbol << (2 * held);
                        held += 1;
                        if held == 16 {
                            out[count] = word;
                            count += 1;
                            word = 0;
                            held = 0;
                        }
                    }
                    index += 1;
                }
            }
        }
        if held > 0 {
            out[count] = word;
            count += 1;
        }
        count
    }

    #[test]
    fn a_frame_decodes_whichever_sampling_phase_the_capture_landed_on() {
        let mut frame = [0u8; MAX_FRAME];
        let mut symbols = [0u32; MAX_WORDS];
        let (len, words) = ntp_sized(&mut frame, &mut symbols);

        for offset in 0..OVERSAMPLE {
            let mut capture = [0u32; MAX_WORDS * OVERSAMPLE + 2];
            let taken = oversample(&symbols[..words], offset, &mut capture);
            let mut out = [0u8; MAX_FRAME];
            let got = decode_frame(&capture[..taken], &mut out)
                .unwrap_or_else(|| panic!("phase {offset} did not decode"));
            // The FCS is checked and dropped, so what comes back is the frame that was built.
            assert_eq!(got, len - 4, "phase {offset} length");
            assert_eq!(&out[..got], &frame[..len - 4], "phase {offset} bytes");
        }
    }

    #[test]
    fn a_capture_of_nothing_but_idle_decodes_to_nothing() {
        let capture = [0u32; 32];
        let mut out = [0u8; MAX_FRAME];
        assert_eq!(decode_frame(&capture, &mut out), None);
    }

    #[test]
    fn a_capture_with_a_flipped_bit_is_refused() {
        let mut frame = [0u8; MAX_FRAME];
        let mut symbols = [0u32; MAX_WORDS];
        let (_, words) = ntp_sized(&mut frame, &mut symbols);
        let mut capture = [0u32; MAX_WORDS * OVERSAMPLE + 2];
        let taken = oversample(&symbols[..words], 0, &mut capture);
        // Swap one symbol pair in the middle of the frame: the length still works out, and only
        // the FCS says the bits did not survive.
        capture[60] ^= 0b11 << 8;
        capture[60] ^= 0b11 << 10;
        let mut out = [0u8; MAX_FRAME];
        assert_eq!(decode_frame(&capture[..taken], &mut out), None);
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
