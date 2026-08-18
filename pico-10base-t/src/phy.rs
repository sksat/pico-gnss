//! Line coding for 10BASE-T: Manchester symbols, the preamble, and the signals that frame a
//! transmission.
//!
//! # The symbol representation
//!
//! This follows [kingyoPiyo/Pico-10BASE-T](https://github.com/kingyoPiyo/Pico-10BASE-T) exactly,
//! because the encoding is inseparable from the PIO program that plays it back. That program is
//! three instructions long:
//!
//! ```text
//! .side_set 2
//! .origin 0
//! .wrap_target
//!     out pc, 2  side 0b00    ; address 0 = IDLE
//!     out pc, 2  side 0b01    ; address 1 = LOW
//!     out pc, 2  side 0b10    ; address 2 = HIGH
//! .wrap
//! ```
//!
//! Each instruction drives the differential pair to the state its own address stands for, then
//! pulls two bits and *jumps to them*. So a 2-bit symbol is not data to be interpreted — it is the
//! address of the next line state, and the instruction slots **are** the states. One instruction
//! per PIO cycle at 20 MHz gives a 50 ns half-bit, so a 10 Mbit/s bit is two symbols.
//!
//! Symbols are consumed LSB-first (the state machine shifts its OSR right), so the first half-bit
//! on the wire sits in the least significant bits of each word.
//!
//! # Manchester polarity
//!
//! Derived from upstream's table rather than from memory: a `0` bit is HIGH then LOW, a `1` bit is
//! LOW then HIGH, and bits leave a byte least-significant first as Ethernet requires.

/// A line state, which doubles as an address in the PIO program.
pub const SYMBOL_IDLE: u32 = 0b00;
/// Differential low.
pub const SYMBOL_LOW: u32 = 0b01;
/// Differential high.
pub const SYMBOL_HIGH: u32 = 0b10;

/// Preamble (7 × `0x55`) and start-frame delimiter (`0xD5`), which precede every frame and are not
/// covered by the FCS.
pub const PREAMBLE_SFD: [u8; 8] = [0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0xD5];

/// End-of-transmission (TP_IDL): six half-bits of HIGH — 300 ns, over the 802.3 minimum — then
/// idle. Sent once after the FCS.
pub const TP_IDL_WORD: u32 = 0x0000_0AAA;

/// A normal link pulse: 100 ns of HIGH, then idle. With nothing else to send, one of these every
/// [`NLP_INTERVAL_US`] is what keeps the far end's link light on.
pub const NLP_WORD: u32 = 0x0000_000A;

/// Interval between link pulses when idle (802.3 specifies 16 ms ± tolerance).
pub const NLP_INTERVAL_US: u32 = 16_000;

/// Symbol words emitted per frame byte: 8 bits × 2 half-bits × 2 bits = 32 bits.
pub const WORDS_PER_BYTE: usize = 1;

/// Manchester-encode one byte into eight half-bit pairs, packed two bits per symbol, LSB first.
///
/// Computed rather than looked up. Upstream ships a 256-entry `tbl_manchester`, and the tests here
/// check this against entries read out of it; see `tests/bench_host.rs` for what the 1 KB of flash
/// would actually buy.
pub const fn manchester_word(byte: u8) -> u32 {
    let mut word = 0u32;
    let mut bit = 0;
    while bit < 8 {
        // A bit becomes one nibble: two 2-bit symbols, first-on-the-wire in the low half.
        //   0 -> HIGH(0b10) then LOW(0b01)  = 0b0110 = 0x6
        //   1 -> LOW(0b01)  then HIGH(0b10) = 0b1001 = 0x9
        let nibble = if byte & (1 << bit) != 0 { 0x9 } else { 0x6 };
        // Bit 0 leaves first, so it occupies the least significant nibble.
        word |= nibble << (4 * bit);
        bit += 1;
    }
    word
}

/// Number of symbol words [`encode_frame`] writes for a frame of `frame_len` bytes.
pub const fn encoded_words(frame_len: usize) -> usize {
    PREAMBLE_SFD.len() + frame_len + 1 // + TP_IDL
}

/// Encode a complete Ethernet frame (as built by [`crate::frame::build_udp_frame`], FCS included)
/// into the symbol words the PIO plays back: preamble, SFD, the frame, then TP_IDL.
///
/// Returns the number of words written, or `None` if `out` is too small.
pub fn encode_frame(frame: &[u8], out: &mut [u32]) -> Option<usize> {
    let n = encoded_words(frame.len());
    if out.len() < n {
        return None;
    }
    let mut i = 0;
    // The preamble and SFD are line coding, not frame content: they precede the FCS-covered bytes
    // and exist only to let the receiver's clock recovery lock on.
    for b in PREAMBLE_SFD {
        out[i] = manchester_word(b);
        i += 1;
    }
    for &b in frame {
        out[i] = manchester_word(b);
        i += 1;
    }
    // One TP_IDL closes the transmission and hands the line back to idle.
    out[i] = TP_IDL_WORD;
    Some(i + 1)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Extract the `n`th 2-bit symbol from a word, in transmission order.
    fn symbol(word: u32, n: usize) -> u32 {
        (word >> (2 * n)) & 0b11
    }

    // --- Manchester encoding, checked against upstream's generated table ---
    //
    // These expectations are read from `tbl_manchester` in kingyoPiyo/Pico-10BASE-T's udp.c, i.e.
    // from the reference implementation this is a port of, not from anyone's recollection of how
    // Manchester coding goes.

    #[test]
    fn a_zero_bit_is_high_then_low() {
        let w = manchester_word(0x00);
        assert_eq!(w, 0x6666_6666, "upstream tbl_manchester[0x00]");
        assert_eq!(symbol(w, 0), SYMBOL_HIGH);
        assert_eq!(symbol(w, 1), SYMBOL_LOW);
    }

    #[test]
    fn a_one_bit_is_low_then_high() {
        let w = manchester_word(0xFF);
        assert_eq!(w, 0x9999_9999, "upstream tbl_manchester[0xFF]");
        assert_eq!(symbol(w, 0), SYMBOL_LOW);
        assert_eq!(symbol(w, 1), SYMBOL_HIGH);
    }

    #[test]
    fn bits_leave_a_byte_least_significant_first() {
        // 0x01 differs from 0x00 only in bit 0, and that difference must land in the *first*
        // half-bit pair on the wire, i.e. the low nibble.
        assert_eq!(manchester_word(0x01), 0x6666_6669);
        // 0x02 sets bit 1, so the change moves one nibble up.
        assert_eq!(manchester_word(0x02), 0x6666_6696);
        assert_eq!(manchester_word(0x03), 0x6666_6699);
    }

    #[test]
    fn matches_upstream_for_the_preamble_and_sfd_bytes() {
        // The two bytes every frame actually starts with.
        assert_eq!(manchester_word(0x55), 0x6969_6969, "preamble byte");
        assert_eq!(manchester_word(0xD5), 0x9969_6969, "SFD");
    }

    #[test]
    fn never_emits_the_unused_symbol() {
        // 0b11 is not an address in the PIO program; emitting one would jump the state machine out
        // of its own three instructions.
        for b in 0..=255u8 {
            let w = manchester_word(b);
            for n in 0..16 {
                assert_ne!(symbol(w, n), 0b11, "byte {b:#04x} symbol {n}");
            }
        }
    }

    #[test]
    fn every_bit_period_has_a_mid_bit_transition() {
        // That property is the whole point of Manchester coding — it is what carries the clock.
        for b in 0..=255u8 {
            let w = manchester_word(b);
            for bit in 0..8 {
                let first = symbol(w, bit * 2);
                let second = symbol(w, bit * 2 + 1);
                assert_ne!(first, second, "byte {b:#04x} bit {bit} has no transition");
                assert!(first == SYMBOL_HIGH || first == SYMBOL_LOW);
                assert!(second == SYMBOL_HIGH || second == SYMBOL_LOW);
            }
        }
    }

    // --- Framing signals ---

    #[test]
    fn tp_idl_is_a_300ns_high_pulse_then_idle() {
        // 802.3 wants at least 250 ns; six 50 ns half-bits gives 300 ns.
        for n in 0..6 {
            assert_eq!(symbol(TP_IDL_WORD, n), SYMBOL_HIGH, "symbol {n}");
        }
        for n in 6..16 {
            assert_eq!(symbol(TP_IDL_WORD, n), SYMBOL_IDLE, "symbol {n}");
        }
    }

    #[test]
    fn nlp_is_a_100ns_high_pulse_then_idle() {
        assert_eq!(symbol(NLP_WORD, 0), SYMBOL_HIGH);
        assert_eq!(symbol(NLP_WORD, 1), SYMBOL_HIGH);
        for n in 2..16 {
            assert_eq!(symbol(NLP_WORD, n), SYMBOL_IDLE, "symbol {n}");
        }
    }

    // --- Whole-frame encoding ---

    #[test]
    fn encoded_length_is_preamble_plus_frame_plus_tp_idl() {
        assert_eq!(encoded_words(94), 8 + 94 + 1);
    }

    #[test]
    fn encode_frame_writes_preamble_then_frame_then_tp_idl() {
        let frame = [0x01u8, 0x02, 0x03];
        let mut out = [0u32; 16];
        let n = encode_frame(&frame, &mut out).expect("buffer is big enough");
        assert_eq!(n, encoded_words(frame.len()));

        for (i, b) in PREAMBLE_SFD.iter().enumerate() {
            assert_eq!(out[i], manchester_word(*b), "preamble word {i}");
        }
        for (i, b) in frame.iter().enumerate() {
            assert_eq!(
                out[PREAMBLE_SFD.len() + i],
                manchester_word(*b),
                "frame {i}"
            );
        }
        assert_eq!(out[n - 1], TP_IDL_WORD, "TP_IDL closes the transmission");
    }

    #[test]
    fn encode_frame_refuses_a_buffer_that_is_too_small() {
        let mut out = [0u32; 11];
        assert_eq!(encode_frame(&[0u8; 3], &mut out), None);
    }

    #[test]
    fn a_frame_starts_low_because_the_preamble_starts_with_a_one_bit() {
        // 0x55 has bit 0 set, so the very first half-bit a receiver sees is LOW. Getting this
        // inverted would still look like a valid Manchester stream on a scope, and would still be
        // undecodable by every receiver on the segment.
        let mut out = [0u32; 16];
        encode_frame(&[0u8; 3], &mut out).unwrap();
        assert_eq!(symbol(out[0], 0), SYMBOL_LOW);
        assert_eq!(symbol(out[0], 1), SYMBOL_HIGH);
    }
}
