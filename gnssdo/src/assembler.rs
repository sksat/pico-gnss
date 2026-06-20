//! Reassemble NMEA sentences, one line at a time, from a UART byte stream.
//!
//! Parsers (the built-in helpers, or the `nmea` crate) expect one complete sentence
//! as a `&str`, so a framing layer is needed in front to cut the byte stream into
//! individual sentences. Returns one sentence that starts with `$` and ends at CR/LF.
//! A mid-line `$` resynchronizes; an overflowing buffer is dropped. The returned slice
//! includes the leading `$` and excludes the trailing CR/LF. Checksum validation is not
//! done here; it is left to the parser layer.

use heapless::Vec;

/// Maximum length of one NMEA 0183 sentence (82 chars including CR/LF). Reserve 82.
pub const MAX_SENTENCE_LEN: usize = 82;

/// State machine that is fed UART bytes and assembles complete NMEA sentences.
#[derive(Debug, Default)]
pub struct NmeaLineAssembler {
    buf: Vec<u8, MAX_SENTENCE_LEN>,
    in_sentence: bool,
}

impl NmeaLineAssembler {
    pub const fn new() -> Self {
        Self {
            buf: Vec::new(),
            in_sentence: false,
        }
    }

    /// Feed one byte. When a complete sentence (`$` up to just before CR/LF) is ready,
    /// returns its contents as `Some(&[u8])`. The return value is valid until the next `push`.
    pub fn push(&mut self, byte: u8) -> Option<&[u8]> {
        match byte {
            b'$' => {
                // Start of a new sentence; discard any in-progress content and resync.
                self.buf.clear();
                let _ = self.buf.push(b'$');
                self.in_sentence = true;
                None
            }
            b'\r' | b'\n' => {
                if self.in_sentence && self.buf.len() > 1 {
                    self.in_sentence = false;
                    Some(&self.buf[..])
                } else {
                    // Already emitted, or only '$'. Drop the terminator.
                    self.in_sentence = false;
                    None
                }
            }
            _ => {
                if self.in_sentence {
                    if self.buf.push(byte).is_err() {
                        // Buffer overflow: drop this sentence and wait for the next '$'.
                        self.buf.clear();
                        self.in_sentence = false;
                    }
                }
                None
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Feed all input bytes and return the emitted sentences as a Vec of Strings.
    fn feed_all(input: &[u8]) -> std::vec::Vec<std::string::String> {
        let mut asm = NmeaLineAssembler::new();
        let mut out = std::vec::Vec::new();
        for &b in input {
            if let Some(sentence) = asm.push(b) {
                out.push(std::string::String::from_utf8(sentence.to_vec()).unwrap());
            }
        }
        out
    }

    #[test]
    fn single_sentence_crlf() {
        let got = feed_all(b"$GPGGA,123519,4807.038,N*47\r\n");
        assert_eq!(got, ["$GPGGA,123519,4807.038,N*47"]);
    }

    #[test]
    fn lf_only_terminator() {
        // Some receivers send LF only; handle it leniently.
        let got = feed_all(b"$GPGLL,4916.45,N*7C\n");
        assert_eq!(got, ["$GPGLL,4916.45,N*7C"]);
    }

    #[test]
    fn two_sentences() {
        let got = feed_all(b"$GPGGA,1*00\r\n$GPRMC,2*00\r\n");
        assert_eq!(got, ["$GPGGA,1*00", "$GPRMC,2*00"]);
    }

    #[test]
    fn garbage_before_dollar_ignored() {
        let got = feed_all(b"\x00\xffxyz$GPVTG,054.7,T*10\r\n");
        assert_eq!(got, ["$GPVTG,054.7,T*10"]);
    }

    #[test]
    fn resync_on_new_dollar() {
        // A mid-line '$' discards the previous partial and starts a new sentence.
        let got = feed_all(b"$GPGGA,partial-no-term$GPRMC,full*00\r\n");
        assert_eq!(got, ["$GPRMC,full*00"]);
    }

    #[test]
    fn buffer_overflow_dropped_then_resync() {
        // Unterminated data beyond MAX is dropped, then resync from the next '$'.
        let mut input = std::vec::Vec::new();
        input.extend_from_slice(b"$");
        input.extend(core::iter::repeat(b'A').take(MAX_SENTENCE_LEN + 10));
        input.extend_from_slice(b"$GPGGA,ok*00\r\n");
        let got = feed_all(&input);
        assert_eq!(got, ["$GPGGA,ok*00"]);
    }

    #[test]
    fn bare_dollar_then_terminator_emits_nothing() {
        let got = feed_all(b"$\r\n");
        assert!(got.is_empty());
    }
}
