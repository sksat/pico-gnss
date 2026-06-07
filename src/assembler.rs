//! UART バイト列から NMEA センテンスを 1 行ずつ組み立てる。
//!
//! `nmea` クレートの `parse` は完結した 1 行の `&str` を要求するため、その前段として
//! バイトストリームを 1 センテンスに切り出す層が要る。`$` で始まり CR/LF で終わる
//! 1 センテンスを返す。途中で `$` が来たら再同期し、バッファ溢れは破棄する。
//! 返すスライスは先頭 `$` を含み、末尾の CR/LF は含まない。チェックサム検証は
//! ここではやらず、`nmea` クレート側に任せる。

use heapless::Vec;

/// NMEA 0183 の 1 センテンス最大長 (CR/LF 込みで 82 文字)。余裕を見て 82 を確保。
pub const MAX_SENTENCE_LEN: usize = 82;

/// UART バイトを feed して完全な NMEA センテンスを組み立てる state machine。
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

    /// 1 バイト feed する。完全なセンテンス (`$`〜CR/LF 直前) が揃ったら
    /// その内容を `Some(&[u8])` で返す。返り値は次の `push` まで有効。
    pub fn push(&mut self, byte: u8) -> Option<&[u8]> {
        match byte {
            b'$' => {
                // 新しいセンテンス開始。途中だった内容は破棄して再同期。
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
                    // 既に emit 済み、または '$' のみ。終端は捨てる。
                    self.in_sentence = false;
                    None
                }
            }
            _ => {
                if self.in_sentence {
                    if self.buf.push(byte).is_err() {
                        // バッファ溢れ: このセンテンスは破棄して次の '$' を待つ。
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

    /// 入力バイト列を全て feed し、emit されたセンテンスを String の Vec で返す。
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
        // 一部の受信機は LF のみ。寛容に扱う。
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
        // 途中で '$' が来たら前の partial を捨てて新しいセンテンスを組む。
        let got = feed_all(b"$GPGGA,partial-no-term$GPRMC,full*00\r\n");
        assert_eq!(got, ["$GPRMC,full*00"]);
    }

    #[test]
    fn buffer_overflow_dropped_then_resync() {
        // MAX を超える終端なしデータは破棄され、次の '$' から再同期する。
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
