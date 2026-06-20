//! PPS によるクロック規律 (time sync)。
//!
//! GNSS の 1PPS 立ち上がりは UTC 秒境界に一致する。その瞬間の local timer 値
//! (RP2040 TIMER, 1µs 分解能) を、NMEA から得たその秒の UTC と対応付けることで、
//! device 上に µs 精度の UTC エポックを保持する。
//!
//! **なぜ firmware でやるか**: host 側で RTT/USB 経由に同期すると probe/USB の
//! 往復ジッタ (数十 ms) が乗り、PPS 本来の精度が失われる。PPS エッジ↔UTC 秒の
//! 対応付けは、エッジを µs で刻める MCU 上で行うのが必須。
//!
//! いずれも HAL 非依存の純粋ロジックなので host で `cargo test-host` する。

/// 1970-01-01 からの経過日数 (Howard Hinnant のアルゴリズム, proleptic Gregorian)。
pub const fn days_from_civil(y: i64, m: i64, d: i64) -> i64 {
    let y = if m <= 2 { y - 1 } else { y };
    let era = (if y >= 0 { y } else { y - 399 }) / 400;
    let yoe = y - era * 400; // [0, 399]
    let doy = (153 * (if m > 2 { m - 3 } else { m + 9 }) + 2) / 5 + d - 1; // [0, 365]
    let doe = yoe * 365 + yoe / 4 - yoe / 100 + doy; // [0, 146096]
    era * 146097 + doe - 719468
}

/// UTC civil 日時 → Unix 秒 (閏秒は無視)。
pub const fn civil_to_unix(y: i64, mo: i64, d: i64, h: i64, mi: i64, s: i64) -> i64 {
    days_from_civil(y, mo, d) * 86400 + h * 3600 + mi * 60 + s
}

/// NMEA の時刻フィールド `hhmmss.sss` から (時,分,秒) を取り出す。小数秒は捨てる
/// (PPS 境界なので秒は整数)。
pub fn parse_hhmmss(field: &str) -> Option<(u8, u8, u8)> {
    let int_part = field.split('.').next().unwrap_or(field);
    if int_part.len() < 6 {
        return None;
    }
    let h: u8 = int_part.get(0..2)?.parse().ok()?;
    let mi: u8 = int_part.get(2..4)?.parse().ok()?;
    let s: u8 = int_part.get(4..6)?.parse().ok()?;
    if h > 23 || mi > 59 || s > 60 {
        return None;
    }
    Some((h, mi, s))
}

/// NMEA の日付フィールド `ddmmyy` (RMC) から (日,月,西暦年) を取り出す。年は 20xx 前提。
pub fn parse_ddmmyy(field: &str) -> Option<(u8, u8, u16)> {
    if field.len() < 6 {
        return None;
    }
    let d: u8 = field.get(0..2)?.parse().ok()?;
    let mo: u8 = field.get(2..4)?.parse().ok()?;
    let yy: u16 = field.get(4..6)?.parse().ok()?;
    if d < 1 || d > 31 || mo < 1 || mo > 12 {
        return None;
    }
    Some((d, mo, 2000 + yy))
}

/// RMC センテンス文字列から `((時, 分, 秒), (日, 月, 西暦年))` を取り出す。RMC 以外・解析失敗は `None`。
///
/// 入力は assembler でフレーム済みの NMEA 1 文 (`$` + 2 文字 talker、例 `$GPRMC,...` `$GNRMC,...`) を想定。
///
/// default は自前パーサ。feature `nmea` を有効にすると [`nmea`](https://docs.rs/nmea) crate に委譲する。
/// バックエンドで挙動が次のように異なる:
/// - **checksum**: 自前=**非検証** / nmea=**検証** (不一致は `None`)。
/// - **年**: 自前=**20xx 固定** (`2000+yy`) / nmea=**世紀ピボット** (`yy=94`→1994)。
/// - **閏秒 `ss=60`**: 自前=**受理** (civil_to_unix が次分へ繰り上げ) / nmea=**拒否** (`None`)。
/// - **速度/サイズ**: nmea は実機 RP2040 で約 **17x 遅く・firmware `.text` 約 +52KB** (1Hz では無視可)。
#[cfg(not(feature = "nmea"))]
pub fn parse_rmc_time_date(sentence: &str) -> Option<((u8, u8, u8), (u8, u8, u16))> {
    if sentence.get(3..6) != Some("RMC") {
        return None;
    }
    let time = sentence.split(',').nth(1).and_then(parse_hhmmss)?;
    let date = sentence.split(',').nth(9).and_then(parse_ddmmyy)?;
    Some((time, date))
}

/// RMC から `((時,分,秒),(日,月,年))` を取り出す (nmea crate 版)。詳細は default 版の doc 参照。
/// `nmea::parse_str` は **checksum を検証**してから RMC を取り出す (不一致は `None`)。
#[cfg(feature = "nmea")]
pub fn parse_rmc_time_date(sentence: &str) -> Option<((u8, u8, u8), (u8, u8, u16))> {
    use chrono::{Datelike, Timelike};
    use nmea::ParseResult;
    // parse_str は checksum 検証込みで sentence を解析し、RMC を ParseResult::RMC で返す。
    let rmc = match nmea::parse_str(sentence).ok()? {
        ParseResult::RMC(rmc) => rmc,
        _ => return None,
    };
    let t = rmc.fix_time?;
    let d = rmc.fix_date?;
    Some((
        (t.hour() as u8, t.minute() as u8, t.second() as u8),
        (d.day() as u8, d.month() as u8, d.year() as u16),
    ))
}

/// 確立した同期点: PPS エッジの local 時刻 ↔ その UTC 秒。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SyncPoint {
    /// PPS エッジを刻んだ local timer 値 (µs)。
    pub pps_local_us: u64,
    /// その PPS エッジが指す UTC 秒 (Unix 秒)。
    pub unix_s: i64,
    /// 直近 PPS 間隔の理想 1s からのずれ (= local オシレータの drift, µs)。
    pub drift_us: i64,
}

/// PPS 立ち上がりエッジと、その後に届く NMEA 時刻文 (RMC/ZDA 等) が指す秒の対応。
///
/// PPS のエッジは UTC 秒境界そのものを刻むが、その秒を表す NMEA 時刻文がエッジの
/// **同じ秒 / 1 つ前 / 1 つ後** のどれを指すかは受信機依存。ずれても必ず **±1 秒以内**
/// (エッジと NMEA 文の前後関係の問題で、2 秒以上ずれる正常動作は無い)。ここを誤ると確立する
/// UTC エポックが丸ごと 1 秒ずれる — 本ライブラリ最大の事故要因なので、取りうる 3 値を列挙して
/// 型で閉じている (任意秒オフセットは正常系に存在しないので受け付けない)。
///
/// (`Debug`/`Default` は親 [`PpsTimeSync`] の derive が要求する最小限。)
#[derive(Debug, Default)]
pub enum PpsNmeaAssociation {
    /// NMEA 時刻文はエッジと同じ秒を指す (大半の受信機)。既定。
    #[default]
    SameSecond,
    /// NMEA 時刻文はエッジの 1 つ前の秒を指す (エッジは NMEA より 1 秒後)。
    NmeaIsPreviousSecond,
    /// NMEA 時刻文はエッジの 1 つ次の秒を指す (エッジは NMEA より 1 秒前)。
    NmeaIsNextSecond,
}

impl PpsNmeaAssociation {
    /// エッジの UTC 秒 = (NMEA 秒) + この補正 (±1 or 0)。
    const fn edge_offset_seconds(&self) -> i64 {
        match self {
            Self::SameSecond => 0,
            Self::NmeaIsPreviousSecond => 1,
            Self::NmeaIsNextSecond => -1,
        }
    }
}

/// PPS エッジと NMEA 時刻を対応付けてクロックを規律する state machine。
#[derive(Debug, Default)]
pub struct PpsTimeSync {
    association: PpsNmeaAssociation,
    last_date: Option<(u16, u8, u8)>, // (year, month, day)
    pending_pps_us: Option<u64>,      // まだ UTC とペアにしていない直近 PPS エッジ
    last_pps_us: Option<u64>,         // drift 計算用の前回 PPS エッジ
    epoch_local_us: Option<u64>,      // 確立エポック: local 基準
    epoch_unix_s: Option<i64>,        // 確立エポック: UTC 秒
    last_drift_us: i64,
}

impl PpsTimeSync {
    /// 既定 ([`PpsNmeaAssociation::SameSecond`]) で生成する。
    pub const fn new() -> Self {
        Self::with_association(PpsNmeaAssociation::SameSecond)
    }

    /// PPS↔NMEA の秒対応を指定して生成する。`static` 初期化で使えるよう `const fn`。
    pub const fn with_association(association: PpsNmeaAssociation) -> Self {
        Self {
            association,
            last_date: None,
            pending_pps_us: None,
            last_pps_us: None,
            epoch_local_us: None,
            epoch_unix_s: None,
            last_drift_us: 0,
        }
    }

    /// PPS 立ち上がりエッジを local 時刻 `local_us` で記録する。
    /// 前回エッジがあれば drift (間隔 - 1s, µs) を返す。
    pub fn on_pps(&mut self, local_us: u64) -> Option<i64> {
        let drift = self
            .last_pps_us
            .map(|prev| local_us as i64 - prev as i64 - 1_000_000);
        if let Some(d) = drift {
            self.last_drift_us = d;
        }
        self.last_pps_us = Some(local_us);
        self.pending_pps_us = Some(local_us);
        drift
    }

    /// RMC/ZDA から日付を更新する。
    pub fn set_date(&mut self, year: u16, month: u8, day: u8) {
        self.last_date = Some((year, month, day));
    }

    /// NMEA の時刻 (h,mi,s) を受け取り、直近 PPS エッジとペアにして同期点を確立する。
    /// 日付未取得 or PPS 未受信なら `None`。
    pub fn on_time(&mut self, h: u8, mi: u8, s: u8) -> Option<SyncPoint> {
        let (y, mo, d) = self.last_date?;
        let pps = self.pending_pps_us?;
        // PPS エッジが指す UTC 秒 = NMEA 秒 + 受信機依存補正 (±1 秒ズレ吸収)。
        let unix_s = civil_to_unix(y as i64, mo as i64, d as i64, h as i64, mi as i64, s as i64)
            + self.association.edge_offset_seconds();
        self.epoch_local_us = Some(pps);
        self.epoch_unix_s = Some(unix_s);
        self.pending_pps_us = None;
        Some(SyncPoint {
            pps_local_us: pps,
            unix_s,
            drift_us: self.last_drift_us,
        })
    }

    /// 同期が確立しているか。
    pub fn is_locked(&self) -> bool {
        self.epoch_local_us.is_some() && self.epoch_unix_s.is_some()
    }

    /// 任意の local timer 値に対する規律された UTC (Unix µs)。未同期なら `None`。
    pub fn now_unix_micros(&self, local_us: u64) -> Option<i64> {
        let el = self.epoch_local_us?;
        let eu = self.epoch_unix_s?;
        Some(eu * 1_000_000 + (local_us as i64 - el as i64))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn civil_to_unix_known_anchors() {
        assert_eq!(civil_to_unix(1970, 1, 1, 0, 0, 0), 0);
        assert_eq!(civil_to_unix(2000, 1, 1, 0, 0, 0), 946_684_800);
        // 2026-06-07T17:06:59Z (今回の実測 fix。`date -u -d ... +%s` で検証済み)
        assert_eq!(civil_to_unix(2026, 6, 7, 17, 6, 59), 1_780_852_019);
    }

    #[test]
    fn parse_time_ok() {
        assert_eq!(parse_hhmmss("170658.000"), Some((17, 6, 58)));
        assert_eq!(parse_hhmmss("000000.00"), Some((0, 0, 0)));
        assert_eq!(parse_hhmmss("235959"), Some((23, 59, 59)));
    }

    #[test]
    fn parse_time_rejects_garbage() {
        assert_eq!(parse_hhmmss(""), None);
        assert_eq!(parse_hhmmss("12345"), None); // 短すぎ
        assert_eq!(parse_hhmmss("99xxss"), None);
        assert_eq!(parse_hhmmss("250000"), None); // 時 > 23
    }

    #[test]
    fn parse_rmc_time_date_extracts_time() {
        let s = "$GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6A";
        let r = parse_rmc_time_date(s).unwrap();
        assert_eq!(r.0, (12, 35, 19)); // 時刻は両バックエンドで一致
        // 年解釈は異なる: 自前=20xx 固定 (2094)、nmea=世紀ピボット (1994)。
        #[cfg(not(feature = "nmea"))]
        assert_eq!(r.1, (23, 3, 2094));
        #[cfg(feature = "nmea")]
        assert_eq!(r.1, (23, 3, 1994));
    }

    #[test]
    fn parse_rmc_time_date_rejects_non_rmc() {
        let s = "$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47";
        assert_eq!(parse_rmc_time_date(s), None);
    }

    #[test]
    fn parse_rmc_time_date_checksum_behavior() {
        // checksum を改ざんした RMC (正は *6A、誤の *00)。
        let bad = "$GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*00";
        // 自前は checksum 非検証なので通る。nmea は parse_str が検証して弾く。
        #[cfg(not(feature = "nmea"))]
        assert!(parse_rmc_time_date(bad).is_some());
        #[cfg(feature = "nmea")]
        assert!(parse_rmc_time_date(bad).is_none());
    }

    #[test]
    fn parse_date_ok() {
        assert_eq!(parse_ddmmyy("070626"), Some((7, 6, 2026)));
        assert_eq!(parse_ddmmyy("311299"), Some((31, 12, 2099)));
    }

    #[test]
    fn parse_date_rejects_garbage() {
        assert_eq!(parse_ddmmyy("00xx26"), None);
        assert_eq!(parse_ddmmyy("001326"), None); // 月 13
        assert_eq!(parse_ddmmyy("12"), None);
    }

    #[test]
    fn drift_is_none_then_interval_error() {
        let mut ts = PpsTimeSync::new();
        assert_eq!(ts.on_pps(1_000_000), None); // 初回は前回が無い
        assert_eq!(ts.on_pps(2_000_050), Some(50)); // +50µs drift
        assert_eq!(ts.on_pps(2_999_940), Some(-110)); // -110µs
    }

    #[test]
    fn sync_requires_both_date_and_pps() {
        let mut ts = PpsTimeSync::new();
        // PPS だけ / 時刻だけでは確立しない
        assert!(ts.on_time(17, 6, 58).is_none());
        ts.on_pps(1_000_000);
        assert!(ts.on_time(17, 6, 58).is_none()); // 日付がまだ
        ts.set_date(2026, 6, 7);
        let sp = ts.on_time(17, 6, 58).unwrap();
        assert_eq!(sp.pps_local_us, 1_000_000);
        assert_eq!(sp.unix_s, civil_to_unix(2026, 6, 7, 17, 6, 58));
        assert!(ts.is_locked());
    }

    #[test]
    fn now_micros_interpolates_from_epoch() {
        let mut ts = PpsTimeSync::new();
        ts.set_date(2026, 6, 7);
        ts.on_pps(1_000_000);
        let sp = ts.on_time(17, 6, 58).unwrap();
        let base = sp.unix_s * 1_000_000;
        // エポックそのもの
        assert_eq!(ts.now_unix_micros(1_000_000), Some(base));
        // 0.5s 後
        assert_eq!(ts.now_unix_micros(1_500_000), Some(base + 500_000));
        // エポック前 (PPS より前の local 値)
        assert_eq!(ts.now_unix_micros(999_000), Some(base - 1_000));
    }

    #[test]
    fn association_nmea_next_second_shifts_epoch_back() {
        // 受信機が「次の秒」を先に送る: エッジの UTC 秒は NMEA 秒 - 1。
        let mut ts = PpsTimeSync::with_association(PpsNmeaAssociation::NmeaIsNextSecond);
        ts.set_date(2026, 6, 7);
        ts.on_pps(1_000_000);
        let sp = ts.on_time(17, 6, 58).unwrap();
        // 既定 (SameSecond) なら 17:06:58 だが、NmeaIsNextSecond で 17:06:57 に対応づく。
        assert_eq!(sp.unix_s, civil_to_unix(2026, 6, 7, 17, 6, 57));
        // now も同じエポックずれを反映する。
        assert_eq!(
            ts.now_unix_micros(1_000_000),
            Some(civil_to_unix(2026, 6, 7, 17, 6, 57) * 1_000_000)
        );
    }

    #[test]
    fn association_nmea_previous_second_shifts_epoch_forward() {
        // 受信機が「前の秒」を報告する: エッジの UTC 秒は NMEA 秒 + 1。
        let mut ts = PpsTimeSync::with_association(PpsNmeaAssociation::NmeaIsPreviousSecond);
        ts.set_date(2026, 6, 7);
        ts.on_pps(1_000_000);
        let sp = ts.on_time(17, 6, 58).unwrap();
        assert_eq!(sp.unix_s, civil_to_unix(2026, 6, 7, 17, 6, 59));
    }

    #[test]
    fn association_handles_day_rollover() {
        // 00:00:00 で NmeaIsNextSecond(-1) → 前日 23:59:59 (unix 秒加算で日跨ぎも正しい)。
        let mut ts = PpsTimeSync::with_association(PpsNmeaAssociation::NmeaIsNextSecond);
        ts.set_date(2026, 6, 7);
        ts.on_pps(5_000_000);
        let sp = ts.on_time(0, 0, 0).unwrap();
        assert_eq!(sp.unix_s, civil_to_unix(2026, 6, 6, 23, 59, 59));
    }

    #[test]
    fn association_handles_year_rollover() {
        // 2026-12-31 23:59:59 で NmeaIsPreviousSecond(+1) → 2027-01-01 00:00:00 (年跨ぎ)。
        let mut ts = PpsTimeSync::with_association(PpsNmeaAssociation::NmeaIsPreviousSecond);
        ts.set_date(2026, 12, 31);
        ts.on_pps(1_000_000);
        let sp = ts.on_time(23, 59, 59).unwrap();
        assert_eq!(sp.unix_s, civil_to_unix(2027, 1, 1, 0, 0, 0));
    }

    #[test]
    fn association_offset_does_not_accumulate() {
        // 連続同期で補正が累積しない (毎秒 +1 されず、両エポックに同じだけ効く)。
        let mut ts = PpsTimeSync::with_association(PpsNmeaAssociation::NmeaIsNextSecond);
        ts.set_date(2026, 6, 7);
        ts.on_pps(1_000_000);
        let s1 = ts.on_time(17, 6, 58).unwrap();
        ts.on_pps(2_000_000);
        let s2 = ts.on_time(17, 6, 59).unwrap();
        assert_eq!(s2.unix_s - s1.unix_s, 1);
        assert_eq!(s1.unix_s, civil_to_unix(2026, 6, 7, 17, 6, 57));
        assert_eq!(s2.unix_s, civil_to_unix(2026, 6, 7, 17, 6, 58));
    }

    #[test]
    fn sync_advances_each_second() {
        let mut ts = PpsTimeSync::new();
        ts.set_date(2026, 6, 7);
        ts.on_pps(1_000_000);
        let s1 = ts.on_time(17, 6, 58).unwrap();
        ts.on_pps(2_000_000);
        let s2 = ts.on_time(17, 6, 59).unwrap();
        assert_eq!(s2.unix_s - s1.unix_s, 1);
        assert_eq!(s2.pps_local_us, 2_000_000);
        assert_eq!(s2.drift_us, 0); // ちょうど 1s 間隔
    }
}
