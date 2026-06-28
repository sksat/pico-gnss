//! 実験C: **温度連続モデルの holdover-endpoint 検証 (実ログ replay)**。
//!
//! 自己クロック=水晶周波数は連続量で、PPS が切れても温度で drift し続ける。温度センサは holdover 中も生きて
//! いるので、最後の直測 level に「温度変化分」を連続的に足して外挿できる。ここでは holdover を H 秒入れたときの
//! 終端誤差 E(H) を **level-only / slope-aware / 温度連続** で比較する。終端誤差は H とともに受信ノイズ床の
//! 上に出るので、出力もオシロも同じ受信機基準という計測トラップを外せる稀な軸 (受信機相対でない)。
//!
//! **実験C の核心**: 内蔵 ADC を高速オーバーサンプリングして fractional 化 (新 firmware の temp_raw=avg×8) すると
//! 温度 FF の holdover が改善するか。同一ログ上で温度を `GNSSDO_TEMP_QUANTIZE` で **fractional のまま (=1)** と
//! **integer-raw 相当に再量子化 (=8、×8 fractional を 8 刻みに丸め)** して比べれば、受信エポックを揃えたまま
//! 「fractional ADC の効果だけ」を分離できる。
//!
//!   GNSSDO_REPLAY_LOG=$PWD/logs/pps-expB-s2-ffon.log \
//!     cargo test -p gnssdo --test temp_holdover -- --ignored holdover_endpoint_temp --nocapture
//! env: GNSSDO_TEMP_QUANTIZE(既定1) GNSSDO_TEMP_SM(4) GNSSDO_TEMP_SHIFT(7) GNSSDO_TEMP_GAIN(64, matched-lead では未使用)
//!      GNSSDO_TEMP_LAG(1536=τ/Ts×256≈6s) GNSSDO_TEMP_DLEAD(2) GNSSDO_TEMP_OBS(2=K0.25)

use gnssdo::{DisciplinedClock, DisciplinedClockConfig};

/// defmt ログ1行から `key=<int>` を取り出す。
fn parse_kv(line: &str, key: &str) -> Option<i64> {
    let tok = line.split_whitespace().find(|t| t.starts_with(key))?;
    tok[key.len()..].parse().ok()
}

#[test]
#[ignore = "real-data replay; needs GNSSDO_REPLAY_LOG=<defmt log with temp_raw>"]
fn holdover_endpoint_temp_aware() {
    let Ok(path) = std::env::var("GNSSDO_REPLAY_LOG") else {
        eprintln!("TEMP-HOLDOVER skipped: set GNSSDO_REPLAY_LOG=<defmt log path>");
        return;
    };
    let env = |k: &str, d: i64| -> i64 {
        std::env::var(k).ok().and_then(|s| s.parse().ok()).unwrap_or(d)
    };
    let (quant, sm, sh, gq8) = (
        env("GNSSDO_TEMP_QUANTIZE", 1).max(1),
        env("GNSSDO_TEMP_SM", 4) as u32,
        env("GNSSDO_TEMP_SHIFT", 7) as u32,
        env("GNSSDO_TEMP_GAIN", 64),
    );
    // matched-lead + 残差 observer の新ノブ (掃引用)。既定は DisciplinedClockConfig::DEFAULT に一致。
    let (lag_q8, dlead, obs) = (
        env("GNSSDO_TEMP_LAG", 1536),
        env("GNSSDO_TEMP_DLEAD", 2) as u32,
        env("GNSSDO_TEMP_OBS", 2) as u32,
    );
    let text = std::fs::read_to_string(&path).expect("read log");
    // (interval, temp) ペア列。interval は "PPS count=" 行、temp は "PPSGEN count=" 行 (1:1 interleave)。
    let mut ivs: Vec<i64> = Vec::new();
    let mut temps: Vec<i64> = Vec::new();
    let mut last_iv: Option<i64> = None;
    for l in text.lines() {
        if l.contains("] PPS count=") {
            if let Some(iv) = parse_kv(l, "interval_ns=") {
                if (iv - 1_000_000_000).abs() < 1_000_000 {
                    last_iv = Some(iv);
                }
            }
        } else if l.contains("PPSGEN count=") {
            if let (Some(iv), Some(t)) = (last_iv, parse_kv(l, "temp_raw=")) {
                ivs.push(iv);
                // 再量子化: Q 刻みに丸める (Q=1 で fractional そのまま、Q=8 で integer-raw 相当)。
                temps.push(((t + quant / 2) / quant) * quant);
            }
        }
    }
    let n = ivs.len();
    assert!(n > 1000, "not enough (interval,temp) pairs: {n}");
    let mut cap = vec![0i64; n + 1];
    for i in 0..n {
        cap[i + 1] = cap[i] + ivs[i];
    }
    let rms = |v: &[f64]| (v.iter().map(|x| x * x).sum::<f64>() / v.len().max(1) as f64).sqrt();
    // 温度 FF 有効のクロックを作る (config は Copy でないので毎回構築)。
    let mk = || {
        DisciplinedClock::with_config(DisciplinedClockConfig {
            temp_ff_enable: true,
            temp_ff_sm_shift: sm,
            temp_ff_shift: sh,
            temp_ff_gain_q8: gq8, // matched-lead path では未使用 (sweep 互換のため受ける)
            temp_ff_lag_q8: lag_q8,
            temp_ff_dlead_shift: dlead,
            resid_obs_shift: obs,
            ..DisciplinedClockConfig::DEFAULT
        })
    };
    let warmup = 200usize;
    eprintln!(
        "TEMP-HOLDOVER replay: n={n} temp_span={} quantize={quant} sm={sm} shift={sh} \
         gain_q8={gq8} lag_q8={lag_q8} dlead={dlead} obs={obs}",
        temps.iter().max().unwrap() - temps.iter().min().unwrap()
    );
    eprintln!("  H[s]   n   level |E| rms    slope |E| rms    temp |E| rms   (ns)   k");
    for &h in &[10i64, 30, 100, 300, 600] {
        let hh = h as usize;
        let (mut el, mut es, mut et) = (Vec::new(), Vec::new(), Vec::new());
        let mut k_seen = 0i64;
        let stride = (hh / 2).max(20);
        let mut i = warmup;
        while i + hh < n {
            let mut c = mk();
            for j in 0..i {
                c.update_temp(temps[j]);
                c.update_freq(ivs[j]);
            }
            k_seen = c.temp_k_mppb_per_unit();
            c.update_epoch(cap[i] as u64, cap[i] as u64, i as i64 * 1_000_000_000);
            let cap_end = cap[i + hh] as u64;
            let unix_end = (i + hh) as i64 * 1_000_000_000;
            let (le, se) = match (
                c.prediction_residual_ns(cap_end, unix_end),
                c.prediction_residual_holdover_ns(cap_end, unix_end),
            ) {
                (Some(a), Some(b)) => (a, b),
                _ => {
                    i += stride;
                    continue;
                }
            };
            // 温度連続: hold 中 1 秒ごとに温度を与えながら predicted_freq を実 local 経過に積分。
            let mut utc_temp = 0i128;
            for j in i..i + hh {
                c.update_temp(temps[j]);
                let f = c.predicted_freq_mppb();
                utc_temp += ivs[j] as i128 - (ivs[j] as i128 * f as i128 / 1_000_000_000_000i128);
            }
            let te = (utc_temp - hh as i128 * 1_000_000_000) as i64;
            el.push(le.abs() as f64);
            es.push(se.abs() as f64);
            et.push(te.abs() as f64);
            i += stride;
        }
        if !el.is_empty() {
            eprintln!(
                "  {:4}  {:3}   {:10.0}    {:10.0}    {:10.0}    {}",
                h,
                el.len(),
                rms(&el),
                rms(&es),
                rms(&et),
                k_seen
            );
        }
    }
    // 健全性: テストが走ったこと (合否は数値レポートで人が判断する診断 test)。
    assert!(n > 1000);
}
