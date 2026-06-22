//! 温度駆動の水晶プラントモデルに対する GPSDO 規律ループの性質試験。
//!
//! [`DisciplinedClock`] の α-β 周波数推定 (level+slope) は、温度ドリフト = 水晶周波数のランプに対する
//! 出力位相の定常誤差 (offset② のドリフト) を抑えるために入れた。ここではその制御を、**特定の基板の実測値
//! ではなく一般的に妥当な物理レンジ**で生成した多数の温度プロファイルに対して proptest で検証する
//! (gnssdo は汎用 GPSDO ライブラリなので、一台でなく広い水晶/環境を覆うのが目的)。
//!
//! 二層構成:
//! - 常時実行 (proptest): 有界 ramp + ジッタの任意プロファイルで「発散せずロックを保つ」頑健性と、
//!   単調ドリフトで「α-β が単一 EMA より悪化しない」優位性。**特定の実機ログの数値には依存しない**
//!   (入力は一般物理レンジ)。ただし検証対象は `*::DEFAULT` の tuning なので、厳密には「default tuning の
//!   物理レンジ回帰」であって「任意の GPSDO 制御則の保証」ではない。良好アンテナの 1PPS GPSDO 想定。
//! - `#[ignore]` のモデル検証 + 実ログ replay: プラント+モデルが実機の観測 (緩い warming で ~100ns、急な
//!   加熱で ~µs、replay 相関) を再現するかを確認する。観測値はこの基板固有なので常時は走らせない。
//!
//! プラントの式 `phase 前進 = 出力周期偏差 − 基準周期偏差` は定義的恒等で、実機ログでも per-step 残差 0。
//! ただしこれは**モデル式の整合性**を示すだけで、出力周期の量子化/dither、比較器遅延、GNSS 異常値、温度
//! ヒステリシスといった**未モデルの I/O は保証しない**。捕捉ジッタ/量子化は呼び出し側がレンジで与える
//! (特定 HW の 16ns をハードコードしない)。

use gnssdo::{DisciplinedClock, DisciplinedClockConfig, PhaseLockLoop, PhaseLockLoopConfig};
use proptest::prelude::*;

/// defmt ログ1行から `key=<int>` を取り出す (例 `interval_ns=1000002928`)。
fn parse_kv(line: &str, key: &str) -> Option<i64> {
    let tok = line.split_whitespace().find(|t| t.starts_with(key))?;
    tok[key.len()..].parse().ok()
}

/// 温度→水晶周波数は「滑らかに変化する量」。1 サンプル(=1s)ごとの真の周波数オフセット列 (mppb)。
struct Profile {
    f: Vec<i64>,
}

/// 区分線形の周波数プロファイル。`warmup` サンプルは f0 で平坦 (clock/PLL のロック確立用)、その後
/// 各区間で `ramps[i]` mppb/sample のランプ。総オフセットは ±100 ppm の物理レンジにクランプ。
fn build_profile(f0: i64, ramps: &[i64], seg_len: usize, warmup: usize) -> Profile {
    let mut f = vec![f0; warmup];
    let mut cur = f0;
    for &rate in ramps {
        for _ in 0..seg_len {
            cur = (cur + rate).clamp(-100_000_000, 100_000_000);
            f.push(cur);
        }
    }
    Profile { f }
}

/// 決定的な擬似ジッタ (±amp ns)。乱数 crate に頼らず index ハッシュで再現可能にする。
fn jitter(n: usize, amp: i64) -> i64 {
    if amp <= 0 {
        return 0;
    }
    let h = (n as u64)
        .wrapping_mul(2_654_435_761)
        .wrapping_add(1_013_904_223);
    (h % (2 * amp as u64 + 1)) as i64 - amp
}

struct SimStats {
    max_abs_phase_ns: i64,
    tail_mean_abs_phase_ns: i64,
    locked_end: bool,
}

/// 単一 EMA ベースライン (slope を 0 にクランプ = 傾き先回り無し) の設定。
fn ema_only_cfg() -> DisciplinedClockConfig {
    DisciplinedClockConfig {
        slope_max_mppb: 0,
        ..DisciplinedClockConfig::DEFAULT
    }
}

/// GPSDO 閉ループの離散シミュレーション。各 step:
/// 1. 真の水晶周波数 f[n] → 測定 PPS 間隔 (整数 ns 量子化 + 捕捉ジッタ) を [`DisciplinedClock`] に与える。
/// 2. 予測周波数 `predicted_freq_mppb()` + PLL の I トリムを「出力周期に入れる周波数」とする。
/// 3. 出力位相は (commanded - true_avg)/1000 - phase_corr で進む
///    (出力周期が真の 1 秒からずれた分が位相に積算、PLL の P+D が即時補正)。
/// 4. その位相 (loopback 捕捉相当 + ~16ns ジッタ) を次の PLL 入力にする。
fn simulate(
    prof: &Profile,
    clock_cfg: DisciplinedClockConfig,
    interval_jitter_ns: i64,
    cap_jitter_ns: i64,
) -> SimStats {
    let mut clock = DisciplinedClock::with_config(clock_cfg);
    let mut pll = PhaseLockLoop::new();
    let mut phase_ns: i64 = 0;
    let mut max_abs = 0i64;
    let (mut tail_sum, mut tail_cnt) = (0i64, 0i64);
    let n_total = prof.f.len();
    let tail_start = n_total.saturating_sub(30); // 末尾 30s を steady とみなす

    for n in 0..n_total {
        let true_f = prof.f[n];
        let interval_ns = 1_000_000_000 + true_f / 1000 + jitter(n, interval_jitter_ns);
        clock.update_freq(interval_ns);

        // 位相捕捉ジッタ/量子化 (ハード捕捉の分解能)。RP2040 PIO なら ~16ns だが、汎用 (他チップのタイマ
        // 入力キャプチャ等) を覆うため呼び出し側がレンジを与える (特定 HW の 16ns をハードコードしない)。
        let measured_phase = phase_ns + jitter(n + 7, cap_jitter_ns);
        let u = pll.update(measured_phase, true);
        let commanded = clock.predicted_freq_mppb() + u.freq_trim_mppb;

        let true_avg = if n + 1 < n_total {
            (true_f + prof.f[n + 1]) / 2
        } else {
            true_f
        };
        phase_ns += (commanded - true_avg) / 1000 - u.phase_corr_ns;

        let a = phase_ns.abs();
        max_abs = max_abs.max(a);
        if n >= tail_start {
            tail_sum += a;
            tail_cnt += 1;
        }
    }
    SimStats {
        max_abs_phase_ns: max_abs,
        tail_mean_abs_phase_ns: if tail_cnt > 0 { tail_sum / tail_cnt } else { 0 },
        locked_end: pll.is_locked(),
    }
}

proptest! {
    // 頑健性 (特定ログ非依存): 物理的に妥当な温度プロファイル (±20 ppb/s までの有界 ramp を最大 7 区間、
    // ±50ns 捕捉ジッタ) のどれに対しても、α-β 閉ループは発散せずロックを保ち、|phase| は generous な
    // 上限 (50µs) 未満に留まる。proptest はこれを破るプロファイルを探索する。
    #[test]
    fn alpha_beta_loop_stays_bounded(
        f0 in -30_000_000i64..30_000_000,
        ramps in prop::collection::vec(-20_000i64..20_000, 1..8),
        jit in 0i64..50,
        cap in 0i64..100,
    ) {
        // 安全性 (特定ログ非依存・default tuning の物理レンジ回帰): ±20 ppb/s までの有界 ramp を最大 7 区間
        // どう並べても、α-β 閉ループは発散しない (|phase| < 50µs)。捕捉ジッタ/量子化 cap は 0..100ns で振り、
        // 特定 HW の 16ns に依存しないことを確認。1µs lock 窓は速いランプ中に一時的に外れるのが正常なので、
        // ここでは「発散しない」ことだけを保証する (再ロックは別プロパティ)。
        let prof = build_profile(f0, &ramps, 20, 25);
        let s = simulate(&prof, DisciplinedClockConfig::DEFAULT, jit, cap);
        prop_assert!(s.max_abs_phase_ns < 50_000, "phase diverged under thermal stress: {}ns", s.max_abs_phase_ns);
    }

    // 回復性 (特定ログ非依存): 現実的な緩い片方向ドリフト (単調 ±5 ppb/s) のあと温度が落ち着けば、α-β は再ロック
    // して ns 級へ戻る (整定区間 80s)。激しい持続ランプ (>>5 ppb/s を数十秒) は別問題で、上の有界性のみ保証。
    #[test]
    fn alpha_beta_relocks_after_realistic_drift(
        f0 in -20_000_000i64..20_000_000,
        rate in -5_000i64..5_000,
    ) {
        let prof = build_profile(f0, &[rate, rate, 0, 0, 0, 0], 20, 25);
        let s = simulate(&prof, DisciplinedClockConfig::DEFAULT, 16, 16);
        prop_assert!(s.locked_end, "did not re-lock after realistic drift settled (max={}ns)", s.max_abs_phase_ns);
        prop_assert!(s.tail_mean_abs_phase_ns < 1_000, "settled phase not ns-order: {}ns", s.tail_mean_abs_phase_ns);
    }

    // 優位性 (特定ログ非依存): 温度が片方向に緩くドリフト (単調 ±0.2..5 ppb/s ランプ) する間、α-β の steady 位相は
    // 単一 EMA を上回らない (傾き先回りが定常ランプ誤差を消すので、悪化しないことを保証)。
    #[test]
    fn alpha_beta_not_worse_than_single_ema_on_monotone_drift(
        f0 in -20_000_000i64..20_000_000,
        rate in 200i64..5_000,
        positive in prop::bool::ANY,
    ) {
        let r = if positive { rate } else { -rate };
        let prof = build_profile(f0, &[r], 200, 25);
        let ab = simulate(&prof, DisciplinedClockConfig::DEFAULT, 0, 0);
        let ema = simulate(&prof, ema_only_cfg(), 0, 0);
        prop_assert!(ab.locked_end && ema.locked_end);
        // +50ns は微小ランプでの整数丸めマージン。
        prop_assert!(
            ab.tail_mean_abs_phase_ns <= ema.tail_mean_abs_phase_ns + 50,
            "α-β worse than EMA on monotone drift: ab={}ns ema={}ns rate={}mppb/s",
            ab.tail_mean_abs_phase_ns, ema.tail_mean_abs_phase_ns, r
        );
    }
}

// --- モデル検証 (常時は走らせない。`cargo test -p gnssdo --test thermal_plant -- --ignored`) ---
// プラント+単一EMA モデルが、この基板の実機観測を桁で再現するかを確認する。観測値は HW 固有なので
// 一般性質の proptest とは分離し #[ignore] にする (モデルの妥当性が取れて初めて proptest の結論を信じられる)。

#[test]
#[ignore = "model-fidelity check against observed hardware values (HW-specific); run manually"]
fn model_reproduces_observed_drift() {
    // 現実的 warming = 指数接近 f(t)=A(1-e^{-t/τ})。レートが時間で変化 (周波数加速度) するのが要点で、
    // 一定レートのランプでは PLL の積分が吸収して定常 ~0 になってしまい、観測の µs を再現できない。
    let warming = |a: f64, tau: f64, n: usize, settle: usize| -> Profile {
        let mut f = vec![0i64; 25];
        for k in 0..n {
            f.push((a * (1.0 - (-(k as f64) / tau).exp())) as i64);
        }
        let last = *f.last().unwrap();
        f.extend(core::iter::repeat(last).take(settle));
        Profile { f }
    };

    // 観測1: 緩い warming ~1 ppb/s 相当のとき hwphase は ~100ns 級 (実測 ~116ns)。
    let slow = build_profile(3_000_000, &[1_000], 200, 60);
    let s_slow = simulate(&slow, ema_only_cfg(), 0, 0);
    eprintln!(
        "MODEL slow ~1ppb/s     : max={}ns tail_mean={}ns  (observed ~116ns)",
        s_slow.max_abs_phase_ns, s_slow.tail_mean_abs_phase_ns
    );

    // 観測2: 数十分スケールの warming (加速度あり) → ~µs の excursion (実測 offset② ~2µs / 4.5h ドリフト)。
    let warm = warming(8_000_000.0, 300.0, 600, 60);
    let s_warm_ema = simulate(&warm, ema_only_cfg(), 0, 0);
    let s_warm_ab = simulate(&warming(8_000_000.0, 300.0, 600, 60), DisciplinedClockConfig::DEFAULT, 0, 0);
    eprintln!(
        "MODEL warming (accel)  : EMA max={}ns tail={}ns | α-β max={}ns tail={}ns  (observed peak ~2000ns)",
        s_warm_ema.max_abs_phase_ns, s_warm_ema.tail_mean_abs_phase_ns,
        s_warm_ab.max_abs_phase_ns, s_warm_ab.tail_mean_abs_phase_ns
    );

    // モデルが観測の桁を再現: 緩いドリフトで O(100ns)、能動 warming のピークで O(µs)。
    assert!(
        (40..=300).contains(&s_slow.max_abs_phase_ns),
        "slow-drift model peak {}ns not in observed O(100ns) ballpark",
        s_slow.max_abs_phase_ns
    );
    assert!(
        (1_000..=4_000).contains(&s_warm_ema.max_abs_phase_ns),
        "warming model peak {}ns not in observed O(µs) ballpark",
        s_warm_ema.max_abs_phase_ns
    );
    // 重要な含意: 閉ループでは PLL 積分が一定ランプを吸収するため、α-β の上乗せは warming ピークを ~1.2-2x
    // 縮める程度 (steady では既に ns)。ns オーダー化には TCXO/恒温など HW 側が要る、という結論の裏付け。
    assert!(
        s_warm_ab.max_abs_phase_ns < s_warm_ema.max_abs_phase_ns,
        "α-β should reduce the warming peak: ab={}ns ema={}ns",
        s_warm_ab.max_abs_phase_ns, s_warm_ema.max_abs_phase_ns
    );
    // 整定後はどちらも ns 級 (warming が止めば offset は戻る)。
    assert!(
        s_warm_ab.tail_mean_abs_phase_ns < 500,
        "settled phase should be ns-order: {}ns",
        s_warm_ab.tail_mean_abs_phase_ns
    );
}

/// 実データ replay (モデル検証の本命)。現行 firmware (single-EMA) の実ログから生 GPS PPS 間隔 (PPS 行) を
/// プラントへ流し、デプロイ構成 (単一EMA, 出力は freq_ppb の ppb 丸め + trim, PLL 既定 i_den=128) でモデルの
/// hwphase を生成。実測 hwphase (PPSGEN 行) と「励起の大きさ (std) と相関」を比べる。手で選んだ理想曲線でなく
/// 実データを入力にするので循環しない (モデル妥当性の最強の裏取り)。
///   GNSSDO_REPLAY_LOG=/path/to/pps-flash.log cargo test -p gnssdo --test thermal_plant -- --ignored replay
#[test]
#[ignore = "real-data replay; needs GNSSDO_REPLAY_LOG=<defmt log>"]
fn replay_real_log_matches_hwphase() {
    let Ok(path) = std::env::var("GNSSDO_REPLAY_LOG") else {
        eprintln!("REPLAY skipped: set GNSSDO_REPLAY_LOG=<defmt log path>");
        return;
    };
    let text = std::fs::read_to_string(&path).expect("read log");
    let gps: Vec<i64> = text
        .lines()
        .filter(|l| l.contains("] PPS count="))
        .filter_map(|l| parse_kv(l, "interval_ns="))
        .collect();
    let real_hw: Vec<i64> = text
        .lines()
        .filter(|l| l.contains("PPSGEN count="))
        .filter_map(|l| parse_kv(l, "hwphase_ns="))
        .collect();
    let n = gps.len().min(real_hw.len());
    assert!(n > 100, "not enough samples: {n}");

    // デプロイ構成を env で選ぶ (ログを採ったファームに合わせる):
    //  GNSSDO_REPLAY_IDEN=<128|64|32|16>  PLL 積分分母 (既定 128)
    //  GNSSDO_REPLAY_AB=1                  α-β + predicted_freq_mppb (丸め無し)。未設定なら単一EMA + freq_ppb 丸め
    let iden: i64 = std::env::var("GNSSDO_REPLAY_IDEN")
        .ok()
        .and_then(|s| s.parse().ok())
        .unwrap_or(128);
    let ab = std::env::var("GNSSDO_REPLAY_AB").is_ok();
    let clk_cfg = if ab {
        DisciplinedClockConfig::DEFAULT
    } else {
        ema_only_cfg()
    };
    let mut clock = DisciplinedClock::with_config(clk_cfg);
    let mut pll = PhaseLockLoop::with_config(PhaseLockLoopConfig {
        i_den: iden,
        ..PhaseLockLoopConfig::DEFAULT
    });
    let mut phase = 0i64;
    let mut model_hw = Vec::with_capacity(n);
    for &iv in gps.iter().take(n) {
        clock.update_freq(iv);
        let true_f = (iv - 1_000_000_000) * 1000; // mppb: この秒の真の水晶周波数
        let u = pll.update(phase, true);
        let commanded = if ab {
            clock.predicted_freq_mppb() + u.freq_trim_mppb
        } else {
            clock.freq_ppb() * 1000 + u.freq_trim_mppb // 単一EMA デプロイは ppb 丸め
        };
        phase += (commanded - true_f) / 1000 - u.phase_corr_ns;
        model_hw.push(phase);
    }
    eprintln!("REPLAY cfg: i_den={iden} alpha_beta={ab}");
    // ショック再現の確認: 窓全体 (除外なし) での実測/モデルの最大励起。
    let rmax = real_hw.iter().take(n).map(|x| x.abs()).max().unwrap_or(0);
    let mmax = model_hw.iter().map(|x| x.abs()).max().unwrap_or(0);
    eprintln!("REPLAY max|hwphase| full window: real={rmax}ns model={mmax}ns");

    // 1ステップ予測誤差 (累積なし): 実 hwphase[n] を制御に入れ、同じ制御コードで 1 歩先を予測し real[n+1]
    // と比べる。プラント残差は 0 なので、これが小さければ「モデルの制御計算 (commanded) も実機と一致 =
    // per-step 忠実」が確定し、閉ループ replay の高ゲイン劣化は累積感度であってモデル不備でないと言える。
    let mut clock2 = DisciplinedClock::with_config(if ab {
        DisciplinedClockConfig::DEFAULT
    } else {
        ema_only_cfg()
    });
    let mut pll2 = PhaseLockLoop::with_config(PhaseLockLoopConfig {
        i_den: iden,
        ..PhaseLockLoopConfig::DEFAULT
    });
    let mut os_err = Vec::with_capacity(n);
    for i in 0..n - 1 {
        let iv = gps[i];
        clock2.update_freq(iv);
        let u = pll2.update(real_hw[i], true); // 実測 hwphase を入力
        let cmd = if ab {
            clock2.predicted_freq_mppb() + u.freq_trim_mppb
        } else {
            clock2.freq_ppb() * 1000 + u.freq_trim_mppb
        };
        let pred_next = real_hw[i] + (cmd - (iv - 1_000_000_000) * 1000) / 1000 - u.phase_corr_ns;
        if real_hw[i].abs() < 5_000 && real_hw[i + 1].abs() < 5_000 {
            os_err.push((real_hw[i + 1] - pred_next).abs());
        }
    }
    if !os_err.is_empty() {
        let m = os_err.iter().sum::<i64>() as f64 / os_err.len() as f64;
        let mx = *os_err.iter().max().unwrap();
        eprintln!(
            "REPLAY one-step pred error: mean={m:.0}ns max={mx}ns (n={}) [plant residual=0; tests control match]",
            os_err.len()
        );
    }

    // 起動収束 (最初の 10%) と、ログ側の K較正/holdover復帰の巨大スパイク (|hw|>5µs) を除外した窓で比較。
    let (mut rr, mut mm) = (Vec::new(), Vec::new());
    for i in (n / 10)..n {
        if real_hw[i].abs() > 5_000 {
            continue;
        }
        rr.push(real_hw[i] as f64);
        mm.push(model_hw[i] as f64);
    }
    let cnt = rr.len() as f64;
    let (mr, mmean) = (rr.iter().sum::<f64>() / cnt, mm.iter().sum::<f64>() / cnt);
    let (mut vr, mut vm, mut cov) = (0f64, 0f64, 0f64);
    for k in 0..rr.len() {
        let (dr, dm) = (rr[k] - mr, mm[k] - mmean);
        vr += dr * dr;
        vm += dm * dm;
        cov += dr * dm;
    }
    let (sdr, sdm) = ((vr / cnt).sqrt(), (vm / cnt).sqrt());
    let corr = cov / (vr.sqrt() * vm.sqrt());
    eprintln!(
        "REPLAY n={n} window={} | real: mean={mr:.0}ns std={sdr:.0}ns | model: mean={mmean:.0}ns std={sdm:.0}ns | corr={corr:.2}",
        rr.len()
    );
    // モデルが実 hwphase の励起スケールを桁で再現 (std が 0.3..3x) し、正の相関を持つこと。
    assert!(
        sdm > sdr * 0.3 && sdm < sdr * 3.0,
        "model excursion std {sdm:.0} not within 0.3-3x of real {sdr:.0}"
    );
    assert!(corr > 0.3, "model hwphase weakly correlated with real: corr={corr:.2}");
}
