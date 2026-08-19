//! GNSS 受信機固有レイヤ — 秋月 AE-GNSS-EXTANT / GYSFFMANC (MediaTek MT3333)。
//!
//! *この受信機* に依存するもの — デフォルトボーレート・PMTK 設定コマンド (dynamic model / SBAS /
//! NMEA 文選択) と MTK 固有のフレーミング — をここに集約する。別の GNSS モジュールに替えるときは
//! このモジュールだけ書き換えればよい。firmware の他の部分は汎用 NMEA / PPS だけを扱う。

use core::fmt::Write as _;

use defmt::{info, warn};
use embassy_rp::uart::{BufferedUart, BufferedUartTx};
use embassy_time::{Duration, Instant, Timer, with_timeout};
use embedded_io_async::{Read as _, Write};
use heapless::String;
use portable_atomic::{AtomicU32, Ordering};

/// AE-GNSS-EXTANT (GYSFFMANC) のデフォルトボーレート 9600。
pub const GNSS_BAUD: u32 = 9600;

/// 起動時に引き上げる先のボーレート。
///
/// # なぜ上げるのか (精度ではなく**正しさ**の問題)
///
/// PPS エッジと UTC 秒の対応付けは NMEA センテンスの到着で行うが、9600bps ではセンテンス
/// バーストが ~640ms あり、しかも PPS エッジから数百 ms 遅れて始まる。結果、時刻センテンスは
/// **次のパルスのほぼ真上**に届く。実測 (188 サンプル):
///
/// ```text
/// RMC から次の PPS エッジまでの余裕: mean 490ms, sd 460ms, min 2ms
/// ```
///
/// 分布はエッジの前後数十 ms に割れた二峰で、どちらに落ちるかはコイン投げ。衛星数が増えて
/// GSV が伸びただけで反転し、**GPSDO の UTC の秒番号が実行中に 1 秒飛ぶ**。位相しか見ていない限り
/// 見えない (GPSDO 1PPS は GPS エッジ上に ns で乗ったまま、違う秒のラベルを付けている)。
///
/// 12 倍にするとバーストは ~53ms になり、コイン投げが数百 ms の余裕に変わる。実機で確認済み:
/// 9600 では ±1 秒の補正が要ったが、115200 では補正なしで時刻が合う。
pub const GNSS_FAST_BAUD: u32 = 115_200;

/// **実験**: dynamic model (PMTK886) を一定間隔で交互切替し、各セグメントの出力位相 (hwphase) を
/// 受信を揃えた状態で比較する A/B。stationary(886,4) が wander を抑えるか増やすかを実機で決める
/// (過去レポートの paired A/B は mobile が ~90ns 良かったが本番未採用・要再現)。
/// **production 既定 false** (OP_MODE を起動時に一度だけ送り固定)。
pub const DYNMODEL_AB: bool = false; // production 既定。実験後 revert。
/// 各セグメントの長さ (秒)。切替直後は受信機内部フィルタの過渡があるので解析側で先頭を捨てる。
const DYNMODEL_AB_SECS: u64 = 360; // 6 分/セグメント
/// 現在の dynamic model コード (PPSGEN の `dynmode` 欄)。4=stationary(886,4)、0=mobile(886,0)、
/// 255=未設定。gen_capture_task がこれを読んでログするので、セグメントごとに hwphase を層別できる。
pub static DYN_MODE: AtomicU32 = AtomicU32::new(255);

/// 運用モード = 受信機の dynamic model (MT3333 PMTK886)。
/// **この GPSDO は固定 timing 用途**なので既定は `FixedTiming` (stationary)。受信機に
/// 「動いていない」という強い事前情報を渡すと、弱信号での位置/速度の暴れと PPS 選別への悪影響が
/// 減る (smart-friend GPT-5.5)。移動運用するときだけ `Mobile` にして stationary を無効化する。
/// 自動切替はしない (弱信号の NMEA speed が嘘をつくとモード切替自体が新たな不安定要因になる)。
#[derive(Clone, Copy)]
enum OpMode {
    FixedTiming, // PMTK886,4 = stationary。窓際固定・timing 専用。
    #[allow(dead_code)]
    Mobile, // PMTK886,0 = normal。移動運用時はこちら (将来 UI/設定口から切替)。
}
const OP_MODE: OpMode = OpMode::FixedTiming;

impl OpMode {
    /// dynamic model を設定する PMTK886 コマンド本体 (チェックサムは送信時に付く)。
    const fn pmtk886(self) -> &'static str {
        match self {
            OpMode::FixedTiming => "PMTK886,4", // stationary
            OpMode::Mobile => "PMTK886,0",      // normal
        }
    }

    /// `DYN_MODE` 欄に載せるコード (4=stationary, 0=mobile)。
    const fn code(self) -> u32 {
        match self {
            OpMode::FixedTiming => 4,
            OpMode::Mobile => 0,
        }
    }
}

/// dynmode コード (4=stationary, それ以外=mobile) → PMTK886 コマンド。A/B 実験で使う。
const fn pmtk886_for(code: u32) -> &'static str {
    if code == 4 { "PMTK886,4" } else { "PMTK886,0" }
}

/// 起動時にモジュールへ送る PMTK 設定 (チェックサムは送信時に計算)。
/// 実機で各コマンドに `$PMTK001,<cmd>,3`(成功) が返ることを確認済 (チップは MT3333 系)。
/// 注: SBAS は地域依存。日本の MSAS は 2020 終了済なので fix quality は 1 のまま
///     (WAAS/EGNOS 圏では有効)。GYSFFMANC は QZSS SLAS 非対応で sub-meter は不可。詳細は NOTES.md。
const PMTK_INIT: &[&str] = &[
    OP_MODE.pmtk886(), // dynamic model (既定 stationary。固定 timing 用途の事前情報)
    "PMTK313,1",       // SBAS 探索を有効化 (WAAS/EGNOS 圏で有効)
    "PMTK301,2",       // DGPS 補正源 = SBAS
    "PMTK286,1",       // AIC (アクティブ干渉除去) 有効化
    // 試したが不採用: PMTK319,1 (SBAS Integrity) / PMTK386,0 (static-nav freeze off) は実機で ACK
    // する ($PMTK001,*,3) が timing に観測効果なし (319 は日本は SBAS 無で no-op、386 は 886,4
    // stationary が位置を保持するため無効)。MT3333 は qErr/survey-in 非対応でコマンドで詰める余地は無い。
    // NMEA 出力: GLL/RMC/VTG/GGA/GSA/GSV(各1) + GST(測位σ, field7) + ZDA(field17) を有効化。
    // フィールド順: GLL,RMC,VTG,GGA,GSA,GSV,GRS,GST,(res×5),MALM,MEPH,MDGP,MDBG,ZDA,MCHN。
    "PMTK314,1,1,1,1,1,1,0,1,0,0,0,0,0,0,0,0,0,1,0",
    "PMTK605", // FW バージョン照会 → $PMTK705 で返る (ACK は無く応答が版情報)
];

/// `$<payload>*<csum>\r\n` を組み立てて送る (汎用 NMEA チェックサムは rp-pps)。
async fn send_pmtk<W: Write>(tx: &mut W, payload: &str) {
    let cs = rp_pps::nmea_checksum(payload.as_bytes());
    // PMTK314 (NMEA 出力設定) は ~51 文字になるので余裕を持って 96。溢れると truncate されて
    // 不完全コマンドになり、モジュールに拒否される (実際にハマった)。
    let mut line: String<96> = String::new();
    if write!(line, "${}*{:02X}\r\n", payload, cs).is_ok() {
        let _ = tx.write_all(line.as_bytes()).await;
    } else {
        warn!("send_pmtk: line too long: {=str}", payload);
    }
}

/// 受信機がどのボーレートで喋っているかを探し、[`GNSS_FAST_BAUD`] へ引き上げて**検証**する。
///
/// 戻り値は最終的に確立したレート。UART は分割前に渡すこと (`set_baudrate` は
/// [`BufferedUart`] にしかない)。
///
/// # なぜ「送って終わり」ではないのか
///
/// * **PMTK251 に ACK は無い**。返事は片側がまだ採用していないレートで送ることになるので原理的に
///   返せない。MT3333 の NMEA 仕様 (§2.3.14) もコマンドと許容レートを規定するだけで、切替に
///   どれだけかかるかは書いていない。つまり**確かめる以外に知る方法が無い**。
/// * **設定は firmware の再フラッシュを跨いで残る**。PMTK251 が既定へ戻るのは full cold start か
///   standby のときだけなので、再フラッシュ後もモジュールは前回のレートのまま。9600 で開いて
///   沈黙を見て「受信機が死んだ」と判断してしまう (実際にそう誤判定した)。
///
/// したがってコマンドの前後で port を **probe** する。これで後段の待ち時間は critical でなくなる
/// — 短すぎてもモジュールが実際に落ち着いた側を probe が見つけて追従するので、「一つのモジュールを
/// 一日測って調整した定数」を作らずに済む。
pub async fn establish_link(uart: &mut BufferedUart) -> u32 {
    let Some(found) = probe_baud(uart).await else {
        // probe は最後に試したレート (fast) を残したまま抜けている。既定へ戻しておかないと、
        // 受信機が後から既定レートで起動しても受信ループが追従できず、二度と復帰しない。
        uart.set_baudrate(GNSS_BAUD);
        warn!("GNSS: no NMEA at either baud — check wiring, or power-cycle the receiver");
        return GNSS_BAUD;
    };
    if found == GNSS_FAST_BAUD {
        info!("GNSS: already at {=u32}", GNSS_FAST_BAUD);
        return found;
    }

    send_pmtk(uart, "PMTK251,115200").await;
    // 9600 でコマンドが出切る (~21ms) のと、モジュールが送信中のセンテンスを終えるのに十分な長さ。
    // 上のとおり probe があるので厳密でなくてよい。起動時 1 回きりで、同期のロックには数秒かかる。
    Timer::after_millis(500).await;

    match probe_baud(uart).await {
        Some(rate) if rate == GNSS_FAST_BAUD => {
            info!("GNSS: baud raised to {=u32}", GNSS_FAST_BAUD);
            rate
        }
        Some(rate) => {
            warn!("GNSS: PMTK251 did not take; continuing at {=u32}", rate);
            rate
        }
        None => {
            warn!(
                "GNSS: lost the receiver after PMTK251; leaving port at {=u32}",
                GNSS_BAUD
            );
            uart.set_baudrate(GNSS_BAUD);
            GNSS_BAUD
        }
    }
}

/// 受信機が今どのレートで喋っているかを探し、port をそこに合わせる。**設定は変えない**。
///
/// [`establish_link`] と違って受信機に何も送らないので、設定を持たない側 (例のバイナリなど) が
/// 使う。`PMTK251` の設定は firmware の再フラッシュを跨いで残るため、9600 決め打ちで開くと、
/// 一度でも引き上げた受信機からは何も受け取れなくなる。
pub async fn follow_baud(uart: &mut BufferedUart) -> Option<u32> {
    let found = probe_baud(uart).await;
    match found {
        Some(rate) => info!("GNSS: talking at {=u32}", rate),
        None => {
            uart.set_baudrate(GNSS_BAUD);
            warn!("GNSS: no NMEA at either baud — check wiring, or power-cycle the receiver");
        }
    }
    found
}

/// 各レートを順に試し、NMEA が framing できたところで止める。port はそのレートのまま残す。
async fn probe_baud(uart: &mut BufferedUart) -> Option<u32> {
    for baud in [GNSS_BAUD, GNSS_FAST_BAUD] {
        uart.set_baudrate(baud);
        if await_nmea(uart, Duration::from_secs(3)).await {
            return Some(baud);
        }
    }
    None
}

/// NMEA センテンスが 1 本組み立つまで読む。中身は問わない — **framing が成立すること自体**が
/// 「このレートで合っている」の証拠で、それはレートが違えば真っ先に壊れる性質だから。
async fn await_nmea(uart: &mut BufferedUart, timeout: Duration) -> bool {
    let mut assembler = rp_pps::NmeaLineAssembler::new();
    let mut buf = [0u8; 64];
    let deadline = Instant::now() + timeout;
    loop {
        let Some(remaining) = deadline.checked_duration_since(Instant::now()) else {
            return false;
        };
        let Ok(read) = with_timeout(remaining, uart.read(&mut buf)).await else {
            return false;
        };
        // レート違いは framing / break エラーとして出る。切替直後は数個出るのが普通なので、
        // 判定材料にせず読み続ける。
        let Ok(n) = read else { continue };
        for &b in &buf[..n] {
            if let Some(sentence) = assembler.push(b)
                && sentence.starts_with(b"$G")
            {
                return true;
            }
        }
    }
}

/// モジュール設定 (PMTK) を送るタスク。**RX 受信 (main) をブロックしないよう別タスクにする**
/// — 同じループで送ると Timer/送信中に RX が読まれず、届く ACK が RX バッファ溢れで消える。
#[embassy_executor::task]
pub async fn config_task(mut tx: BufferedUartTx) {
    // 起動直後はモジュールが PMTK を受け付けないので十分待ってから、間隔も空けて送る。
    // 起動直後 (~1s) は UART が同期せず framing が多発しモジュールも未準備なので十分待つ。
    Timer::after_millis(2000).await;
    for cmd in PMTK_INIT {
        send_pmtk(&mut tx, cmd).await;
        Timer::after_millis(600).await;
    }
    // production は OP_MODE 固定。その固定コードを DYN_MODE に記録する (PPSGEN の dynmode 欄が常に有効)。
    DYN_MODE.store(OP_MODE.code(), Ordering::Relaxed);
    info!("pico-gnss: PMTK config sent ({} cmds)", PMTK_INIT.len());
    if DYNMODEL_AB {
        // 実験: dynamic model を **ABBA** で交互送出 (単純交互の周期が環境周期にエイリアスするのを避け、
        // 各セグメントの平均時刻位置を揃えて slow drift を一次で相殺する)。各セグメント DYNMODEL_AB_SECS。
        // 解析側は dynmode で層別し、切替直後の過渡を捨て、隣接ペア差 (mobile−stationary) で比較する。
        const SCHEDULE: [u32; 4] = [4, 0, 0, 4];
        info!(
            "DYNMODEL A/B: ABBA 886,4/886,0 every {=u64}s",
            DYNMODEL_AB_SECS
        );
        let mut i = 0usize;
        loop {
            let code = SCHEDULE[i % SCHEDULE.len()];
            send_pmtk(&mut tx, pmtk886_for(code)).await;
            DYN_MODE.store(code, Ordering::Relaxed);
            info!("DYNMODEL switch dynmode={=u32}", code);
            Timer::after_secs(DYNMODEL_AB_SECS).await;
            i += 1;
        }
    }
}

/// FW バージョン応答 ($PMTK705) か。受信機固有の talker 判定をここに集約。
pub fn is_fw_version_response(sentence: &str) -> bool {
    sentence.starts_with("$PMTK705")
}
