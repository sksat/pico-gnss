//! GNSS 受信機固有レイヤ — 秋月 AE-GNSS-EXTANT / GYSFFMANC (MediaTek MT3333)。
//!
//! *この受信機* に依存するもの — デフォルトボーレート・PMTK 設定コマンド (dynamic model / SBAS /
//! NMEA 文選択) と MTK 固有のフレーミング — をここに集約する。別の GNSS モジュールに替えるときは
//! このモジュールだけ書き換えればよい。firmware の他の部分は汎用 NMEA / PPS だけを扱う。

use core::fmt::Write as _;

use defmt::{info, warn};
use embassy_rp::uart::BufferedUartTx;
use embassy_time::Timer;
use embedded_io_async::Write;
use heapless::String;

/// AE-GNSS-EXTANT (GYSFFMANC) のデフォルトボーレート 9600。
pub const GNSS_BAUD: u32 = 9600;

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
    info!("pico-gnss: PMTK config sent ({} cmds)", PMTK_INIT.len());
}

/// FW バージョン応答 ($PMTK705) か。受信機固有の talker 判定をここに集約。
pub fn is_fw_version_response(sentence: &str) -> bool {
    sentence.starts_with("$PMTK705")
}
