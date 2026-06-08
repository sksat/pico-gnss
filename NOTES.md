# pico-gnss 設計メモ・知見

Raspberry Pi Pico (RP2040) + 秋月 AE-GNSS-EXTANT+ANT_SET (太陽誘電 GYSFFMANC) で
GNSS 受信・PPS 時刻同期をテストし、リアルタイム可視化するプロジェクトの設計判断と、
bring-up でハマった罠の記録。

## アーキテクチャ

```
GYSFFMANC ──NMEA(UART0 RX=GP1)──┐
          ──PPS(GP2)───────────┤   RP2040 firmware        probe-rs (PicoBridge Lite)
          ←─PMTK(UART0 TX=GP0)──┘   - 生 NMEA を forward      RTT/defmt
                                    - PPS↔UTC 秒を対応付け  ──────────────► Node ブリッジ ──WS──► React ダッシュボード
```

- **firmware は「フォワーダ＋時刻同期」**に徹する。NMEA のパース・可視化は host (Web) 側。
  firmware は defmt-rtt に 3 種類の行を流す:
  - `NMEA $GxXXX,...*hh`
  - `PPS count=<n> interval_us=<us> state=<First|Locked|Irregular> missed=<m>`
  - `SYNC pps_local_us=<t> unix_s=<s> drift_us=<d>`
- **host テスト可能なロジックを lib に分離** (`src/assembler.rs` フレーミング, `src/pps.rs` PPS 判定,
  `src/timesync.rs` クロック規律)。embassy を target-gated dep にし `cargo test-host` で host 実行。
- webapp は **React 19 + Vite + TypeScript + react-leaflet**、Node ブリッジは依存ゼロ。

## 時刻同期の設計

PPS 立ち上がりは UTC 秒境界。**その瞬間の local timer 値 (RP2040 TIMER, 1µs) を、後続 NMEA(RMC)
の UTC 秒と対応付けて** µs 精度の UTC エポックを device 上に保持する ([`PpsTimeSync`])。

**なぜ firmware 側でやるか**: host (probe-rs RTT 経由) で同期すると USB/probe の往復ジッタ
(数十 ms) が乗り、PPS 本来の精度が失われる。エッジを µs で刻める MCU 上で対応付けるのが必須。

### PPS タイムスタンプ: ソフト ~9µs → PIO ハードキャプチャ ~10ns (重要)

**ソフト (embassy Input + `Instant::now()`) はジッタ σ ≈ 9µs が床**。これは PPS 信号自体
(モジュール仕様で数十 ns) ではなく RP2040 側のソフトタイムスタンプのレイテンシ揺らぎ。

- 原因: RP2040 は Cortex-M0+ で BASEPRI が無く、`critical-section` が**全割り込みをマスク**する。
  defmt の RTT 書き込み等で critical-section 中は PPS エッジ割り込みが遅延する。
- **効かなかった対策**: GPIO 割込を最優先(P0)に・PPS タスクを高優先 InterruptExecutor(P1) で走らせる
  → critical-section マスクの前では無力で σ は改善せず (9〜10µs)。複雑さだけ増えるので不採用 (revert 済)。

**PIO ハードキャプチャで σ ≈ 10ns を達成** (約 900 倍改善):
- PIO0 SM0 で自走ダウンカウンタ X を 2 サイクル毎に減算しつつ pin を監視し、立ち上がりで X を FIFO に push。
  tick = 2 cyc = **16ns @125MHz**。CPU/割込/critical-section に一切依存しない (ハードでラッチ)。
- CPU は連続する X の差 (wrapping_sub, ダウンカウンタ) から間隔を ns で得る。
- 実測: interval ≈ 1,000,002,5xx ns、**ばらつきは 16ns 量子化だけ (σ ≈ 8-10ns, peak-peak 32ns)**。
  平均オフセット ≈ +2.5µs/s = **RP2040 水晶の +2.5 ppm** がクリーンに測れる。
- 注意: X は ~68s で 1 周し、0 通過時に低位相ループで稀に誤キャプチャ (短い偽間隔 ≈ -37ms) が出る。
  **このグリッチを弾くフィルタは ±1ms と厳しくする** — 当初 ±50ms だと周回グリッチがすり抜けて
  GPSDO の周波数 EMA を汚染し、freq が -3826ppm・σ が 2.5ms に化けた (実機評価で発覚)。真の間隔は
  1s±数µs なので 1ms でも余裕。firmware (`DisciplinedClock::SANE_DEV_NS`) と webapp 両方で ±1ms。
- firmware は `PPS ... interval_ns=<ns> ...` を出し、webapp は ns でジッタ σ を表示する。

### GPSDO (GPS 規律発振器)

PIO の精密な PPS 間隔から **RP2040 水晶の周波数オフセットを推定**し、UTC を規律する
([`DisciplinedClock`], host テスト済み)。**PPS が切れている間 (holdover) も推定周波数で外挿**して
時刻を保つのが GPSDO の肝。

- PPS 間隔の偏差 (interval_ns − 1e9) がそのまま周波数オフセット **ppb (= ns/s)**。
  実測 ≈ **+2.6 ppm (= +2600 ppb)** (RP2040 水晶)。これを EMA (α=1/32) で平滑する。
- 周波数推定は PIO の精密間隔から、UTC エポックは連続して読める local clock (embassy Instant) から
  と別々に与える (Instant の µs ジッタは絶対オフセットにのみ効き、周波数=holdover ドリフトには効かない)。
- firmware は 1Hz で `TIME unix_ns=<規律UTC> ppb=<推定> holdover_ms=<経過> locked=<0|1>` を出す。
  webapp はこれで device 規律クロックを表示し、PPS 断時は holdover カウンタを出す。
- 補正式: 真の経過 = local 経過 − local経過 × ppb/1e9 (local が ppb 分速い/遅いぶんを補正)。
  ロック後 holdover に入ると、残差周波数誤差ぶんだけ時刻がドリフトする (フル ppm でなく)。
- **時刻補正 API (PIO/Instant 二系統)**: クロックは 2 つの local 時刻でアンカーする。
  - `now_ns(pio)` = **PIO timebase** (ns 精度)。PPS エッジでの `err_ns` 計測に使う。
  - `now_from_instant_ns(inst)` = **Instant timebase** (連続クエリ/ticker 用、サブ秒は µs)。
  - `local_instant_for_unix_ns(utc)` = 指定 UTC が来る Instant local 時刻 (スケジューリング/補正待ち)。
- 精度は `err_ns` (補正後の 1 秒先読み残差) で測れる。当初エポックを Instant (µs) で取っていたので
  **±数µs** が床だったが、err はエッジ同士で測るので **エポックも予測も PIO の ns 時刻**にしたら
  **σ ≈ 11ns / peak-peak ~37ns** (16ns tick が床) まで下がった。補正なしなら毎秒 ~2.8µs ずれる。
  ※ 連続時計 (TIME 行) の絶対値は依然 µs アンカー (PIO は FIFO 経由でエッジ時しか読めないため)。
- **補正タイマ**: `true_to_local_ns(true_ns)` で「真の時間で N 待つ」のに必要なローカル ns を得る
  (生の `Timer::after` は水晶公差 +2.7ppm 分ズレる)。これと `local_instant_for_unix_ns` が補正の素。
- **規律 PPS 出力 (GP3)**: `pps_out_task` が GPSDO 補正済みクロックで UTC 秒境界をスケジュールし、
  GP3 に立ち上がりエッジを出す。GPS PPS が切れても holdover で UTC 秒に合わせて出し続ける。
  ただし**ソフトタイミング**なので `late` は embassy executor のジッタが下限 (実機 mean ~1.4ms / σ ~244µs;
  UART 処理と CLOCK ロック競合で遅延)。秒境界には毎回揃う (unix_s 連番・ドリフト無し) が、**ns/µs 精度の
  エッジには PIO ハードウェア生成が必要** (今後)。`PPSOUT unix_s= sched_us= fired_us= late_us= holdover_ms=` を出力。

## ハマった罠

### 1. `DEFMT_LOG` 未設定で RTT が無言になる
probe-rs で flash しても RTT に何も出ない (起動直後の `info!` すら出ない) → まず `DEFMT_LOG`
を疑う。defmt はビルド時に `DEFMT_LOG` でレベルを決め、**未設定だと `info!`/`warn!` 等が
コンパイル時に消える** (`println!` だけは常に出る)。`.cargo/config.toml` の `[env] DEFMT_LOG="info"`
で解決。半日溶かした罠。GDB で halt すると executor は正常に poll ループを回していて切り分けられた。

### 2. embassy-rp 0.10 は defmt 1.x。logger も揃える
embassy-rp 0.10 / embassy-executor 0.10 は `defmt = "1.0.1"` を要求する。logger も同世代の
**`defmt-rtt = "1.x"`** に揃えること。defmt-rtt 0.4 (defmt 0.3 世代) を混ぜると `defmt 0.3.100`
シム経由でコンパイルは通るが、**probe-rs が RTT フレームをデコードできず無言**になる。

### 3. embassy-executor 0.10 の API 変更
- feature 名: `arch-cortex-m` → **`platform-cortex-m`**。
- spawn: `#[task]` 関数が `Result<SpawnToken, SpawnError>` を返す →
  `spawner.spawn(my_task(..).unwrap())` の形。

### 4. Cortex-M0+ は atomic CAS が無い
`static_cell` 等が `portable-atomic` を引くので
`portable-atomic = { features = ["critical-section"] }` を有効化しないとビルド不可。

### 5. 共通 GND が無いと電源も UART も成立しない
最終的な実機ブロッカーはこれだった。モジュール GND と Pico GND が繋がっておらず、モジュール
LED 消灯・UART 0 バイト。**GND 共通は最優先で確認**。

### 6. GYSFFMANC の電源は 3.8〜5V
Pico の 3V3 (3.3V) では電圧不足で動かない。VCC は **5V (VBUS)** を使う。ただし Pico を
PicoBridge Lite 経由給電のみにすると VBUS は無電圧になる (VBUS は Pico 自身の USB を挿した
ときだけ 5V) → Pico USB も挿す。IO は 2〜3.6V で 3.3V TTL 接続可。

### 7. 1PPS は 3 次元測位 (fix) 後のみ出力
屋内・アンテナ未接続・fix 前は PPS が出ないのが正常。NMEA センテンスは fix 無しでも連続で出る
(lat/lon 空) ので、まず NMEA 受信で UART 疎通を確認する。

### 8. QZSS (みちびき) は `$GPGSV` talker で PRN 193+ として出る
GYSFFMANC は QZSS を GLONASS のような専用 talker ($GQGSV) でなく **$GPGSV の PRN 193+** で
出す。talker だけ見ると QZSS が GPS 扱いになる → **PRN レンジでコンステレーション分類**する
(193–202 = QZSS, 120–158 = SBAS 等)。日本では QZSS が数機受かる。

### 9. ビルド失敗時に古いバイナリを焼かない (`;` でなく `&&`)
`cargo build ; probe-rs run <elf>` だと **build が失敗しても古い .elf が焼かれ**、変更が反映されてない
バイナリで延々デバッグする羽目になる (実際にハマった: `BufferedUartTx<'static>` の型エラーに気付かず、
config 変更が一切効いてないのに「PMTK が効かない」と誤診)。**必ず `cargo build && probe-rs run` で繋ぐ**。
バックグラウンド実行や `| tail` でエラーを見落としやすいので特に注意。

## 精度指標の意味 (webapp ヘッダ)

- **位置 `X m` = 水平 CEP(50%)**: 直近 ~2 分窓の測位点の経験的ばらつき (50% がこの半径内)。
  詳細パネルに R95(95%)・2DRMS・σE/σN も。**ばらつき(精度=precision)であって真値とのズレ
  (確度=accuracy)ではない**。cold start 収束時のジャンプを除くため直近窓で評価する。
- **時刻 `±X µs` = PPS タイムスタンプのジッタ(1σ)**: PPS 間隔偏差の標準偏差。上記の ~9µs 床。
  一定レイテンシ(固定オフセット)は σ に出ない。絶対的 UTC 一致はモジュール PPS 確度(数十 ns)に依存。

## モジュール設定 (PMTK)

起動時に PMTK コマンドを UART0 TX(GP0) から `config_task` (RX をブロックしない別タスク) で送る。
TX→モジュール RX 配線時に適用され、各コマンドに `$PMTK001,<cmd>,3`(成功) が返る。**実機で全 ACK 成功を確認**:

- `PMTK313,1` — SBAS 探索 ・ `PMTK301,2` — DGPS=SBAS ・ `PMTK286,1` — AIC
- `PMTK314,...` — NMEA 出力に **GST を追加** (測位の標準偏差 σ を受信機が直接出す)。webapp の精度パネルに
  `rcv σH/σV/RMS` として表示。フィールド順: GLL,RMC,VTG,GGA,GSA,GSV,GRS,GST,(res×5),MALM,MEPH,MDGP,MDBG,ZDA,MCHN。
- `PMTK605` — FW バージョン照会 → `$PMTK705` で返る。**この個体は `MT3333_AXN5.1.9_MODULE_STD_F0 / 太陽誘電 /
  9600bps`** (チップは MediaTek MT3333 確定)。webapp ヘッダに表示。

### PMTK 送信で踏んだ罠
- **送信は RX をブロックしない別タスクで** — 同じ main ループで `Timer`/送信中だと RX が読まれず、
  届く ACK/NMEA が RX バッファ溢れで消える。
- **行バッファは長コマンド分確保** — `PMTK314` は ~51 文字。`String<48>` だと truncate されて不完全コマンドになり拒否される。
- **`$PMTK705` 等の一度きりの応答はサーバ側でキャッシュ**して後続クライアントに再送 (WS は接続後しか受け取れない)。

### このモジュールでは config による sub-meter 化は不可 (調査結果)

- チップは **MediaTek MT3333 系** (PMTK が通り、各コマンドに `$PMTK001,<cmd>,3` (3=成功) を ACK。
  実機で TX→モジュール RX 配線時に 313/301/286 すべて成功を確認)。
- **GYSFFMANC は QZSS SLAS 非対応**: [QZSS 公式の SLAS/CLAS 対応製品リスト](https://qzss.go.jp/usage/products/slas-clas.html)
  に太陽誘電/GYSF 系は無い (u-blox/Sony/Furuno 等のみ)。秋月の説明も「みちびき対応(=QZSS 測距)」止まりで
  サブメータを謳わない。→ **L1S 補強を復調しないので SLAS を有効化する PMTK は存在しない**。
- 日本の従来型 SBAS (**MSAS**) は **2020 年に運用終了**。よって `PMTK313,1`/`PMTK301,2` は ACK は
  成功するが日本では fix quality は 1 のまま (WAAS/EGNOS 圏では有効なので firmware には残す)。
- QZSS (PRN 193-202) は既に測距用に NMEA 出力されている (GSV)。使われない時は SNR が無い=信号が弱い
  だけで config の問題ではない。

**結論**: このハードの精度レバーは config でなく**アンテナの天空確保 (衛星数↑・HDOP↓)** のみ。
現実的な水平精度の床は **~2-3m** (補強なしコンシューマ GNSS 相当)。サブメータが要るなら
SLAS/CLAS 対応受信機 (u-blox F9 系等) が必要。
秋月に[みちびき受信不具合の告知 (2022)](https://akizukidenshi.com/goodsaffix/gysffmanc_notification_20221118.pdf)あり。
