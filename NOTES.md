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

**ソフト (embassy Input + `Instant::now()`) はジッタ σ ≈ 9µs が下限**。これは PPS 信号自体
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
  **±数µs** が下限だったが、err はエッジ同士で測るので **エポックも予測も PIO の ns 時刻**にしたら
  **σ ≈ 11ns / peak-peak ~37ns** (16ns tick が下限) まで下がった。補正なしなら毎秒 ~2.8µs ずれる。
  ※ 連続時計 (TIME 行) の絶対値は依然 µs アンカー (PIO は FIFO 経由でエッジ時しか読めないため)。
- **PPS 欠落・holdover をまたぐ err**: ① **freshness ガード** — 新しい PPS エッジが来た時だけ RMC とペア
  する (`pending_fresh`)。欠落中に stale エッジを複数秒へペアすると err が ±整数秒の偽値になるのを防ぐ
  (実機ログで `pps_local_us` が複数秒重複して発覚)。② **`snap_to_second_ns`** — 復帰時の整数秒ズレを除いて
  sub 秒残差だけ残す。これで **N 秒 holdover の真の時刻誤差**が読める (実データ: 25s holdover → 360ns)。
- **補正タイマ**: `true_to_local_ns(true_ns)` で「真の時間で N 待つ」のに必要なローカル ns を得る
  (生の `Timer::after` は水晶公差 +2.7ppm 分ズレる)。これと `local_instant_for_unix_ns` が補正の素。
- **規律 PPS 出力 — 2 段階**:
  - ソフト版 (旧 `pps_out_task`): `local_instant_for_unix_ns` で UTC 秒境界をスケジュールし GPIO トグル。
    holdover 対応だが `late` は embassy executor ジッタが下限 (実機 mean ~1.4ms / σ ~244µs) → 廃止。
  - **PIO 版 (現行)**: SM1 が GP3 に規律パルスを**ハード生成** (周期 = clk×(1+ppb/1e9)-overhead を CPU が
    毎秒 push、`pull noblock` で保持)。エッジは executor 非依存。SM2 が GP4 で**ループバック捕捉** (GP3→GP4
    ジャンパ) し周期を計測。実機 **ジッタ 16ns (= PIO 1 tick、量子化限界)**・GPS PPS と ~24ppb 一致。
    `PPSGEN count= interval_ns= dev_ns=` を出力。ソフト版比 ~1.5 万倍クリーン。
- **規律 PPS 出力の UTC 位相同期 (stage ①, ソフト)**: 周波数規律は周期が合うだけで、エッジが UTC 秒境界に
  乗るとは限らない (位相は任意)。位相を合わせるべく gen_capture_task で出力エッジの UTC (Instant 経由) →
  秒境界ズレ `phase_ns` を測り、周期を 1 周だけ伸縮して引き込む。**到達点: 周期 16ns クリーンを保ったまま
  UTC 位相を ~±1〜2ms に整定**。`PPSGEN ... phase_ns=` を出力。制御で踏んだ罠 → 罠 12。
- **時刻系の限界整理**: freq/周期 = 16ns。**ソフト位相同期 = ~±1〜2ms**。pps_task と gen_capture を
  **InterruptExecutor (高優先度割込, SWI_IRQ_0=P0)** に載せ、エッジ→Instant 読みのウェイクアップ遅延を
  ms→µs に下げると一部 ±40µs まで締まるが、~±1.5ms のリミットサイクルが残る (測定の構造ノイズ + 制御の
  相互作用)。**ns/µs の位相同期にはエッジを PIO ハードでタイムスタンプする stage ② が必須** (今後)。
  (注: PIO IRQ の優先度を下げると pps_task/gen_capture の捕捉が枯渇するので SWI だけ上げる。罠13)

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

### 10. PIO 自走カウンタ生成器: 周期保持は X、カウントダウンは Y に分ける
規律 PPS 生成 (SM1) で `pull noblock; mov x,osr; ...; delay: jmp x-- delay` と書くと、`jmp x--` が
**周期を保持しているはずの X を 0/0xFFFFFFFF まで潰す**。次周の `pull noblock` (FIFO 空→`mov osr,x` 仕様)
がゴミを再ロードし、2 発目以降が ~34s 周期に化けた。→ **周期は X に保持、カウントダウンは Y** (`mov y,osr;
jmp y-- delay`)。出力ピンは起動時に `set pindirs, 1` で出力許可する (SET ピン = 生成ピン)。

### 11. 新しい firmware ログ行は「観測経路」にも必ず通す (phantom バグの温床)
PIO 規律出力のループバック計測で `PPSGEN` が全く出ず、「3 個目の SM(SM2) が壊れている」と何度も
リフラッシュして誤デバッグした。真因は **webapp server の `--log` フィルタ (NMEA/PPS/SYNC/TIME/… の
include リスト) に `PPSGEN` を入れ忘れていただけ** で、firmware は最初から正常に出していた。
→ **新ログ行を足したら server のフィルタ/パースにも足す。疑わしいときは server を介さず
`probe-rs run <elf>` で生出力を直接 grep** して切り分ける (これで一発で判明した)。

### 12. 位相同期の制御ループ: 測定遅れ → 連続フィードバックは発振、平滑 one-shot に
規律 PPS 出力の UTC 位相同期で、出力エッジの位相を測って周期を補正する制御を入れたら **±1.5ms で発振**した。
- **原因**: 補正は「1 つ前のエッジで測った位相」に基づき効くのは「次のエッジ」= **1 サンプル遅れ**。
  これに連続比例制御 (ゲイン k) をかけると `λ²−λ+k=0`。k=1 (デッドビート) で発振、k=1/2 でリンギング
  (複素根 \|λ\|=0.707, 偏角45°→周期8ステップ。実測の振動周期とぴったり一致)、**k=1/4 が実根境界=臨界減衰**。
- それでも安定しなかった真因: **位相の「測定」が CPU の `wait_pull` ウェイクアップ遅延 (~ms, executor の
  忙しさで変動) に汚染される**こと。連続フィードバックがこのノイズを追って発振。
- **対処**: 位相を EMA 平滑し、**8 エッジに 1 回だけ one-shot 補正**。連続フィードバックをやめたので発振せず、
  周期も 7/8 は無補正で 16ns クリーン。ただし絶対精度は測定ノイズ律速で **~±1〜2ms** が下限。
- **教訓**: 制御対象 (周期) の精度 (ns) と、フィードバックの「測定」精度 (ms) は別物。測定が悪いと制御で
  そこを超えられない。ns 位相同期は測定をハード化 (PIO で出力エッジを GPS PPS と同じカウンタで捕捉) する以外ない。

## 精度指標の意味 (webapp ヘッダ)

- **位置 `X m` = 水平 CEP(50%)**: 直近 ~2 分窓の測位点の経験的ばらつき (50% がこの半径内)。
  詳細パネルに R95(95%)・2DRMS・σE/σN も。**ばらつき(精度=precision)であって真値とのズレ
  (確度=accuracy)ではない**。cold start 収束時のジャンプを除くため直近窓で評価する。
- **時刻 `±X µs` = PPS タイムスタンプのジッタ(1σ)**: PPS 間隔偏差の標準偏差。上記の ~9µs 下限。
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
現実的な水平精度の下限は **~2-3m** (補強なしコンシューマ GNSS 相当)。サブメータが要るなら
SLAS/CLAS 対応受信機 (u-blox F9 系等) が必要。
秋月に[みちびき受信不具合の告知 (2022)](https://akizukidenshi.com/goodsaffix/gysffmanc_notification_20221118.pdf)あり。

## 仕様との突き合わせ (実測 vs データシート)

中身は MediaTek **MT3333** (FW `MT3333_AXN5.1.9`)。公称値と窓際固定での実測の比較:

| 項目 | モジュール仕様 | 実測 (窓際) | 評価 |
|---|---|---|---|
| **1PPS 確度** | **±10 ns RMS** (MT3333) | **σ 11ns** (好条件) / 35ns (38分・温度込み) | **ほぼ仕様通り**。PIO 捕捉(16ns 分解能)が仕様を検証できている |
| 測位精度 | 2m / 2.5m CEP (公称, 良好環境) | CEP 5〜13m | 窓際の弱信号で悪化 (環境律速、モジュールの非) |
| 構成 | GPS/GLONASS/QZSS, 99ch, 10Hz | GPS+GLONASS 捕捉, QZSS 未捕捉, 1Hz 運用 | QZSS は窓際で C/N0 不足 |
| 感度 (追尾) | ~ -165 dBm (MT3333) | C/N0 max ~28 dBHz | 弱信号 (窓際) |
| 電源 | VCC 3.8-5V / IO 2-3.6V | VBUS 5V / 3.3V TTL | OK |

- **1PPS が ±10ns 仕様にドンピシャ**なのが最大の収穫: 受信機が仕様通りに動いていることと、こちらの
  PIO ハードキャプチャ (σ11ns) が**その仕様を実測検証できる精度**であることを同時に確認できた。
  GPSDO 時刻補正の残差 (clock err σ ~11ns) もこの 1PPS 確度が源泉。
- holdover 精度はデータシートに無い (GPSDO は自前実装)。実測で **25s 途切れ → ~360ns** (周波数安定度
  ~15ppb からの理論見積りと一致)。webapp の「holdover 経過→誤差」散布図で可視化。
- 出典: [MT3333 (MediaTek)](https://www.mediatek.com/products/location-intelligence/mt3333),
  1PPS ±10ns は MT3333 系モジュール (LOCOSYS MC-1612 ±11ns / Skylab SKG09D ±10ns) でも一致。
