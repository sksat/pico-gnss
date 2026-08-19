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
- **host テスト可能なロジックを 2 つのコア crate に分離** (Cargo workspace, ともに host で `cargo test`):
  - **`gnssdo/`** = 同期そのもの・**依存ゼロ** no_std lib。GPSDO の同期=`gpsdo.rs`、PPS エッジ分類=`pps.rs`、
    出力位相 PLL=`pll.rs`。`update_epoch` で epoch を消費するだけで、時刻ソースには非依存。
  - **`rp-pps/`** = RP2040 PIO I/O **+ 時刻取り込み**。PIO program/capture/output/dither、NMEA フレーミング=
    `assembler.rs`、PPS↔UTC 秒の対応付け=`timesync.rs` (`PpsTimeSync`→`SyncEpoch`)。embassy-rp/rp2040-hal backend。
  - **`pico-gnss/`** = embedded 専用 firmware (`cd pico-gnss && cargo run`)。両 crate を配線する。
- webapp は **React 19 + Vite + TypeScript + react-leaflet**、Node ブリッジは依存ゼロ。

## 時刻同期の設計

PPS のパルスの始まりが UTC 秒境界 (この GPS-R は active low なので立ち下がり)。**そのエッジを 2 系統 (capture=PIO ns / query=Instant ns) で打刻し、
後続 NMEA(RMC) の UTC 秒と対応付けて** UTC エポックを device 上に確立する。対応付けは rp-pps の
[`PpsTimeSync`] が担い (`on_pps_edge`/`set_date`/`on_time`→`SyncEpoch`)、その epoch を gnssdo の同期
クロック (`Gnssdo::on_utc`/`DisciplinedClock::update_epoch`) に渡す。

**なぜ firmware 側でやるか**: host (probe-rs RTT 経由) で同期すると USB/probe の往復ジッタ
(数十 ms) が乗り、PPS 本来の精度が失われる。エッジを µs で刻める MCU 上で対応付けるのが必須。

### PPS タイムスタンプ: ソフト µs オーダ → PIO ハードキャプチャ ~10ns (重要)

**ソフト (embassy Input + `Instant::now()`) は µs オーダのジッタが下限**。σ は負荷や boot で
~2〜10µs 動く (20260703 の 8 boot 実測: 本体 ±2µs + critical-section 衝突時に数十 µs のスパイク、
プール σ ≈ 4µs。旧計測の 9.8µs はその日の負荷での値で、下限ではなかった)。これは PPS 信号自体
(モジュール仕様で数十 ns) ではなく RP2040 側のソフトタイムスタンプのレイテンシ揺らぎ。

- 原因: RP2040 は Cortex-M0+ で BASEPRI が無く、`critical-section` が**全割り込みをマスク**する。
  defmt の RTT 書き込み等で critical-section 中は PPS エッジ割り込みが遅延する。
- **効かなかった対策**: GPIO 割込を最優先(P0)に・PPS タスクを高優先 InterruptExecutor(P1) で走らせる
  → critical-section マスクの前では無力で σ は改善せず (9〜10µs)。複雑さだけ増えるので不採用 (revert 済)。

**PIO ハードキャプチャで σ ≈ 10ns を達成** (µs → ns、2〜3 桁の改善):
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

### GPSDO (GPS 同期発振器)

PIO の精密な PPS 間隔から **RP2040 水晶の周波数オフセットを推定**し、UTC を GPS に合わせ込む
([`DisciplinedClock`], host テスト済み)。**PPS が切れている間 (holdover) も推定周波数で外挿**して
時刻を保つのが GPSDO の肝。

- PPS 間隔の偏差 (interval_ns − 1e9) がそのまま周波数オフセット **ppb (= ns/s)**。
  実測 ≈ **+2.6 ppm (= +2600 ppb)** (RP2040 水晶)。これを EMA (α=1/32) で平滑する。
- 周波数推定は PIO の精密間隔から、UTC エポックは連続して読める local clock (embassy Instant) から
  と別々に与える (Instant の µs ジッタは絶対オフセットにのみ効き、周波数=holdover ドリフトには効かない)。
- firmware は 1Hz で `TIME unix_ns=<GPSDO の UTC> ppb=<推定> holdover_ms=<経過> locked=<0|1>` を出す。
  webapp はこれで device の同期クロックを表示し、PPS 断時は holdover カウンタを出す。
- 補正式: 真の経過 = local 経過 − local経過 × ppb/1e9 (local が ppb 分速い/遅いぶんを補正)。
  ロック後 holdover に入ると、残差周波数誤差ぶんだけ時刻がドリフトする (フル ppm でなく)。
- **時刻補正 API (capture/query 二系統)**: クロックは 2 つの local 時刻でアンカーする (RP2040 では capture=PIO, query=Instant)。
  - `now_from_capture_ns(capture)` = **capture timebase** (RP2040 では PIO, ns 精度)。PPS エッジでの `err_ns` 計測に使う。
  - `now_from_query_ns(query)` = **query timebase** (RP2040 では Instant, 連続クエリ/ticker 用、サブ秒は µs)。
  - `query_ns_for_unix_ns(utc)` = 指定 UTC が来る query (Instant) local 時刻 (スケジューリング/補正待ち)。
- 精度は `err_ns` (補正後の 1 秒先読み残差) で測れる。当初エポックを Instant (µs) で取っていたので
  **±数µs** が下限だったが、err はエッジ同士で測るので **エポックも予測も PIO の ns 時刻**にしたら
  **σ ≈ 11ns / peak-peak ~37ns** (16ns tick が下限) まで下がった。補正なしなら毎秒 ~2.8µs ずれる。
  ※ 連続時計 (TIME 行) の絶対値は依然 µs アンカー (PIO は FIFO 経由でエッジ時しか読めないため)。
- **PPS 欠落・holdover をまたぐ err**: ① **freshness ガード** — 新しい PPS エッジが来た時だけ RMC とペア
  する (`PpsTimeSync::on_time` が `Option::take` でエッジを 1 回だけ消費する)。欠落中に stale エッジを
  複数秒へペアすると err が ±整数秒の偽値になるのを防ぐ (実機ログで `pps_local_us` が複数秒重複して発覚)。
  ② **`snap_to_second_ns`** — 復帰時の整数秒ズレを除いて
  sub 秒残差だけ残す。これで **N 秒 holdover の真の時刻誤差**が読める (実データ: 25s holdover → 360ns)。
- **補正タイマ**: `true_to_local_ns(true_ns)` で「真の時間で N 待つ」のに必要なローカル ns を得る
  (生の `Timer::after` は水晶公差 +2.7ppm 分ズレる)。これと `query_ns_for_unix_ns` が補正の素。
- **GPSDO PPS 出力 — 2 段階**:
  - ソフト版 (旧 `pps_out_task`): `query_ns_for_unix_ns` で UTC 秒境界をスケジュールし GPIO トグル。
    holdover 対応だが `late` は embassy executor ジッタが下限 (実機 mean ~1.4ms / σ ~244µs) → 廃止。
  - **PIO 版 (現行)**: SM1 が GP3 に GPSDO のパルスを**ハード生成** (周期 = clk×(1+ppb/1e9)-overhead を CPU が
    毎秒 push、`pull noblock` で保持)。エッジは executor 非依存。SM2 が GP4 で**ループバック捕捉** (GP3→GP4
    ジャンパ) し周期を計測。実機 **ジッタ 16ns (= PIO 1 tick、量子化限界)**・GPS PPS と ~24ppb 一致。
    `PPSGEN count= interval_ns= dev_ns=` を出力。ソフト版比 ~1.5 万倍クリーン。
- **GPSDO PPS 出力の UTC 位相同期 (stage ①, ソフト)**: 周波数同期は周期が合うだけで、エッジが UTC 秒境界に
  乗るとは限らない (位相は任意)。位相を合わせるべく gen_capture_task で出力エッジの UTC (Instant 経由) →
  秒境界ズレ `phase_ns` を測り、周期を 1 周だけ伸縮して引き込む。**到達点: 周期 16ns クリーンを保ったまま
  UTC 位相を ~±1〜2ms に整定**。`PPSGEN ... phase_ns=` を出力。制御で踏んだ罠 → 罠 12。
- **時刻系の限界整理**: freq/周期 = 16ns。**ソフト位相同期 = ~±1〜2ms**。pps_task と gen_capture を
  **InterruptExecutor (高優先度割込, SWI_IRQ_0=P0)** に載せ、エッジ→Instant 読みのウェイクアップ遅延を
  ms→µs に下げると一部 ±40µs まで締まるが、~±1.5ms のリミットサイクルが残る (測定の構造ノイズ + 制御の
  相互作用)。**ns/µs の位相同期にはエッジを PIO ハードでタイムスタンプする stage ② が必須**。
  (注: PIO IRQ の優先度を下げると pps_task/gen_capture の捕捉が枯渇するので SWI だけ上げる。罠13)
- **UTC 位相同期 stage ② (PIO ハード測定, 実装済)**: 位相を Instant でなく **PIO の生カウンタ差**で測る。
  GPS PPS は SM0、出力ループバックは SM2 が捕捉。両者は同じ clk_sys なので位相 = `signed_mod(C0_gps −
  C2_out − K)×16ns` (16ns 分解能, Instant の ~ms ノイズ無し)。**K** は起動時に SM2 を一瞬 GP2 に向け SM0 と
  同じ GPS エッジを両方で捕捉して較正 (`set_config` は scratch X を触らないのでピン切替後も K 有効。位相は
  mod 1秒なので K の整数秒誤差は無害)。この ns 測定でコントローラを回すと **±ms → ロック後 σ~460ns
  (sub-µs) で数分間張り付く**。実測比較 (`webapp/plot_compare.py`): 測定が Instant σ~360µs → PIO 16ns
  (~2万×), 出力位相が 旧 Instant 制御 ±1.2ms → 新 PIO 制御 sub-µs。`PPSGEN ... hwphase_ns=` を追加。
  制御源は `PHASE_USE_HW` で PIO/Instant を切替えられる (比較計測用)。
  - 制御ゲインは **k=1/16** (k=1/4 だとループ遅れ d≈2 で ±15µs リミットサイクル)。
  - 周回グリッチの偽キャプチャ → **捕捉プログラムに `jmp low` を足し源で除去** (罠 14)。
  - **長尺で発覚 → smart-friend 相談**: ~300ns 収束後も ~100ms の蹴りを繰り返した。原因は窓際の弱信号で
    GPS PPS が ~19% 欠落 → 古い C0_gps で巨大補正。対処は 2 段 (罠 15): ①GPS 世代が進まないエッジは補正
    スキップ(欠落ホールド)、②**ロック中に 50µs 超の位相は単発 garbage として棄却**(外れ値除去)。これで
    ロック後の蹴りが消え sub-µs 維持。
  - **定常オフセット → type-II PI で解消 (smart-friend 助言)**: P のみ(type-I)だと水晶の残差周波数誤差
    (~数十ppb)に対し補正と漂流が釣り合う「ゼロでない位相」で停まる(ドループ。実測 ~+550ns オフセット)。
    → **位相を周波数トリム ppb_trim に積分する I 項を追加** (`ppb_trim -= ctrl/128`, ロック中のみ, ±3ppm)。
    周期 = clk·(1 + (ppb+ppb_trim)/1e9) − overhead − P。I 項が残差周波数を吸収し**オフセットが 0 に収束**
    (実測 trim が −43ppb に巻き上がり位相が +550ns→~0 へ)。σ は deadband 律速で ~450ns(sub-µs)。
  - **残差振動 → D 項 (PID)**: PI は type-II + ループ遅れ(d≈2)で ~±250ns のゆっくり振動が残る。位相速度
    (`ctrl − last_ctrl`)に比例する **D 項**を追加 (`d_corr = Δctrl/4`, ロック中)。PIO 測定が clean なので D の
    ノイズ増幅問題が無く使える。振動を ~2 割減衰 (σ354→277ns, オフライン sim)。完全には消えない(遅延律速)。
  - **ゲインのオフライン整定 (実機焼かない)**: `PHASE_EXPERIMENT=true` で P→PI→PID を巡回し制御信号を全部
    ログ → `webapp/tune_gains.py` が **実機ログから外乱(系統ドリフト+測定フロア)を逆算** (`dist[n]=Δφ[n] −
    (trim−p−d)[n−D]`, 1ppb=1ns/edge で回帰不要) → 任意ゲインのコントローラを同じ外乱+遅延に通して σ/offset を
    掃引。欠落(ms級)は reject が別処理なので解析外乱から分離。推奨 P=1/8 だが実機 P=1/4 発振歴ありで保守的に
    P=1/16 維持。`webapp/plot_terms.py` は巡回ログを cfg で分け項別に可視化。
    - **罠: 外乱を白色ガウスにすると sim が実機より振動過大**。実機の外乱は隣接 16ns/edge と滑らか(相関あり=
      水晶/GPS は白色でない)。**AR1 相関ノイズ**に直すと実機 PID(σ286) と sim PID(σ249) の滑らかさ・振幅が一致。
      閉ループからの外乱/プラント同定は本質的に難しいので **sim は定性ツール、最終ゲインは実機判断**。
  - **更なる将来**: freq(EMA, GPS 間隔から)と位相 I 項(出力から)を 1 本の type-II ループに完全統合 +
    ループ遅れ d を実測した PI 設計。今は freq=入力同期 / 位相 PID=出力整相 で分離。
- **精度限界の分析 (smart-friend + 実機)**: σ~300ns の支配律速を誤差二乗和で切り分け:
  測定量子化 16ns→σ≈16/√12≈4.6ns、GPS PPS ~10ns、RSS≈11ns vs 観測300ns → **測定+GPS 起因は分散の 0.13%**。
  残り 99.9% は**制御のリミットサイクル**(決定論的振動, ガウスノイズでない)。
  - リミットサイクルは type-II の固有振動: I 単独で `e''=-e/Ki` → 周期 ≈ 2π√Ki = 2π√128 ≈ **71 エッジ**
    (実測 ~75 と一致)、減衰 ζ≈Kp/(2√Ki)≈0.35。trim が ±60ppb 滑らかに振動(1-LSB ディザでない)。
  - 単発では効かなかった: ①周期を sigma-delta 小数 dither + I を milli-ppb 化 (8ppb 量子化除去) →
    リミットサイクル不変 = actuator 量子化が主因でない (分解能改善なので**採用**)。②deadband=0 → ζ0.35 のまま。
    ③Ki=1/512 で ζ0.71 のはず → **遅延 d≈2 が ζ 公式を裏切り改善せず**。④弱信号で σ が 300〜1100ns とブレ
    経験チューニングが収束しない (スパイク 4-10µs が trim を蹴る)。
  - **解決 → σ 300ns→35ns (~9×, mean~0, |max| 80ns)。friend の本命 Smith 予測子が当たり**:
    - **Smith 予測子で遅延補償**: 在飛行中(前エッジに出した、まだ位相に現れてない) P/D 補正を引いた
      **予測位相 `pred = ctrl − last_pd`** で P/I/D を計算 → ループ遅れ d≈2 が消えて ζ 公式が成立。
    - これで **Kp=1/8 (ζ=Kp/(2√Ki)≈0.71)** が本当に効く + **deadband=0** で全域減衰 → trim が ±60ppb 振動から
      **滑らかに整定** (-25ppb 一定)、位相 ±50ns に。
    - **外れ値棄却を 50µs→3µs に**: ロックが ±80ns になったので、弱信号スパイク(4-10µs)を弾けて trim が蹴られ
      なくなった (これが σ を安定させた決め手)。LOCK_NS も 5µs→1µs。
  - **残るフロア**: σ35ns は測定量子化16ns(σ4.6ns)+GPS 10ns の RSS≈11ns に迫る。ここから先は **8ns 捕捉化**
    (overclock/PIO 2SM) が初めて意味を持つ域 + holdover に **TCXO**。**<15ns は GYSFFMANC+窓際の物理壁**
    (timing 受信機 + 良アンテナの別プロジェクト)。実用上は **σ35ns で十分以上**。

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
GPSDO PPS 生成 (SM1) で `pull noblock; mov x,osr; ...; delay: jmp x-- delay` と書くと、`jmp x--` が
**周期を保持しているはずの X を 0/0xFFFFFFFF まで潰す**。次周の `pull noblock` (FIFO 空→`mov osr,x` 仕様)
がゴミを再ロードし、2 発目以降が ~34s 周期に化けた。→ **周期は X に保持、カウントダウンは Y** (`mov y,osr;
jmp y-- delay`)。出力ピンは起動時に `set pindirs, 1` で出力許可する (SET ピン = 生成ピン)。

### 11. 新しい firmware ログ行は「観測経路」にも必ず通す (phantom バグの温床)
PIO の GPSDO 出力のループバック計測で `PPSGEN` が全く出ず、「3 個目の SM(SM2) が壊れている」と何度も
リフラッシュして誤デバッグした。真因は **webapp server の `--log` フィルタ (NMEA/PPS/SYNC/TIME/… の
include リスト) に `PPSGEN` を入れ忘れていただけ** で、firmware は最初から正常に出していた。
→ **新ログ行を足したら server のフィルタ/パースにも足す。疑わしいときは server を介さず
`probe-rs run <elf>` で生出力を直接 grep** して切り分ける (これで一発で判明した)。

### 12. 位相同期の制御ループ: 測定遅れ → 連続フィードバックは発振、平滑 one-shot に
GPSDO PPS 出力の UTC 位相同期で、出力エッジの位相を測って周期を補正する制御を入れたら **±1.5ms で発振**した。
- **原因**: 補正は「1 つ前のエッジで測った位相」に基づき効くのは「次のエッジ」= **1 サンプル遅れ**。
  これに連続比例制御 (ゲイン k) をかけると `λ²−λ+k=0`。k=1 (デッドビート) で発振、k=1/2 でリンギング
  (複素根 \|λ\|=0.707, 偏角45°→周期8ステップ。実測の振動周期とぴったり一致)、**k=1/4 が実根境界=臨界減衰**。
- それでも安定しなかった真因: **位相の「測定」が CPU の `wait_pull` ウェイクアップ遅延 (~ms, executor の
  忙しさで変動) に汚染される**こと。連続フィードバックがこのノイズを追って発振。
- **対処**: 位相を EMA 平滑し、**8 エッジに 1 回だけ one-shot 補正**。連続フィードバックをやめたので発振せず、
  周期も 7/8 は無補正で 16ns クリーン。ただし絶対精度は測定ノイズ律速で **~±1〜2ms** が下限。
- **教訓**: 制御対象 (周期) の精度 (ns) と、フィードバックの「測定」精度 (ms) は別物。測定が悪いと制御で
  そこを超えられない。ns 位相同期は測定をハード化 (PIO で出力エッジを GPS PPS と同じカウンタで捕捉) する以外ない。

### 13. InterruptExecutor は SWI だけ上げる。PIO IRQ を下げると捕捉が枯渇する
位相測定の遅延を下げようと pps_task/gen_capture を高優先割込 (SWI_IRQ_0=P0) に載せたとき、PIO0_IRQ_0 も
P1 に**下げた**ら捕捉がほぼ止まった (PPSGEN が 1 個しか出ない)。PIO IRQ は RP2040 起動時 P0 (最高) なので、
下げると他 (UART 等) に負けてエッジ捕捉のウェイクが枯渇する。**SWI だけ P0 にし PIO IRQ は触らない**。

### 14. PIO 自走カウンタ捕捉の ~68s 周回グリッチを源で消す
低位相ループ `low: jmp pin rising / jmp x-- low` は、X が 0 を通過するとき `jmp x--` が分岐せず `rising` へ
落ちて**偽キャプチャ**する (~68s 毎)。当初は ±1ms フィルタで弾いていたが、stage② の位相制御では garbage
hwphase が出力を蹴って不安定化した。→ `jmp x-- low` の直後に **`jmp low` を 1 行足す**と、X=0 の落下は
そこで捕捉せず low へ戻るので**グリッチが源から消える** (通常の 2cyc/tick は不変)。GPSDO の周波数汚染も同時に解消。

### 15. 弱信号で位相ロックが ~100ms 蹴られる: 欠落ガードだけでは不十分、外れ値除去が要る
stage② の位相ロックが sub-µs に収束した後、長尺 (~10分) で見ると ~100ms の蹴りを繰り返した (69% off-lock)。
窓際の弱信号で **GPS PPS が ~19% 欠落** → C0_gps が 1 秒古いまま → garbage 位相 → 巨大補正で出力が飛ぶ。
- **対処①(欠落ガード)**: pps_task が GPS エッジ世代カウンタを ++。出力エッジ間に進んでなければ補正スキップ
  (断中は freq のみで holdover 自走)。→ だが **これは「欠落」しか防げない**。
- **対処②(外れ値除去, smart-friend 助言)**: 弱信号は「存在するが不正(ジッタ/ハーフ秒スリップ)」な PPS も出す。
  水晶ドリフトは ~数十ns/s なので **ロック中に 50µs/1s も跳ぶ位相は非物理 = garbage**。ロック成立後 (連続で
  小位相) は 50µs 超の位相を単発棄却 (最大 N 回; それ以上続けば本物として再ロック)。→ これでロック後の蹴りが
  消え σ~460ns 維持。**教訓**: GNSS 同期ループの定石は「欠落ホールド + 外れ値棄却 + 補正のスルーレート制限」。
  欠落検出だけでは「受理した garbage」を防げない。

### 16. 周波数 EMA も「Locked のときだけ + 多段ゲート + 復帰検疫」(smart-friend GPT-5.5)
罠 #15 は **位相**ループの garbage 対策。だが **周波数 EMA (GPSDO の土台)** は別経路で、`pps_task` が
`PpsTracker` 判定**前に無条件で** `update_freq` を呼んでいた。ガードは ±1ms の非常停止枠だけ →
**sub-ms の multipath/ハーフ秒スリップ PPS が EMA に素通り**し、holdover と PPSGEN の土台が腐る。GPT-5.5 曰く
「弱点は `update_freq` 自体より、tracker 判定前に無条件更新している構造」。対処を 4 段で入れた:
1. **順序修正**: tracker 判定後、`state==Locked` のエッジでのみ `update_freq`。First/Irregular の間隔は使わない。
2. **品質ゲート** (±1ms 非常停止とは別の通常判定): 未ロック中は絶対 ±100µs、ロック後は `|measured − EMA|` の
   残差 ±5µs (真のジッタは ns〜数十ns 級なので十分甘い安全側)。±1ms は「最後の非常停止」であって通常品質判定ではない。
3. **復帰検疫**: holdover/Irregular→Locked の復帰直後 5 サンプルは EMA 更新を保留 (復帰直後の PPS は受信機内部状態・
   PPS 位相が未整合で信用できない)。**EMA リセットはしない** — 短断なら過去の水晶推定の方が新しい数発より信用できる。
   初回捕捉 (samples==0) では検疫しない (起動時の捕捉を遅らせない)。
4. **状態依存 alpha**: 未ロック収束中は速い `1/8`、ロック後は `1/32`。
- ログ `PPS ... freq=<ok|gate|quar|sane>` で各エッジの採否が見える。実機: 固定窓際の clean 受信では全 `ok`、
  欠落復帰時のみ `quar`×5。`FreqUpdate` enum + host テスト (`gnssdo` の test) でゲート/検疫を網羅。

## 精度指標の意味 (webapp ヘッダ)

- **位置 `X m` = 水平 CEP(50%)**: 直近 ~2 分窓の測位点の経験的ばらつき (50% がこの半径内)。
  詳細パネルに R95(95%)・2DRMS・σE/σN も。**ばらつき(精度=precision)であって真値とのズレ
  (確度=accuracy)ではない**。cold start 収束時のジャンプを除くため直近窓で評価する。
- **時刻 `±X µs` = PPS タイムスタンプのジッタ(1σ)**: PPS 間隔偏差の標準偏差。上記の µs オーダ下限。
  一定レイテンシ(固定オフセット)は σ に出ない。絶対的 UTC 一致はモジュール PPS 確度(数十 ns)に依存。

## モジュール設定 (PMTK)

起動時に PMTK コマンドを UART0 TX(GP0) から `config_task` (RX をブロックしない別タスク) で送る。
TX→モジュール RX 配線時に適用され、各コマンドに `$PMTK001,<cmd>,3`(成功) が返る。**実機で全 ACK 成功を確認**:

- `PMTK886,4` — **dynamic model = stationary** (運用モード)。この GPSDO は固定 timing 用途なので、受信機に
  「動いていない」という強い事前情報を渡す → 弱信号での位置/速度の暴れと PPS 選別への悪影響を抑える。実機 `$PMTK001,886,3`
  成功確認。**移動運用時は `PMTK886,0` (normal) に切替**える (`OpMode` enum で選択。自動切替はしない — 弱信号の NMEA
  speed が嘘をつくとモード切替自体が新たな不安定要因になる)。
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

### 実効 K スリップの再計測 (20260703): エッジ非対称ゼロでも這う

stage-3 (起動時較正のみ、recal なし) を 100 分定点観測すると、gap (scope 実測 − hwphase) は
+187→+424 ns と単調に歩いた (t000-t080、+3.8 ns/min。最終点 t100 はオシロ側の計測不良で棄却)。
この間 PPS の Irregular/missed は **0**。旧仮説「capture エッジ数の非対称 × capture gap (~2 tick) の蓄積」
では説明できない (非対称イベントが無いのに這う)。recal あり 40 分連続 (2402 shot、失敗 0) では
−0.1 ns/min で平坦。

**→ 同日夜の第3 SM 実験 (KEXP) で決着** (`logs/20260703-kexp/`、firmware に SM3=GP2 純観測を追加し
c0/c2/c3 を毎秒 2.4h ログ + オシロ 7802 shot 並走。評価は workflow 4 系統 + 判定):
- **SM 個体差は棄却**: 同じ GP2 を見る SM0−SM3 差 (K_same) は p-p 32ns (2 tick)・傾き 0.000 ns/min で lockstep。
- **dk の大半は「GP4→GP2 切替量子」**: dk は 56 回中 42 回がぴったり −4 tick (98% が −3..−5)、温度と無相関。
  −4 tick/157.5s = −24.4 ns/min が kt の歩きの正体。切替量子は c2 の実在の段として現れ recal が段込みで
  K を測るため hwphase で相殺され、**ピンには一切出ない** (オシロ 2.4h 平坦 −0.006 ns/min、kt と無相関)。
- **実ドリフトは ~3-4 ns/min だけ** (dk のうち ~−0.6 tick 分。stage-3 の +3.8 ns/min と同源とみられる。出所は
  依然未特定だが、recal が吸収しピンは平坦)。旧「dk レートと pin creep の 8 倍乖離」は切替量子+非同時計測の
  見かけで、勘定は全て閉じた (c0−c2=(c0−c3)+(c3−c2) 一致、dk 積算=k 変化 0ns 一致、位相恒等式 100% ビット一致)。
- **→ 翌 20260704 の KPOKE 実験で切替量子の機構まで特定** (`logs/20260704-kpoke/`、純観測 SM3 に
  「同値」書込みを 60s ごと 6 種巡回で打ち、K_same=c0−c3 の段を計測。オシロ並走 1300+ shot):
  - CLKDIV/EXECCTRL/SHIFTCTRL/PINCTRL の単離同値書込みは**全てシロ** (段 0±0.5 tick、n=7 each)。
  - `set_config()` 丸ごとだけがクロ: ×1 で −1〜−2 tick、×4 で −6〜−8 tick。**犯人は embassy set_config 末尾の
    `exec_jmp(origin)`** (use_program 済み config は走行中 SM の PC をプログラム先頭へ強制ジャンプする)。
  - 機構: エッジ直後 (pin high) に飛ぶと先頭の `jmp pin rising` が即 capture 経路へ入り、強制 jmp(1)+jmp pin(1)+
    in(1)+push(1) = 4 cyc 無減算 = **−2 tick/呼び出し** (pin low なら −1)。recal は 2 呼び出しで −4。
    副産物の**偽 FIFO push も決定論的に確認** (full×1 の次エッジは必ず c3n=2、×4 は FIFO 満杯で c3n=4 + 真エッジ push 欠落)。
  - **+33ns 固定オフセットの正体も同機構**: 校正は K を測った「後」に GP4 へ戻すため、戻しの −2 tick (32ns) が
    K に取り込まれず残り、サーボが偏った hwphase を 0 に保つ結果ピンが +32ns 側へ張り付く (オシロ実測 +33.6ns と一致)。
  - **対策実装・検証済み** (`logs/20260704-jmppin-fix/`): 切替 3 箇所 (recal 往復 2 + ブート校正の戻し 1) を
    EXECCTRL の jmp_pin フィールドだけの PAC modify (`switch_jmp_pin`) に置換。1h 検証ラン (recal 23 回 +
    オシロ 3584 shot 並走) で **dk は −4 → −1 が 20/23** (他は 0×2, −2×1。実ドリフト ≈ −6 ns/min のみ、recal が吸収)、
    **オシロ gap 平均は +37.2 → +9.3 ns** (Δ−28ns、+32ns バイアス除去の予言と wander 誤差内で整合)。
    Irregular 0、ロック安定。残る +9ns は配線/プローブのスキューと捕捉経路差とみられる。
    dk=−1/2.6min の実ドリフト (GP2 非同期 vs GP4 自己同期の捕捉オフセットの系統移動、codex 説) は未追跡。
