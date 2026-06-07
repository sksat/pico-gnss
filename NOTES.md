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
- 注意: X は ~68s で 1 周し、0 通過時に低位相ループで稀に誤キャプチャ (短い外れ間隔) が出る。
  host 側で 1s±50ms 外の間隔を統計から除外している。
- firmware は `PPS ... interval_ns=<ns> ...` を出し、webapp は ns でジッタ σ を表示する。

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

## 精度指標の意味 (webapp ヘッダ)

- **位置 `X m` = 水平 CEP(50%)**: 直近 ~2 分窓の測位点の経験的ばらつき (50% がこの半径内)。
  詳細パネルに R95(95%)・2DRMS・σE/σN も。**ばらつき(精度=precision)であって真値とのズレ
  (確度=accuracy)ではない**。cold start 収束時のジャンプを除くため直近窓で評価する。
- **時刻 `±X µs` = PPS タイムスタンプのジッタ(1σ)**: PPS 間隔偏差の標準偏差。上記の ~9µs 床。
  一定レイテンシ(固定オフセット)は σ に出ない。絶対的 UTC 一致はモジュール PPS 確度(数十 ns)に依存。

## モジュール設定 (PMTK)

起動時に PMTK コマンドを UART0 TX(GP0) から送る (チェックサムは送信時計算)。TX→モジュール RX が
配線されていれば適用される (MediaTek 系チップ前提):

- `PMTK313,1` — SBAS(MSAS) 探索を有効化
- `PMTK301,2` — DGPS 補正源を SBAS に
- `PMTK286,1` — AIC (アクティブ干渉除去)

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
