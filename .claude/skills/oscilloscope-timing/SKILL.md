---
name: oscilloscope-timing
description: >-
  オシロスコープで2つの信号のあいだのエッジ時間差(位相/オフセット/スキュー/遅延)を測る、その時間差の
  ジッタやドリフトを見る、1PPS どうしの整合を測る、Rigol を SCPI(LAN)でスクリプト計測する、波形を吸い
  出してエッジ時刻の差を出す、ときは必ずこの skill を参照する。「CH1 と CH2 のエッジ差」「2つのパルスが
  どれだけずれてる」「PPS の位相/整合」「立ち上がりの時間差/遅延/スキュー」「GPSDO 出力 vs 基準のずれを
  ベンチで実測」「オシロ計測の python」「(2信号の時間差計測で)トリガがかからない/プローブが原因か」
  「オシロで測った時間差は本物か」の文脈なら、明示的に "oscilloscope" と言わなくても発火する。自力で
  答えられそうでも参照すること: 素の知識では誤りやすいハード固有の罠(内蔵 delay/RDELay は幅の広いパルスで
  エッジをペアにできず失敗、×1/×10 プローブのスキューが固定オフセット、単発値はドリフトの1点)と実証済み
  ヘルパ scripts/rigol_scpi.py を含むため。発火しない(対象外。いずれも「2信号のエッジ時間差」ではない):
  単一信号の周波数・デューティ・電圧・波形をオシロで見るだけ、ロジックアナライザやプロトコル(SPI 等)の
  デコード、ファンクションジェネレータでの波形出力、スペアナ、TDC/周波数カウンタ、アイダイアグラム、
  phase noise や Allan 偏差などの周波数安定度解析、回路シミュレーション。
---

# オシロスコープでのタイミング/位相測定 (Rigol DHO800, SCPI)

2つの信号の立ち上がりエッジの時間差(位相/オフセット/スキュー)を、Rigol DHO800 を LAN 経由 SCPI で
駆動して測る。バンドルした `scripts/rigol_scpi.py` が接続/単発取込/エッジ検出/統計までやる。

この skill の価値の半分は計測手順、もう半分は **「測った数字を信じてよいか」の作法** にある。実機の時間差は
たいてい揺れる量なので、測り方を誤ると簡単に偽の結論が出る。下の「落とし穴」を先に読むこと。

## 接続

scope の IP は環境変数 `RIGOL_HOST` から取る。**IP をコミットされるファイルに直書きしない**(機器の所在は
構成固有の情報で、リポジトリに残すべきでない)。raw TCP の既定ポートは 5555(`RIGOL_PORT` で変更可)。

```bash
RIGOL_HOST=<scope-ip> python3 scripts/rigol_scpi.py phase <ref_ch> <sig_ch> [N] [log]
RIGOL_HOST=<scope-ip> python3 scripts/rigol_scpi.py capture <out.png> [s/div]
```

スクリプトを呼ばず自分で SCPI を組むなら `Rigol` クラス(`send`/`query`/`query_block`/`waveform`/
`single`/`drain_errors`)を import して使う。

## 核レシピ: 2チャネルのエッジ時間差

手順は「基準チャネルのエッジでトリガ → 両チャネルの波形を吸い出す → それぞれの立ち上がり交差点を求める →
差を取る」。これを N 回単発取込して **分布(mean/sigma/pp)** で報告する。`measure_edge_offset()` /
`rigol_scpi.py phase` がこれを実装している。

```
RIGOL_HOST=192.0.2.10 python3 scripts/rigol_scpi.py phase 1 2 60 /tmp/off.log
# CH1=基準(トリガ), CH2=測りたい信号。60 shot。1行/shot で /tmp/off.log にも出す。
# -> xinc=... ns/pt  N_ok=58/60
#    offset(ch2-ch1): mean=-2803ns sigma=104ns min=... max=... pp=500ns
```

**なぜ内蔵の delay/period 測定でなく波形を吸い出すのか**: 幅の広いパルス(例: ~100ms の 1PPS)は µs 窓に
エッジが1本しか映らず、内蔵のエッジペアリングが成立しない。波形を取って自分で交差点を求める方が堅い。

**なぜ分布で報告するのか**: 下の落とし穴の筆頭。

## 落とし穴(ここが本体)

### 1. 揺れる量を単発スナップショットで判断しない

時間差は温度/受信機の量子化 sawtooth/源の揺らぎでドリフトする。**1発の値はその分布からのランダムな1点**で
あって「オフセット」ではない。同じセットアップでも、ある1発は +2.1µs、15発平均では 244ns、という乖離が
普通に起きる。

- 必ず **密に多 shot(数秒間隔 × 数十発以上)** 取り、mean/sigma/pp と、必要なら時系列(ドリフトの向き)で見る。
- 2つの計測系(例: 機器の自己計測 vs オシロ)を**ドリフト中に緩く時刻整合して比べると、「測った時刻のズレ ×
  ドリフト速度」が見かけの系統オフセットを作る**。比べるなら密な時系列で分布として突き合わせる。
- 「改善を観測した」と「機構を確認した」は別物。新しい計測で failure を**再現**するまで「直った」と言わない。

ユーザが「1回測ったら Xµs だった、これは本物?」と聞いてきたら、まずこの点を確認する(単発では判断できない、
密な多 shot を取る)。

### 2. プローブ減衰とチャネルスキュー: 固定オフセット ≠ 物理

2 本のプローブの減衰比が違う(片方 ×1、片方 ×10 など)と、**プローブ自身の伝搬遅延差 + チャネルスキューが
固定の時間差として乗る**。これは数 ns から数十 ns になり得て、被測定の物理オフセットではない。

- 配線が短い(数 cm)なら経路の伝搬遅延は sub-ns で無視できる。それでも mean に数十 ns の定数が残るなら、
  **プローブ/チャネルスキューを疑う**。両プローブを同じ1点に当てて差を測れば、その固定スキューが直接出る。
- つまり `mean` は「物理オフセット + 未較正の固定スキュー(プローブ/ケーブル/チャネル)」。絶対値を物理量と
  断定する前に、スキュー分を切り分けるか、最低限その存在を明示する。

### 3. トリガレベルはプローブ減衰の後で効く

**物理 ×10 プローブを使っているのにチャネルの `:CHANnelN:PROBe` が ×1 のまま**だと、scope は減衰を戻さず
3.3V のパルスを 0.33V と表示し、1.65V のトリガレベルを跨がず**永遠にトリガしない**。`:CHANnelN:PROBe` を
実際のプローブ比(×10 等)に合わせ、トリガレベルは「画面に出る電圧」で決める。トリガしない/エッジが取れない
ときはまずここを疑う。

### 4. timebase が分解能を決める

時間差の分解能は s/div(と記録長)で決まる。`:WAVeform:XINCrement?` が ns/sample を返すので、これを必ず
読んで換算に使う。粗い窓のまま測ると量子化で sigma が水増しされる。収束後は s/div を詰める。なお内蔵の
位相/遅延測定コマンドや一部の自動レンジは **timebase を内部で書き換える** ことがあるので、レンジを固定したい
なら明示設定して自分で交差点を求める。`measure_edge_offset` は **現在の s/div の XINCrement をそのまま読む**
(timebase を変えない)。細かく測りたいなら、事前に scope の s/div を詰めるか `timebase_s=` 引数で渡す。
CLI の `phase` は現在の s/div で測る。

**ただし wander する信号を連続ログするときは「詰める」と逆効果**: trigger 中央の基準エッジに対し信号
エッジが振れて画面外に出ると、その shot だけ交差点が取れず落ちる。これは単なる取りこぼしでなく、**大きく
振れた瞬間を選択的に捨てるので sigma を下に偏らせる**(50ns/div=±250ns 窓で σ≈130ns と出たが、
200ns/div に広げたら同じ信号が σ≈165ns と PIO 実測 ~196ns に寄った)。分解能と窓幅はトレードオフ:
収束量を細かく測るなら詰める、揺れ幅そのものを正しく測るなら **±3σ を収める窓**にする(hwphase σ≈200ns
なら 200ns/div)。連続ログの ok 率が低い・σ が他計器より小さいときは、まず窓幅でクリップしていないか疑う。

### 5. ground-truth 側の配線/プローブ自体を疑う

オシロは「接地真値」だが、その手前の配線やプローブ自体が偽の値を作る。今回 µs オーダーの偽アーティファクトの
正体は**緩んだプローブ**だった(接触不良で別物を拾っていた)。妙な値が出たら被測定より先に、プローブの嵌合/
グランド/減衰設定を確認する。µs 級のずれはまず、トリガ条件、AC/DC coupling、遅いエッジ、別エッジの捕捉、
誤配線を疑う(グランドリードのインダクタンスで効くのは ns 級のリンギングで、µs ではない)。

### 6. DHO800 固有の癖

- SCPI エラーキューは FIFO。コマンドを疑う前に `drain_errors()` で全部吐かせる。
- `:MEASure:CLEar` は **引数を取らない**(`ALL` を付けると -108 Parameter not allowed)。
- 波形ブロックは IEEE-488.2 の `#<n><len><payload>`。`query_block()` が処理する。

### 7. 1PPS(低レート)では sweep=NORMal。状態語と生バイトで「信号なし」と即断しない

実際に踏んだ罠(誤診の連鎖)。1PPS のような **1Hz の低レート**信号を見るときは:

- **sweep は必ず `NORMal`**(エッジでだけトリガ)。`AUTO` にすると 1 秒のエッジを待たず自動トリガして
  **未同期の波形**を出し続ける(「トリガかかってない」ように見える)。逆に「トリガが効いてない?」の第一手は
  `:TRIGger:SWEep?` を見て AUTO を NORMal に直すこと。
- **`:TRIGger:STATus?` の `WAIT` を「不発」と誤読しない**。NORM + 1Hz では 1 秒のほとんどが `WAIT`(エッジ待ち)で、
  発火は一瞬 `TD`/`STOP`。状態語を数回読んで WAIT だらけでも正常。**発火しているかは `:SINGle` が STOP を返すか**で
  判定する(`Rigol.single()`)。N 回回して N/N 返れば確実にトリガできている。
- **`:MEASure:ITEM?` が `9.9E37` = 「測定無効」であって「信号なし」ではない**。新鮮な triggered acquisition が
  無い(NORM で待機中、または 1Hz を広い timebase で測る)と無効値になる。VPP/VMAX が無効でも信号は在りうる。
- **垂直 offset を間違えると生バイトが偽る**。`:CHANnel:OFFSet 0`(scale 1V/div)だと 3.3V が画面上端で clip し、
  生バイトが全 255(railed)や平坦に見えて「信号なし」と誤判定する。0–3.3V がちゃんと画面内に入る offset
  (例: 1V/div で offset −1.5V)にしてから振幅を見る。
- 結論: 「トリガしてない/信号がない」と言う前に、**正しい垂直(信号が画面内)+ `single()` の発火回数 + 生波形の
  swing(BYTE)**で事実を取る。`drain_errors()` と同じで、checker(single の発火と swing)が真。

### 8. probe ratio は物理結線に合わせる。CH 間で絶対電圧が食い違ったら信号でなく設定/結線を疑う

実例(誤診→是正): CH2(GP4, 10× プローブ)が settled 3.28V と正しいのに、CH1(GPS PPS)だけ settled
**5.80V** を出した。「GPS PPS は 5V 系で Pico 直結が危ない?」と早合点しかけたが、真因は **probe ratio と
物理結線のミスマッチ**だった:

- **CH1(GPS PPS)は 1× 直結**(直の同軸タップ)なのに、自己設定スクリプトが `:CHANnel1:PROBe 10` を
  押し込んでいた。すると 3.3V × 10 = 33V で画面上端を突き抜け、**生バイトが railed**(罠 #7 と同じ)。
  byte→volt 変換は真値でなく**表示レンジの天井(~5.8V)**を返し、それを settled と誤読した。
- 直し方: probe ratio を**物理結線に合わせる**。直結なら 1×、10× プローブなら 10×。CH1 を 1× に戻すと
  ちゃんと **3.277V**(=3.3V)を表示した。
- データシートでの裏取り: **秋月 AE-GNSS-EXTANT(GYSFFMANC / MT3333)は VCC 3.8–5V だが IO 電圧 2–3.6V**
  (I/O は内部 regulate の 3.3V 系)。PPS High は ~3.3V、5V には追従しない → 5.8V は信号ではありえない。
  RP2040 GPIO abs-max = IOVDD+0.5 ≒ 3.8V > モジュール IO max 3.6V なので **GP2 直結で OK**(過電圧無し)。
- 一般化: **CH 間で絶対電圧が食い違ったら、まず probe ratio が物理結線と一致しているか**を見る。確定法は
  **両プローブを同じ既知ノードに当てて**同電圧を返すか、または **1× で生 BNC 電圧**を読んでデータシートの
  IO レベルと突き合わせる(`scope_autoscale.py` / `scope_wander.py` は CH1=1×, CH2=10× を毎回固定する)。
- 注意: このスケール誤差は**タイミング/位相**には効かない(byte が railed でも立ち上がりの mid クロスは拾え、
  firmware の hwphase は PIO 入力しきい値で測りオシロと無関係)。直すのは**絶対電圧のクロスチェック精度**のため。

### 9. 記録(history)モードのフレームタイムスタンプは粗い。ns エッジを秒スパンで測るのは TIC の仕事

「出力 PPS と GPS PPS を**それぞれ**オシロの自前時計に対して測れば、差分(両者とも同一受信機相対)でなく
どちらが揺れているか分離できるのでは」は原理的に正しい。が、**オシロでは届かない**ことが多い:

- DHO800 の波形記録(`:RECord:WRECord:OPERate RUN`、`:RECord:WREPlay:FCURrent:TIME?` でフレーム時刻)を
  試したが、**タイムスタンプは ~10ms 分解能**(`1.28217ks` のような文字列、最小桁 0.01s)で、しかも 1Hz の
  NORM トリガにロックせず free-run 気味だった。履歴閲覧用の大まかな採取時刻であって精密トリガ時刻ではない。
  ~100〜600ns の位相 wander には 5 桁足りず使えない。
- 本質的な理由: 立ち上がりが速い(ns)エッジを **ns 分解能で・かつ秒オーダーのスパン**でタイムスタンプするには
  G サンプルのメモリが要る。オシロは「ns 分解能」か「秒スパン」の片方しか持てない(深メモリでも
  sample rate↔record length のトレードオフで、PPS エッジが 1 サンプル未満になり交差点が取れない)。
- 結論: **各エッジを自前基準に対して ns で連続タイムスタンプするのは TIC/TDC(時間間隔カウンタ)の仕事**。
  オシロが得意なのは「2 信号の**差**(output−ref)を 1 ショットで ns」まで。第3時計アトリビューション
  (出力の真の揺れ vs 基準の揺れの分離)が要るなら、TIC か 2 台目基準を使う。オシロ単独では差分止まり。

## 結果の読み方

- **mean**: 平均オフセット。物理オフセット + 固定スキュー(落とし穴2)。符号は `sig - ref`(負 = sig が先行)。
- **sigma**: shot 間ばらつき。源のジッタ/受信機の sawtooth/トリガ量子化/温度ドリフトが混じる。s/div を詰めると
  量子化分は減る。残るのは源の素の揺れ。
- **pp**: 外れ値に敏感。エッジが画面端に来た shot は `measure_edge_offset` が落とすが、ドリフトで時々跨ぐと pp が膨らむ。

可視化が要るなら、`phase ... <log>` の 1行/shot ログを matplotlib で時系列 + ヒストグラムにする(ドリフトと
分布が一目で分かる)。

### 9.5 `*RST` のあとは結合を明示する。エッジが消えて「信号が無い」ように見える

`*RST` や別の実験の残りでチャネルの結合が AC になっていると、**10% duty の 1PPS は DC 成分が
抜けて 0 V 付近の平らな線**になり、エッジが画面から消える。板は正しく出力していて、firmware の
ログも同期を示しているのに、そのチャネルだけ「信号なし」に見える(実際に踏んだ: client の GP6 が
`funcsel 7`、duty 10%、NTP の残差 8 µs で動いているのに CH4 が平坦だった)。

- setup で `:CHANnelN:COUPling DC` を毎回押す。probe ratio と同じで、**依存するものは明示する**。
- 1 本だけエッジが取れないときは、被測定より先に結合を疑う(次がプローブの嵌合、罠 #5)。
- `*RST` は画面レイアウトも変えるので、スクリーンショットに文字を焼き込んでいる場合は
  重ねる座標も見直す。

### 10. VXI-11 read が完全に死ぬことがある → raw socket 5555 にフォールバック(lazy-flush 地獄)

VXI-11(`scripts/rigol_vxi11.py`)は基本だが、**read が完全に死ぬ**ことがある(`device clear` は成功するのに
`ask()` の read が毎回 IO timeout。backlog 排出でも復旧せず、その session では VXI-11 が使えない)。このとき
raw socket 5555 は生きているが SCPI が **lazy-flush** で厄介:

- コマンドを受け取ると「それ以前に pending だった応答を**全部** socket に push」し、**自身の応答は次コマンドまで
  pending**(素朴な read は1つ遅延。drain しても pending は wire 上に無いので出てこない)。
- `*CLS` は応答を持たない pusher。**text 応答は flush するが、pending の波形 block / measurement は ABORT する**。
  だから瞬時の text query は `send(cmd); send("*CLS"); readline()` でOK、**block は実 query(`:WAV:FORMat?`)を
  pusher** にして flush(abort されない)→ skip-to-`#`。block の前に `drain` して設定が flush した stale text を除く。
- **取得中は *CLS を使わない**(捕捉済みフレームを消す。:SINGle の status polling に混ぜると顕著)。毎 shot は
  `:SINGle` + 固定待ち(>1 PPS 周期 ~1.4s)で独立フレームを1枚。**トリガ発火は ch1(=トリガ源)のエッジ有無で
  検証**(status polling より安全。空フレーム=トリガ未発火を弾ける)。RDELay 等の delay measurement は raw では
  返らないので**波形 download + 自前エッジ検出**。
- 実装は `scripts/scope_raw.py`(`RawScope` + CLI `phase`/`shot`/`convergence`/`wander`)。`rigol_vxi11.py` が死んだら此方。
- **統合 GIF**: `scripts/scope_combo.py <rtt_log> <out.gif> [dur] [sdiv_ns]` は上段=オシロ波形(毎PPS)、
  下段=firmware パラメータ時系列(scope offset / hwphase_ns / ppb / temp_raw を毎PPSで成長プロット、現在点●)
  を1枚に重ねる。scope は RUN-grab-dedup で毎PPS、パラメータは **live RTT ログ(`cargo run` の出力)を tail** して
  最新 PPSGEN/TIME 値を snapshot し wall-clock で同期(両方 live なので sub-second で揃う)。`cargo run > <rtt_log>`
  を並走させて使う。y 軸は expanding(縮まない)でアニメのジッタを防ぐ。

## 波形/スクリーンショットの記録

`capture <out.png> [s/div]`(VXI-11)/ `scope_raw.py shot <out.png> [sdiv_ns]`(raw)で1発トリガしてスクショ保存。
`Rigol.waveform()` / `RawScope.waveform()` で生サンプル(BYTE)を吸い出し自前解析もできる。**作法**:

- **時間スケールは実現精度に合わせる**。数十 ns の整合を 500ns/div で撮ると「重なって見える」だけで何も分からない。
  20〜50ns/div(達成 σ の ~1div 相当)まで詰める。粗いと精度が見えず、細かすぎると wander でエッジが画面外に出る。
- **スクショ/GIF では基準でない冗長 ch を切る**。ここでは GP2(ch3, 受信機@Pico)は ch1(受信機源)とほぼ同一で、
  かつ小信号で立上りが緩く ringing → エッジ検出も暴れる。**ch3 を OFF にして ch1(GPS基準)vs ch2(出力)の2本**に
  すると見やすく、しかも ch1(クリーン&トリガ源)基準にすると σ が大きく改善する(実測 std 84→36ns)。
- **GIF は「毎トリガ独立フレーム」**。`:SINGle`+トリガ検証(ch1 エッジ有無)で空フレームを混ぜない。ただし
  `:SINGle` の直列 floor は「1トリガ(≤1s)+読み出し(PNG ~0.5s + 波形 ~0.24s×n)」≈ 1.5-2s で、**1 PPS=1frame
  には届かない**(エッジを1つ飛ばす)。
- **真の毎 PPS GIF は NORMal sweep で RUN したまま最速 grab + dedup**(`scope_raw.py wander`)。RUN 中は scope が
  毎 PPS 自動トリガし最新フレームを保持。grab(~0.4s)< PPS周期(1s)なので各 PPS を 2-3 回拾う=取りこぼし無し。
  **ch2 波形バイトのハッシュで dedup** して 1 PPS=1 フレームにする(offset が連続推移すれば取りこぼし無しの証拠)。
  `:SINGle`/status polling/RECORD モードは raw socket では脆い・不明瞭(polling は *CLS でフレーム消失 or desync、
  DHO804 の RECORD コマンドツリーは要マニュアル)。RUN-grab-dedup が一番素直で確実。
- **再起動後チェックは起動直後から撮る**。`scope_raw.py convergence <out.gif> [dur]` は boot から PLL 引き込みを
  **毎 PPS**(上の RUN-grab-dedup)で撮り、**per-frame 自動タイムスケール**で「大オフセット→収束」を可視化する
  (実測 +14〜36µs→±数十ns)。スケールは **1-2-5 ラダーを 1フレーム1ノッチずつ**緩やかに動かす(急な decade 飛びは
  見づらい)。エッジが画面外なら素早く widen して見失わない。timebase を変えたら **新トリガを1つ待ってから**採用
  (旧スケールのフレーム取り違え防止)。xinc はクエリせず `sdiv*1e7`(NORMal は ~1000pt/10div)で計算(block 後の
  `:WAV:FORMat?` 応答と desync しないため)。毎 PPS×長尺はフレーム膨大(600s=600枚)なので既定 ~180s(引き込み+定常、
  ~80-150枚)、frames は 400 で cap。

## 他機種への移植

`Rigol` クラスの socket/SCPI の骨格(改行終端の send、`:SYSTem:ERRor?` のドレイン、定義長ブロック読み)は
汎用。機種依存なのはコマンド文字列:

- **波形吸い出し**: `:WAVeform:MODE`(NORMal/RAW/MAX)と `:WAVeform:FORMat`(BYTE/WORD)、ブロックの
  プリアンブル `:WAVeform:PREamble?` の扱いは機種で違う。DS1000Z 等は RAW モードで深メモリを分割読みする。
- **トリガ/タイムベース**: `:TRIGger:EDGE:*`、`:TIMebase:MAIN:SCALe/OFFSet`、`:SINGle`/`:TRIGger:STATus?` は
  概ね共通だが応答語が違うことがある。
- **接続**: LAN raw TCP(5555)以外に USBTMC/VISA を使う機種なら `pyvisa` に差し替える(クラスの公開メソッドは
  そのままにすると呼び出し側が変わらない)。
- まず `drain_errors()` を信じる(checker is truth): コマンドが効かないときは推測せずエラーキューを読む。

## バンドル

- `scripts/rigol_scpi.py`: `Rigol`(SCPI クライアント)、`rising_edge`(交差点検出)、`measure_edge_offset`
  (密多 shot + 統計)、CLI(`phase`/`capture`)。stdlib のみ(可視化する場合だけ matplotlib/numpy)。
