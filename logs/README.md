# logs/

実機試行の作業ディレクトリ。**コミットしない**(`/logs/*` を gitignore、この README だけ追跡)。理由:

- 大きい(RTT 1本で数十 MB、defmt の生ストリーム)。
- NMEA センテンスに**測位座標**が含まれる。リポジトリに残してはいけない。
- scope を実 IP(`RIGOL_HOST`)で駆動した文脈も混ざりうる。

## 構成: 1 試行 = 1 サブディレクトリ

1 回の試行ごとに `logs/<YYYYMMDD>-<topic>/` を切り、その試行の成果物を全部そこに入れる:

- firmware の RTT 実行ログ(`pps-*.log` = `cargo run` の defmt 出力)
- オシロのモニタ出力・スクショ・GIF(`scope_logger.py` / `scripts/scope_raw.py` の生成物)
- 解析の中間生成物、ログ抜粋など
- **その試行専用の単発スクリプト**(使い捨て。再利用可能になったら `scripts/` へ昇格)

`logs/` 配下は丸ごと無視されるので、サブディレクトリ内は自由に散らかしてよい。
真に再利用可能なスクリプトだけ repo top の `scripts/` に置く(env で機器 IP、引数で入出力パス)。
集計・図化して残すべき結論は `docs/report/`(マスク済みの図 PNG・要約)へ。生ログはここで捨ててよい。

## 使い方

```bash
RUN=logs/$(date +%Y%m%d)-precision-scope; mkdir -p $RUN

# 実機フラッシュ + ログ記録 (古いバイナリを焼かないよう build && run)
cd pico-gnss && cargo build --release && cargo run --release > ../$RUN/pps-boot.log 2>&1

# オシロ (IP は env、リポジトリに直書きしない)。再利用ツールは scripts/ から呼ぶ
RIGOL_HOST=<scope-ip> python3 scripts/scope_raw.py convergence $RUN/convergence.gif 180
```

## レポートの図を生成するスクリプト

以前はここ (`logs/<topic>/`) に `git add -f` でコミットしていたが、いまは
**`docs/report/precision-ladder/logs/<topic>/`** に置く (レポートが参照するものはレポート側に持たせる)。
一覧と実行方法はそちらの [README](../docs/report/precision-ladder/logs/README.md) を参照。
スクリプトが読む生データ (RTT ログ、.shots) は引き続きこの `logs/<topic>/` にあり、gitignore のまま
(再生成には各試行のサブディレクトリが手元に残っている必要がある)。
