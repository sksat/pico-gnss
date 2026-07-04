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

## 追跡している図スクリプト

例外として、`docs/report/precision-ladder/precision-figs/` の図を生成するスクリプトは
再現性のためコミットする (`git add -f`。データではないので座標も IP も含まない)。
生データ (RTT ログ、.shots) は上記の方針どおり gitignore のままなので、再生成には
各試行のサブディレクトリが手元に残っている必要がある。実行方法は各スクリプト冒頭の
docstring (usage 行) を参照。

| スクリプト | 生成する図 |
|---|---|
| `precision-rework/plot_figs.py` | fig1-ladder, fig-naive-phase/jitter, fig-pio-*, fig-pll-*, ctrl, smith, fig2-s4-beforeafter, fig5-bootrepro, fig6-wander-source ほか |
| `precision-rework/fig4_dither.py` | fig4-dither |
| `precision-rework/fig_dither_concept.py` | fig-dither-concept |
| `precision-rework/fig_loopback.py` | fig-loopback |
| `precision-rework/fig_k_offset.py` | fig-k-offset |
| `precision-rework/fig_kexp_setup.py` | fig-kexp-setup |
| `precision-rework/fig789_tempff.py` | 温度ステップ実験の図 (fig7/8/9) |
| `precision-rework/fig10_clamp.py` | fig10-clamp |
| `precision-rework/per_step.py`, `standalone.py` | 補助図 |
| `20260703-naive-boots/analyze_fig.py`, `fig_naive_jitter.py` | 素朴 1PPS の図 |
| `20260703-recal-scope/fig_walk.py` | fig-recal-walk |
| `20260704-jmppin-fix/fig_report.py` | fig-kpoke-poke, fig-kslip-fix |
| `20260704-jmppin-fix/fig_fixresult.py` | fig-fix-result |
| `20260704-drift-cause/fig_wrap.py` | fig-wrap-cost |
| `20260704-drift-cause/fig_stairs.py` | fig-wrap-fold |
| `20260704-drift-cause/fig_report_drift.py` | fig-inject, fig-shadow-march, fig-drift-elim, fig-w900 |
