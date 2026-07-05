# precision-ladder の図を生成するスクリプト

`../precision-figs/` の図を再現するためのスクリプト群。試行 (`<YYYYMMDD>-<topic>`) ごとに
サブディレクトリを切ってある。座標も機器 IP も含まない (計測データではなくコード)。

スクリプトが読む**生データ (RTT ログ、.shots) はコミットしていない**。repo top の
`logs/<topic>/` (gitignore) をそのまま読む設計なので、再生成には該当試行の生データが
手元に残っている必要がある。実行方法は各スクリプト冒頭の docstring (usage 行) を参照。
`20260705-tempff-abab/checkpoint.py` と `longrun.py` は図でなく計測 (オシロ取得) 用で、
その試行のデータをどう取ったかの記録としてここに置いてある。

| スクリプト | 生成する図 |
|---|---|
| `precision-rework/plot_figs.py` | fig1-ladder, fig-naive-phase/jitter, fig-pio-*, fig-pll-*, ctrl, smith, fig2-s4-beforeafter, fig5-bootrepro, fig6-wander-source ほか |
| `precision-rework/fig4_dither.py` | fig4-dither |
| `precision-rework/fig_dither_concept.py` | fig-dither-concept |
| `precision-rework/fig_loopback.py` | fig-loopback |
| `precision-rework/fig_k_offset.py` | fig-k-offset |
| `precision-rework/fig_kexp_setup.py` | fig-kexp-setup |
| `precision-rework/fig789_tempff.py` | fig7-tempff-heat, fig8-tempff-ab, fig9-overreact, fig10-clamp-ab |
| `precision-rework/fig10b_clamp_transient.py` | fig10b-clamp-transient |
| `precision-rework/per_step.py`, `standalone.py` | 補助図 |
| `20260703-naive-boots/analyze_fig.py`, `fig_naive_jitter.py` | 素朴 1PPS の図 |
| `20260703-recal-scope/fig_walk.py` | fig-recal-walk |
| `20260704-jmppin-fix/fig_report.py` | fig-kpoke-poke, fig-kslip-fix |
| `20260704-jmppin-fix/fig_fixresult.py` | fig-fix-result |
| `20260704-drift-cause/fig_wrap.py` | fig-wrap-cost |
| `20260704-drift-cause/fig_stairs.py` | fig-wrap-fold |
| `20260704-drift-cause/fig_report_drift.py` | fig-inject, fig-shadow-march, fig-drift-elim, fig-w900 |
| `20260705-tempff-abab/fig_abab.py` | fig11-tempff-abab |
