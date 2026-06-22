# logs/

firmware の RTT 実行ログ(`pps-*.log` = `cargo run` の defmt 出力)と、オシロのモニタ出力
(`offset-*.log` = scope_pps / offset_wander 系)を置く。

**コミットしない(.gitignore 済み)。理由:**

- 大きい(1本で数十 MB、defmt の生ストリーム)。
- NMEA センテンスに**測位座標**が含まれる。リポジトリに残してはいけない。
- scope を実 IP(`RIGOL_HOST`)で駆動した文脈も混ざりうる。

`.gitignore` は `logs/` 配下を無視し、この README だけ追跡する。

## 使い方

```bash
# 実機フラッシュ + ログ記録 (古いバイナリを焼かないよう build && run)
cd pico-gnss && cargo build --release && cargo run --release > ../logs/pps-$(date +%Y%m%d-%H%M).log 2>&1

# オシロのモニタ (IP は env、リポジトリに直書きしない)
RIGOL_HOST=<scope-ip> python3 report/plot_wander.py logs/pps-XXXX.log report/offset-wander.png
```

集計・図化して残すべき結論は `report/`(図 PNG・要約)へ。生ログはここに置いて捨ててよい。
