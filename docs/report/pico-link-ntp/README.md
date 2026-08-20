# Pico 間リンクの時刻同期

Pico 2 台を GPIO で直結し、10BASE-T のロジックだけでフレームをやりとりして、その上で時刻を同期させた回の図。

結論と数値は PR に書いてあるので、ここはその図と、図を作るスクリプトを置く場所である。
生データは repo top の `logs/20260820-pico-link/` にあり、gitignore 配下なのでコミットしない。

## 配線

```
Pico1 GP16 (TX-) ──► Pico2 GP18      Pico1 = server (GNSS 受信機つき)
Pico1 GP17 (TX+) ──► Pico2 GP19      Pico2 = client (GNSS なし)
Pico2 GP16 (TX-) ──► Pico1 GP18
Pico2 GP17 (TX+) ──► Pico1 GP19
GND ───────────────── GND

オシロ  CH1 = GPS-R 1PPS (active low、秒境界は falling edge)
        CH3 = Pico1 GP6
        CH4 = Pico2 GP6
```

## 図

### `fig-scope-5us.png`

3 本を 5 µs/div で見たところ。
黄が GPS 受信機の 1PPS で、この受信機は active low なので秒境界は立ち下がりである。
紫が server、青が client の 1PPS。

client は GNSS を持たず、リンク越しに届く NTP パケットだけから時刻を作っている。

### `fig-broadcast-series.png`

broadcast で同期している間、両方の 1PPS が GPS 受信機の秒からどれだけずれているかを、約 8 分ぶん並べたもの。

client のずれは白色雑音ではなく、3 分ほどの周期で往復している。
これは測定そのものではなく、時刻を追い込むループの応答である。

## スクリプト

`logs/plot_link.py` が `logs/20260820-pico-link/` の CSV を読んで図を書く。

```sh
uv run docs/report/pico-link-ntp/logs/plot_link.py
```

図中に日本語を出すので Noto Sans CJK JP を明示している。
無い環境では豆腐になる。
