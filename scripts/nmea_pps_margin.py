# /// script
# requires-python = ">=3.11"
# dependencies = []
# ///
"""NMEA の時刻センテンスが、次の PPS エッジまでどれだけ余裕を持って届いているかを測る。

    uv run --no-project scripts/nmea_pps_margin.py <rtt.log> [--sentence RMC|ZDA]

# なぜ測るか

`rp-pps` は PPS エッジと UTC 秒を「エッジ + 直近の時刻センテンス」で対応付ける。センテンスは
NMEA バーストの終盤にいるので、バーストが長いと**次のエッジを越えて**届き、対応付けが 1 秒ずれる。
しかも位相しか見ていない限りこれは見えない — 規律 1PPS は GPS エッジ上に ns で乗ったまま、違う秒の
ラベルを付ける。

余裕が小さいほど危ない。実測例:

    9600 bps    mean 490ms, sd 460ms, min   2ms   (エッジの前後に割れた二峰 = コイン投げ)
    115200 bps  mean 749ms, sd  26ms, min 718ms   (エッジから遠い一峰 = 余裕)

sd が大きく min が 0 に近ければ、衛星数が増えて GSV が伸びただけで実行時に 1 秒飛ぶ状態にある。

# 入力

firmware の RTT ログ (先頭が `<uptime秒> ` の defmt 行)。`PPS count=` 行と `NMEA $G..<sentence>`
行の時刻を使う。どちらも出す firmware が要る (`pico-gnss` はどちらも出す)。
"""

import argparse
import re
import sys
from pathlib import Path

TIME_RE = re.compile(r"^(\d+\.\d+)\s")
PPS_RE = re.compile(r"PPS count=\d+ ")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("log", type=Path, help="firmware の RTT ログ")
    ap.add_argument(
        "--sentence",
        default="RMC",
        help="対応付けに使う時刻センテンス (既定 RMC。ZDA など)",
    )
    ap.add_argument("--show", type=int, default=10, help="先頭何件を並べるか")
    args = ap.parse_args()

    sentence_re = re.compile(rf"NMEA \$G[A-Z]{{1,2}}{re.escape(args.sentence.upper())}")

    edges, sentences = [], []
    for line in args.log.read_text(errors="replace").splitlines():
        m = TIME_RE.match(line)
        if not m:
            continue
        t = float(m.group(1))
        if PPS_RE.search(line):
            edges.append(t)
        elif sentence_re.search(line):
            sentences.append(t)

    if len(edges) < 3 or not sentences:
        print(
            f"not enough data: {len(edges)} PPS edges, {len(sentences)} {args.sentence} "
            f"— does this firmware log both?",
            file=sys.stderr,
        )
        return 1

    margins = []
    for s in sentences:
        prev = [e for e in edges if e <= s]
        nxt = [e for e in edges if e > s]
        if prev and nxt:
            margins.append((s - prev[-1], nxt[0] - s))

    if not margins:
        print(f"no {args.sentence} bracketed by two PPS edges", file=sys.stderr)
        return 1

    print(f"{'since prev edge':>16} {'until next edge':>16}")
    for since, until in margins[: args.show]:
        print(f"{since * 1000:>13.0f} ms {until * 1000:>13.0f} ms")

    untils = [u for _, u in margins]
    n = len(untils)
    mean = sum(untils) / n
    sd = (sum((u - mean) ** 2 for u in untils) / n) ** 0.5
    print()
    print(f"n={n}  sentence={args.sentence.upper()}")
    print(f"margin to the next PPS edge: mean {mean * 1000:.0f} ms, sd {sd * 1000:.0f} ms")
    print(f"                             min  {min(untils) * 1000:.0f} ms")
    print()
    print(
        "A small minimum, or a standard deviation approaching half a second, means the pairing is\n"
        "one longer NMEA burst away from slipping a whole second — more satellites means more GSV\n"
        "sentences means a later time sentence."
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
