# /// script
# requires-python = ">=3.11"
# dependencies = []
# ///
"""単発 (この試行専用): RMC が次の PPS エッジまでどれだけ余裕を持って届いているかを測る。

    uv run --no-project rmc_margin.py <rtt.log>

なぜ測るか: rp-pps は RMC で PPS エッジと UTC 秒を対応付けるが、RMC は NMEA バーストの
終盤にいる。9600bps ではバーストが 0.6 秒以上あり、開始も PPS から数百 ms 遅れるので、
RMC が「次のエッジ」を越えて届くと対応付けが 1 秒ずれる。余裕が薄ければ、衛星数が増えて
GSV が伸びただけで実行時に 1 秒飛ぶことになる。

出力は各 RMC について「直前の PPS エッジからの経過」と「次の PPS エッジまでの残り」。
残りが 0 に近いほど危ない。
"""

import re
import sys
from pathlib import Path

TIME_RE = re.compile(r"^(\d+\.\d+)\s")
PPS_RE = re.compile(r"PPS count=\d+ ")
RMC_RE = re.compile(r"NMEA \$G[NP]RMC")


def main() -> int:
    if len(sys.argv) != 2:
        print(__doc__)
        return 2
    lines = Path(sys.argv[1]).read_text(errors="replace").splitlines()

    edges, rmcs = [], []
    for line in lines:
        m = TIME_RE.match(line)
        if not m:
            continue
        t = float(m.group(1))
        if PPS_RE.search(line):
            edges.append(t)
        elif RMC_RE.search(line):
            rmcs.append(t)

    if len(edges) < 3 or not rmcs:
        print(f"not enough data: {len(edges)} edges, {len(rmcs)} RMC", file=sys.stderr)
        return 1

    margins = []
    for r in rmcs:
        prev = [e for e in edges if e <= r]
        nxt = [e for e in edges if e > r]
        if not prev or not nxt:
            continue
        margins.append((r - prev[-1], nxt[0] - r))

    if not margins:
        print("no RMC bracketed by two edges", file=sys.stderr)
        return 1

    print(f"{'since prev edge':>16} {'until next edge':>16}")
    for since, until in margins[:10]:
        print(f"{since * 1000:>13.0f} ms {until * 1000:>13.0f} ms")

    untils = [u for _, u in margins]
    n = len(untils)
    mean = sum(untils) / n
    sd = (sum((u - mean) ** 2 for u in untils) / n) ** 0.5
    print()
    print(f"n={n}")
    print(f"margin to the next PPS edge: mean {mean * 1000:.0f} ms, sd {sd * 1000:.0f} ms")
    print(f"                             min  {min(untils) * 1000:.0f} ms")
    print()
    print(
        "A small minimum means the RMC-to-edge pairing is one longer NMEA burst away from\n"
        "slipping a whole second — more satellites means more GSV sentences means a later RMC."
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
