# /// script
# requires-python = ">=3.11"
# ///
"""レポート `docs/report/pps-nmea-pairing/` の、送出順とサイクル内タイミングの数値を実測する。

    uv run docs/report/pps-nmea-pairing/logs/pairing/measure_pairing.py

生データは repo top の `logs/20260818-ntp-bringup/` から読む (コミットしない)。

サイクルは「同じ UTC 秒を載せたセンテンスの集まり」として作る。エッジに UTC 秒を割り当てて
から数える手も試したが、9600 bps ではバーストが 1 秒を越えて広がるため、割り当ての基準
そのものが測りたい量 (どのパルスを指しているか) を仮定してしまい、1 秒ぶんずれた答えが
出る。UTC 秒でまとめる限り、順序もサイクル内の間隔も基準の取り方に依らない。

直前エッジからの経過も出す。これも生の観測量で、どのパルスを指すかを仮定しない。
"""

import re
import statistics
import sys
from collections import Counter, defaultdict
from pathlib import Path

REPO = Path(__file__).resolve().parents[5]
RAW = REPO / "logs" / "20260818-ntp-bringup"

TIME_RE = re.compile(r"^(\d+\.\d+)\s")
PPS_RE = re.compile(r"PPS count=\d+ ")
# 先頭に hhmmss を持つ 4 本だけを使う。GLL などは時刻が後ろのフィールドにあり、
# サイクルの同定には要らない。
TIMED_RE = re.compile(r"NMEA \$[A-Z]{2}(GGA|RMC|ZDA|GST),(\d{2})(\d{2})(\d{2})\.")


def load(path: Path):
    edges, sents = [], []
    for line in path.read_text(errors="replace").splitlines():
        m = TIME_RE.match(line)
        if not m:
            continue
        t = float(m.group(1))
        if PPS_RE.search(line):
            edges.append(t)
            continue
        u = TIMED_RE.search(line)
        if u:
            secs = int(u.group(2)) * 3600 + int(u.group(3)) * 60 + int(u.group(4))
            sents.append((t, u.group(1), secs))
    return edges, sents


def main() -> int:
    for name, baud in (("rtt-9600-nmea.log", 9600), ("rtt-picognss-fast.log", 115200)):
        path = RAW / name
        if not path.exists():
            print(f"missing raw log: {path}", file=sys.stderr)
            return 1
        edges, sents = load(path)

        cycles = defaultdict(list)
        for t, kind, secs in sents:
            cycles[secs].append((t, kind))
        orders = Counter(tuple(k for _, k in sorted(v)) for v in cycles.values())
        order, n = orders.most_common(1)[0]

        rel = defaultdict(list)
        for v in cycles.values():
            v = sorted(v)
            for t, kind in v:
                rel[kind].append((t - v[0][0]) * 1000)

        prev = defaultdict(list)
        for t, kind, _s in sents:
            p = [e for e in edges if e <= t]
            if p:
                prev[kind].append((t - p[-1]) * 1000)

        print(f"== {name} ({baud} baud): cycles={len(cycles)} edges={len(edges)}")
        print(f"   送出順 (最頻 x{n}): {' -> '.join(order)}")
        for kind in ("GGA", "RMC", "ZDA", "GST"):
            if kind not in rel:
                continue
            print(
                f"   {kind}: サイクル先頭から {statistics.median(rel[kind]):+5.0f} ms / "
                f"直前エッジから {statistics.median(prev[kind]):5.0f} ms  n={len(rel[kind])}"
            )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
