#!/usr/bin/env python3
"""dynamic-model A/B (mobile 886,0 vs stationary 886,4) の hwphase 比較。

firmware の `DYNMODEL_AB=true` で 6 分 ABBA 交互送出した capture を解析する。PPSGEN を `dynmode`
セグメントごとに区切り、切替直後の受信機内部フィルタ過渡を捨て、隣接ペア (mobile−stationary) で
hwphase σ を比べる (slow drift を一次相殺)。受信統制 (slope_mppb=純受信ノイズ proxy, jit, rxbad) も
セグメントごとに出す。座標は出さない (PPSGEN に座標は無い)。

  uv run docs/report/analyze_dynmodel_ab.py logs/pps-dynmodel-ab.log [discard_s] [outlier_ns]
"""
import sys
import statistics as st


def fields(line):
    return dict(t.split("=", 1) for t in line.split() if "=" in t)


def pct(vals, p):
    s = sorted(abs(x) for x in vals)
    return s[min(len(s) - 1, int(len(s) * p))] if s else float("nan")


def main():
    path = sys.argv[1]
    discard_s = int(sys.argv[2]) if len(sys.argv) > 2 else 90  # 切替後に捨てる秒数
    outlier = int(sys.argv[3]) if len(sys.argv) > 3 else 5000  # |hwphase| 上限 (gross 除外)

    rows = []
    for line in open(path, errors="replace"):
        if "PPSGEN" not in line or "count=" not in line:
            continue
        d = fields(line)
        try:
            rows.append(
                {
                    "hw": int(d["hwphase_ns"]),
                    "lk": d.get("lk"),
                    "dyn": d.get("dynmode"),
                    "slope": int(d.get("slope_mppb", 0)),
                    "jit": int(d.get("jit", 0)),
                    "rxbad": int(d.get("rxbad", 0)),
                }
            )
        except (KeyError, ValueError):
            continue

    # dynmode の連続ブロックをセグメントに分割。
    segs = []
    cur = None
    for r in rows:
        if cur is None or r["dyn"] != cur["dyn"]:
            cur = {"dyn": r["dyn"], "rows": []}
            segs.append(cur)
        cur["rows"].append(r)

    print(f"# log={path}  discard={discard_s}s  outlier=±{outlier}ns")
    print(f"# segments={len(segs)}  total PPSGEN={len(rows)}")
    print(
        f"{'seg':>3} {'dyn':>4} {'edges':>5} {'kept':>5} "
        f"{'sd':>5} {'p90':>5} {'p95':>5} {'sd_1h':>5} {'sd_2h':>5} "
        f"{'slope_sd':>8} {'jit_med':>7} {'rxbad':>5}"
    )
    seg_stats = []
    for i, s in enumerate(segs):
        rs = s["rows"]
        kept = [r for r in rs[discard_s:] if r["lk"] == "1" and abs(r["hw"]) < outlier]
        hw = [r["hw"] for r in kept]
        if len(hw) < 20:
            seg_stats.append(None)
            print(f"{i:>3} {s['dyn']:>4} {len(rs):>5} {len(hw):>5}  (too few)")
            continue
        h1, h2 = hw[: len(hw) // 2], hw[len(hw) // 2 :]
        slope_sd = st.pstdev([r["slope"] for r in kept]) if len(kept) > 1 else 0
        stat = {
            "dyn": s["dyn"],
            "sd": st.pstdev(hw),
            "p90": pct(hw, 0.9),
            "p95": pct(hw, 0.95),
            "n": len(hw),
            "slope_sd": slope_sd,
            "jit_med": st.median([r["jit"] for r in kept]),
            "rxbad": sum(r["rxbad"] for r in kept),
        }
        seg_stats.append(stat)
        print(
            f"{i:>3} {s['dyn']:>4} {len(rs):>5} {len(hw):>5} "
            f"{st.pstdev(hw):>5.0f} {pct(hw,.9):>5.0f} {pct(hw,.95):>5.0f} "
            f"{st.pstdev(h1):>5.0f} {st.pstdev(h2):>5.0f} "
            f"{slope_sd:>8.0f} {stat['jit_med']:>7.0f} {stat['rxbad']:>5}"
        )

    # 隣接ペア (mobile=0 と stationary=4) の差。各ペアで受信 (slope_sd) が近いことも確認。
    print("\n# adjacent pairs (mobile−stationary): sd_diff, p95_diff, slope_sd ratio")
    diffs, p95diffs = [], []
    for i in range(len(seg_stats) - 1):
        a, b = seg_stats[i], seg_stats[i + 1]
        if not a or not b or a["dyn"] == b["dyn"]:
            continue
        mob = a if a["dyn"] == "0" else b
        sta = a if a["dyn"] == "4" else b
        if mob["dyn"] != "0" or sta["dyn"] != "4":
            continue
        dsd = mob["sd"] - sta["sd"]
        dp95 = mob["p95"] - sta["p95"]
        ratio = (mob["slope_sd"] + 1) / (sta["slope_sd"] + 1)
        diffs.append(dsd)
        p95diffs.append(dp95)
        print(
            f"  seg{i}/{i+1}: sd mob={mob['sd']:.0f} sta={sta['sd']:.0f} dsd={dsd:+.0f}  "
            f"p95 mob={mob['p95']:.0f} sta={sta['p95']:.0f} dp95={dp95:+.0f}  slope_sd_ratio={ratio:.2f}"
        )

    if diffs:
        neg = sum(1 for d in diffs if d < 0)
        print(
            f"\n# SUMMARY pairs={len(diffs)}  median sd_diff(mob−sta)={st.median(diffs):+.0f}ns  "
            f"median p95_diff={st.median(p95diffs):+.0f}ns  mobile_better={neg}/{len(diffs)}"
        )
        # 全体プール (受信を揃えた比較ではないが参考)。
        print("# (negative = mobile has smaller wander)")
    else:
        print("\n# no opposite-mode adjacent pairs yet (need more segments)")


if __name__ == "__main__":
    main()
