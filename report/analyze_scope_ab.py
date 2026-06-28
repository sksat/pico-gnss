#!/usr/bin/env python3
"""dynamic-model A/B の **scope 側** (output−GPS, 独立計器) を dynmode セグメントで層別する。

firmware hwphase の A/B (`analyze_dynmodel_ab.py`) と対で、同じ A/B を独立計器 (Rigol) で裏取りする。
scope ログ (scope_logger.py 出力: wall_time offset_ns) を、firmware ログの dynmode セグメント
(各 PPSGEN 行の RTT 時刻 + dynmode) に wall↔RTT 橋で対応づけ、scope offset を mobile/stationary に層別。
切替直後を捨て、隣接ペア (mobile−stationary) で scope σ を比べる。座標は読まない。

  uv run report/analyze_scope_ab.py logs/pps-dynmodel-ab.log logs/scope-dynmodel-ab.log [discard_s]
"""
import re
import sys
import statistics as st


def main():
    fw_path, scope_path = sys.argv[1], sys.argv[2]
    discard_s = int(sys.argv[3]) if len(sys.argv) > 3 else 90

    # firmware: (rtt, dynmode) from PPSGEN lines.
    seg_pts = []
    for line in open(fw_path, errors="replace"):
        if "PPSGEN" not in line:
            continue
        m = re.match(r"\s*([\d.]+)\s", line)
        d = dict(t.split("=", 1) for t in line.split() if "=" in t)
        if not m or "dynmode" not in d:
            continue
        seg_pts.append((float(m.group(1)), d["dynmode"]))
    if len(seg_pts) < 50:
        print("not enough firmware points")
        return
    # contiguous dynmode segments: (rtt_start, rtt_end, dynmode)
    segs = []
    s0, dyn = seg_pts[0]
    prev = s0
    for rtt, dm in seg_pts[1:]:
        if dm != dyn:
            segs.append([s0, prev, dyn])
            s0, dyn = rtt, dm
        prev = rtt
    segs.append([s0, prev, dyn])

    # scope: bridge const + shots.
    bridge = None
    for l in open(scope_path, errors="ignore"):
        mm = re.search(r"RTT = wall - ([\d.]+)", l)
        if mm:
            bridge = float(mm.group(1))
            break
    if bridge is None:
        print("no wall↔RTT bridge in scope log header; cannot align")
        return
    shots = []
    for l in open(scope_path, errors="ignore"):
        if not l[:1].isdigit():
            continue
        p = l.split()
        if len(p) == 2:
            shots.append((float(p[0]) - bridge, float(p[1])))  # (rtt, offset_ns)

    print(f"# fw segments={len(segs)} scope shots={len(shots)} bridge={bridge:.1f} discard={discard_s}s")
    print(f"{'seg':>3} {'dyn':>4} {'dur_s':>6} {'scope_n':>7} {'sd':>5} {'mean':>6}")
    stats = []
    for i, (a, b, dm) in enumerate(segs):
        sv = [o for (rtt, o) in shots if a + discard_s <= rtt <= b]
        if len(sv) < 15:
            stats.append(None)
            print(f"{i:>3} {dm:>4} {b-a:>6.0f} {len(sv):>7}  (few)")
            continue
        stats.append({"dyn": dm, "sd": st.pstdev(sv), "mean": st.mean(sv), "n": len(sv)})
        print(f"{i:>3} {dm:>4} {b-a:>6.0f} {len(sv):>7} {st.pstdev(sv):>5.0f} {st.mean(sv):>6.0f}")

    print("\n# adjacent pairs (scope σ, mobile−stationary)")
    diffs = []
    for i in range(len(stats) - 1):
        x, y = stats[i], stats[i + 1]
        if not x or not y or x["dyn"] == y["dyn"]:
            continue
        mob = x if x["dyn"] == "0" else y
        sta = x if x["dyn"] == "4" else y
        if mob["dyn"] != "0" or sta["dyn"] != "4":
            continue
        dsd = mob["sd"] - sta["sd"]
        diffs.append(dsd)
        print(f"  seg{i}/{i+1}: scope σ mob={mob['sd']:.0f} sta={sta['sd']:.0f} dσ={dsd:+.0f}ns")
    if diffs:
        neg = sum(1 for d in diffs if d < 0)
        print(
            f"\n# SCOPE SUMMARY pairs={len(diffs)} median dσ(mob−sta)={st.median(diffs):+.0f}ns "
            f"mobile_better={neg}/{len(diffs)} (negative=mobile quieter)"
        )
    else:
        print("\n# no opposite-mode adjacent pairs yet")


if __name__ == "__main__":
    main()
