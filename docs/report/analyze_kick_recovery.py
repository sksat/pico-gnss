#!/usr/bin/env python3
"""KICK 整列の遷移回復解析(制御器ごと)。PRBS h[k] が測れない「外乱からの回復」を受信非依存に比較する。

firmware が KICK_PERIOD locked エッジ毎に出力位相へ +KICK_AMP_NS のステップを注入する(kick_ns フィールド)。
各 kick イベントに整列し、その後の hwphase が lock 窓へ戻るまでのエッジ数(整定)・ピーク偏差・ab_boost の
boost 発火(state=1)を制御器(cidx)ごとに集計する。kick は固定ステップで全手法同一なので、回復の速さは
受信非依存に比較できる(受信ノイズは kick と無相関で整定判定に平均的にしか効かない)。

座標保護: PPSGEN 行のみ解析。NMEA/座標は一切読まない・出さない。

  python3 docs/report/analyze_kick_recovery.py logs/pps-ctrlsweep-prbs.log
"""
import re, sys, statistics

LOG = sys.argv[1] if len(sys.argv) > 1 else "logs/pps-ctrlsweep-prbs.log"
SETTLE_NS = 1000     # |hwphase| がこれ未満に戻れば整定とみなす(lock 窓)
SETTLE_HOLD = 3      # 連続でこのエッジ数 < SETTLE_NS なら整定確定
WINDOW = 80          # kick 後に追う最大エッジ数
CIDX_NAMES = {0: "pll_prod", 1: "ab_boost", 2: "integ_rework", 3: "pll_smith128", 4: "naive_pid"}


def name(c):
    return CIDX_NAMES.get(c, f"c{c}")


rows = []  # (hw, cidx, lk, kick, state)
for line in open(LOG, errors="ignore"):
    if "PPSGEN" not in line:
        continue
    hw = re.search(r"\bhwphase_ns=(-?\d+)\b", line)
    cidx = re.search(r"\bcidx=(\d+)\b", line)
    lk = re.search(r"\blk=(-?\d+)\b", line)
    kk = re.search(r"\bkick_ns=(-?\d+)\b", line)
    st = re.search(r"\bstate=(\d+)\b", line)
    if hw and cidx and lk and kk:
        rows.append((int(hw.group(1)), int(cidx.group(1)), int(lk.group(1)),
                     int(kk.group(1)), int(st.group(1)) if st else 0))
if not rows:
    sys.exit("no PPSGEN kick_ns rows (need KICK_INJECT firmware)")

n_kick = sum(1 for r in rows if r[3] != 0)
print(f"# {LOG}: {len(rows)} PPSGEN rows, {n_kick} kick events")

# 各 kick イベントごとに回復を測る。kick エッジの cidx をそのイベントの制御器とする。
# 出力 FIFO で外乱は kick の ~2 エッジ後に現れる(kick edge 直後の小 hwphase で誤整定しないよう、整定判定は
# **ピーク偏差の後**から始める)。kick が実際に着弾したイベント(peak ≥ MIN_PEAK)だけ採用する。
MIN_PEAK = 3000
events = {}  # cidx -> list of (settle_edges, peak_ns, boosted)
for i, r in enumerate(rows):
    if r[3] == 0:
        continue
    cidx = r[1]
    win = []  # (k, hw, state)
    for k in range(1, WINDOW + 1):
        if i + k >= len(rows):
            break
        hw, c2, lk2, kick2, state2 = rows[i + k]
        if c2 != cidx or kick2 != 0:
            break  # 次の kick / セグメント切替に達したら打ち切り
        win.append((k, hw, state2))
    if len(win) < 5:
        continue
    peak = max(abs(hw) for _, hw, _ in win)
    if peak < MIN_PEAK:
        continue  # kick が着弾していない(切替直後など)→ スキップ
    peak_k = next(k for k, hw, _ in win if abs(hw) == peak)
    boosted = max(s for _, _, s in win)
    settle = WINDOW
    hold = 0
    for k, hw, _ in win:
        if k <= peak_k:
            continue  # ピーク到達後から整定を測る
        if abs(hw) < SETTLE_NS:
            hold += 1
            if hold >= SETTLE_HOLD:
                settle = k - SETTLE_HOLD + 1
                break
        else:
            hold = 0
    events.setdefault(cidx, []).append((settle, peak, boosted))

print(f"\n{'controller':>13} {'Nkick':>6} {'settle中央':>10} {'settle p90':>10} {'peak中央ns':>11} {'boost発火%':>10}")
for cidx in sorted(events):
    ev = events[cidx]
    settles = sorted(s for s, _, _ in ev)
    peaks = [p for _, p, _ in ev]
    boosts = [b for _, _, b in ev]
    p90 = settles[min(len(settles) - 1, int(0.9 * len(settles)))]
    print(f"{name(cidx):>13} {len(ev):>6} {statistics.median(settles):>10.1f} {p90:>10} "
          f"{statistics.median(peaks):>11.0f} {100.0 * sum(boosts) / len(boosts):>10.1f}")

print("\n# settle = kick から |hwphase|<1000ns が 3 エッジ続くまでのエッジ数(小=速い回復)。"
      "boost発火% = その回復中に ab_boost が boost した割合(他手法は常に 0)。")
print("# 受信非依存性: kick は固定ステップで全手法同一、受信ノイズは kick と無相関。"
      "ただし各手法の kick 数が揃い・同程度の時間帯に分散しているか(interleave)を併せて確認すること。")
