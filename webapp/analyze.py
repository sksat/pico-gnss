#!/usr/bin/env python3
# /// script
# requires-python = ">=3.10"
# dependencies = []
# ///
"""pico-gnss の defmt ログ (probe-rs / server --log) をオフライン解析する。

使い方 (標準ライブラリのみ。uv でも素の python でも可):
    uv run analyze.py <logfile>          # 例: server --log で録った /tmp/eval.log
    probe-rs run --chip RP2040 <elf> > x.log 2>&1 && python3 analyze.py x.log

NMEA / PPS / SYNC(err_ns,holdover) / TIME / PPSGEN を集計し、測位精度・PPS 時刻精度・
GPSDO 安定度・時刻補正残差・GPSDO PPS 出力ジッタ・holdover 誤差・衛星/C/N0 を出す。
webapp ダッシュボードのオフライン版。±1ms フィルタや snap は firmware と揃えてある。
"""
import re, sys, math, statistics as st
from collections import defaultdict, Counter

path = sys.argv[1] if len(sys.argv) > 1 else "/tmp/eval.log"
log = open(path, encoding="utf-8", errors="replace").read().splitlines()


def ts(line):
    m = re.match(r"\s*([0-9]+\.[0-9]+)", line)
    return float(m.group(1)) if m else None


def snap(raw):  # firmware の snap_to_second_ns と同じ
    secs = (raw + (1 if raw >= 0 else -1) * 500_000_000) // 1_000_000_000
    return raw - secs * 1_000_000_000


nmea, pps, sync, time_, ppsgen, fw = [], [], [], [], [], None
for ln in log:
    if (m := re.search(r"NMEA (\$[A-Za-z0-9]{2,5},[^*\s]*\*[0-9A-Fa-f]{2})", ln)):
        nmea.append((ts(ln), m.group(1)))
    elif (m := re.search(r"PPS count=(\d+) interval_us=(\d+) interval_ns=(\d+) state=(\w+) missed=(\d+)", ln)):
        pps.append((ts(ln), int(m.group(3)), m.group(4), int(m.group(5))))
    elif (m := re.search(r"SYNC pps_local_us=(\d+) unix_s=(\d+) drift_us=(-?\d+)(?: err_ns=(-?\d+))?(?: holdover_ms=(\d+))?", ln)):
        sync.append((ts(ln), int(m.group(2)), int(m.group(4) or 0), int(m.group(5) or 1000)))
    elif (m := re.search(r"TIME unix_ns=(\d+) ppb=(-?\d+) holdover_ms=(\d+) locked=([01])", ln)):
        time_.append((ts(ln), int(m.group(2)), int(m.group(3)), m.group(4) == "1"))
    elif (m := re.search(r"PPSGEN count=(\d+) interval_ns=(\d+) dev_ns=(-?\d+)", ln)):
        ppsgen.append(int(m.group(3)))
    elif (m := re.search(r"FW (\$PMTK705,[^*]*)", ln)):
        fw = m.group(1)

dur = (nmea[-1][0] - nmea[0][0]) if len(nmea) > 1 else 0
print(f"=== capture: {path} ===")
print(f"~{dur:.0f}s | NMEA {len(nmea)} | PPS {len(pps)} | SYNC {len(sync)} | TIME {len(time_)} | PPSGEN {len(ppsgen)}")
if fw:
    print(f"FW: {fw}")


def coord(val, hemi):
    dot = val.find(".")
    if dot < 3:
        return None
    dd = int(val[:dot - 2]) + float(val[dot - 2:]) / 60
    return -dd if hemi in ("S", "W") else dd


def classify(talker, prn):
    if talker == "GL" or 65 <= prn <= 96:
        return "GLONASS"
    if talker == "GA" or 301 <= prn <= 336:
        return "Galileo"
    if talker in ("GB", "BD"):
        return "BeiDou"
    if talker == "GQ" or 183 <= prn <= 202:
        return "QZSS"
    if 33 <= prn <= 64 or 120 <= prn <= 158:
        return "SBAS"
    return "GPS"


# ---- 測位 (GGA) ----
pos, fixq, satsu, hd = [], [], [], []
for _, s in nmea:
    p = s.split(",")
    if p[0][3:] == "GGA":
        la, lo = coord(p[2], p[3]), coord(p[4], p[5])
        q = int(p[6]) if p[6] else 0
        fixq.append(q)
        if p[7]:
            satsu.append(int(p[7]))
        if p[8]:
            hd.append(float(p[8]))
        al = float(p[9]) if p[9] else None
        if q > 0 and la is not None and lo is not None:
            pos.append((la, lo, al))
print("\n=== positioning (static → scatter = repeatability) ===")
if satsu:
    print(f"fixq {dict(Counter(fixq))} | sats {min(satsu)}-{max(satsu)} (med {int(st.median(satsu))}) | HDOP {min(hd):.1f}-{max(hd):.1f} (med {st.median(hd):.1f})")
if len(pos) >= 10:
    mlat = st.mean(p[0] for p in pos); mlon = st.mean(p[1] for p in pos)
    cl = math.cos(mlat * math.pi / 180)
    e = [(p[1] - mlon) * 111320 * cl for p in pos]; n = [(p[0] - mlat) * 111320 for p in pos]
    alts = [p[2] for p in pos if p[2] is not None]
    rad = sorted(math.hypot(e[i], n[i]) for i in range(len(e)))
    pct = lambda q: rad[min(len(rad) - 1, int(q * len(rad)))]
    drms = math.hypot(st.pstdev(e), st.pstdev(n))
    print(f"  n={len(pos)} | CEP={pct(.5):.2f}m R95={pct(.95):.2f}m 2DRMS={2*drms:.2f}m | σE={st.pstdev(e):.2f} σN={st.pstdev(n):.2f} σAlt={st.pstdev(alts):.2f}m")


def stats(vals, filt=None):
    v = [x for x in vals if filt is None or filt(x)]
    if len(v) < 2:
        return None
    return len(v), st.mean(v), st.pstdev(v), max(v) - min(v), min(v), max(v)


# ---- PPS ジッタ ----
dev = [i - 1_000_000_000 for (_, i, sstate, _) in pps if sstate == "Locked" and abs(i - 1_000_000_000) < 1_000_000]
miss = sum(m for (_, _, _, m) in pps)
if (r := stats(dev)):
    print(f"\n=== PPS time precision (PIO 16ns) ===")
    print(f"  n={r[0]} | jitter σ={r[2]:.1f}ns p-p={r[3]}ns | offset {(1e9+r[1]-1e9)/1000:.3f}ppm | missed={miss}")

# ---- GPSDO ----
if time_:
    ppb = [x[1] for x in time_]
    print(f"\n=== GPSDO ===")
    print(f"  n={len(ppb)} | freq {st.mean(ppb):.0f}ppb σ={st.pstdev(ppb):.1f}ppb | locked {sum(x[3] for x in time_)/len(time_)*100:.0f}% | max holdover {max(x[2] for x in time_)/1000:.0f}s")

# ---- 時刻補正 err (snap 適用) + holdover 散布 ----
errs = [snap(e) for (_, _, e, _) in sync]
if (r := stats(errs, lambda x: abs(x) < 1_000_000)):
    print(f"\n=== clock err (補正後 UTC 残差, snap 済) ===")
    print(f"  n={r[0]} | σ={r[2]:.1f}ns mean={r[1]:.1f}ns p-p={r[3]}ns (cf. MT3333 1PPS spec ±10ns)")
hold = [(h / 1000, snap(e)) for (_, _, e, h) in sync if h > 2000]
if hold:
    print(f"  holdover 復帰 (h>2s) {len(hold)} 件: " + ", ".join(f"{h:.0f}s→{abs(e)}ns" for h, e in sorted(hold)[-6:]))

# ---- GPSDO PPS 出力 (PPSGEN) ----
if (r := stats([x for x in ppsgen if 1000 < abs(x) < 100000])):
    print(f"\n=== GPSDO PPS 出力 (GP3→GP4 ループバック) ===")
    print(f"  n={r[0]} | 周期 dev mean={r[1]:.0f}ns σ={r[2]:.1f}ns p-p={r[3]}ns (瞬時エッジジッタ ~16ns)")

# ---- 衛星 / C/N0 ----
sat_snr = defaultdict(dict)
for _, s in nmea:
    p = s.split(",")
    if p[0][3:] == "GSV":
        talker = p[0][1:3]; i = 4
        while i + 3 < len(p):
            try:
                prn = int(p[i])
            except ValueError:
                i += 4; continue
            snr = p[i + 3].split("*")[0]
            sat_snr[classify(talker, prn)][prn] = int(snr) if snr.isdigit() else None
            i += 4
print(f"\n=== satellites ===")
allsnr = []
for sysn in ["GPS", "QZSS", "GLONASS", "SBAS", "Galileo", "BeiDou"]:
    if sysn not in sat_snr:
        continue
    tracked = [v for v in sat_snr[sysn].values() if v]
    allsnr += tracked
    extra = f", C/N0 mean {st.mean(tracked):.0f}/max {max(tracked)}" if tracked else ""
    print(f"  {sysn:8s}: {len(sat_snr[sysn])} in view, {len(tracked)} tracked{extra}")
if allsnr:
    print(f"  ALL tracked {len(allsnr)} | C/N0 mean {st.mean(allsnr):.0f}/max {max(allsnr)} dBHz")
