#!/usr/bin/env python3
"""温度 FF の長時間 A/B の scope 取得。毎 PPS の ch2-ch1 エッジ差を .shots に追記しつつ、
SCREENSHOT_EVERY ごとに scope_autoscale.py (live) でレンジを自動調整してからスクショを保存し、
そのあと計測用の setup へ戻す。
usage: RIGOL_HOST=... uv run --python 3.12 --with python-vxi11 python3 longrun.py <tag> <duration_s>
"""
import os
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(HERE)))))
DATA = os.path.join(ROOT, "logs", "20260705-tempff-abab")  # 計測データの置き場 (gitignore、ローカルのみ)
SCRIPTS = os.path.join(ROOT, "scripts")
sys.path.insert(0, SCRIPTS)
sys.path.insert(0, HERE)
if os.environ.get("SCOPE_TRANSPORT") == "raw":
    from rigol_raw import RigolRaw as RigolVxi11  # VXI-11 read 死亡時のフォールバック
else:
    from rigol_vxi11 import RigolVxi11
import checkpoint as cp

SCREENSHOT_EVERY = 600.0  # 10 分ごと (レンジ再調整の追従を速く)


def take_screenshot(r, path):
    """autoscale (live) でレンジを整えてからスクショし、計測 setup へ戻す。"""
    try:
        r.close()
    except Exception:
        pass
    # autoscale は自前の接続を張るので、こちらの接続は一旦closeしてから
    subprocess.run([sys.executable, os.path.join(SCRIPTS, "scope_autoscale.py"), "live", "8"],
                   timeout=120, capture_output=True)
    r2 = RigolVxi11(timeout=8.0)
    time.sleep(2)
    r2.screenshot(path)
    # 計測 setup へ戻して xinc を取り直す
    r2.clear()
    cp.setup(r2)
    xinc_ns = float(r2.query(":WAVeform:XINCrement?")) * 1e9
    return r2, xinc_ns


def main():
    tag = sys.argv[1]
    dur = float(sys.argv[2]) if len(sys.argv) > 2 else 28800
    scr_off = float(os.environ.get("SCR_OFFSET_MIN", "0"))  # 再開時に旧スクショを上書きしない
    r = RigolVxi11(timeout=8.0)
    r.clear()
    cp.setup(r)
    xinc_ns = float(r.query(":WAVeform:XINCrement?")) * 1e9
    t_start = time.time()
    t_end = t_start + dur
    next_shot_png = t_start  # 開始直後に 1 枚
    n_ok = n_fail = n_scr = 0
    out = os.path.join(DATA, f"{tag}.shots")
    while time.time() < t_end:
        if time.time() >= next_shot_png:
            try:
                png = os.path.join(DATA, f"scr-{int((time.time()-t_start)/60 + scr_off):04d}min.png")
                r, xinc_ns = take_screenshot(r, png)
                n_scr += 1
                print(f"screenshot {png} ({n_scr})", flush=True)
            except Exception as e:
                print(f"screenshot failed: {e}", flush=True)
                try:
                    # 固着したリンクが単一クライアント枠を握らないよう必ず先に切る
                    try:
                        r.close()
                    except Exception:
                        pass
                    r = RigolVxi11(timeout=8.0); r.clear(); cp.setup(r)
                    xinc_ns = float(r.query(":WAVeform:XINCrement?")) * 1e9
                except Exception:
                    time.sleep(5)
            next_shot_png = time.time() + SCREENSHOT_EVERY
        try:
            v = cp.one_shot(r, xinc_ns)
        except Exception:
            n_fail += 1
            try:
                r.clear(); cp.setup(r)
                xinc_ns = float(r.query(":WAVeform:XINCrement?")) * 1e9
            except Exception:
                try:
                    try:
                        r.close()
                    except Exception:
                        pass
                    r = RigolVxi11(timeout=8.0); r.clear(); cp.setup(r)
                    xinc_ns = float(r.query(":WAVeform:XINCrement?")) * 1e9
                except Exception:
                    time.sleep(5)
            continue
        if v is None:
            n_fail += 1
        else:
            n_ok += 1
            with open(out, "a") as f:
                f.write(f"{time.time():.1f} {v:.1f}\n")
    print(f"done ok={n_ok} fail={n_fail} scr={n_scr}", flush=True)


main()
