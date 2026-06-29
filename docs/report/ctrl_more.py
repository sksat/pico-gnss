#!/usr/bin/env python3
"""制御器の比較ランナー(Step 2) — host プラント上で linear / 積分改良 / LQG-ish を並べる。

ctrl_eval.evaluate() の共通プラント(16ns 量子化 + colored 受信ノイズ)で、以下を同条件比較する:
  linear_128 / linear_512 : firmware 忠実 PID+Smith(i_den 違い)= ベースライン
  integ_rework           : codex の積分改良 — i_enable ゲートを廃し、積分「入力」を ±e_i に
                            クランプ + back-calculation anti-windup。フラグ(モード切替)無しで
                            復旧時のワインドアップを抑える連続制御器。
  lqg_linear             : KalmanNL を定ゲイン(gmin=gmax)で使った 2-state LQG 線形版。
  kalman_nl              : KalmanNL(クリーン推定に非線形ゲイン)を tuned 設定で。

指標は steady_rms(静定 σ)/ resonance_peak(共振)/ reacq / lock / step 応答 / drift_rms。
小さいほど良い(resonance_peak は 1 に近いほど良い)。座標等は一切扱わない host 専用。

  python3 docs/report/ctrl_more.py
"""
import sys, os, json
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from ctrl_eval import evaluate, PidSmith  # noqa: E402
from ctrl_kalman import KalmanNL          # noqa: E402


class IntegralRework:
    """フラグ無しの連続積分器: i_enable ゲートを使わず常に積分するが、
    積分「入力」pred を ±e_i にクランプ(大外乱で一気に巻かない)+ trim 飽和時は
    back-calculation で積分器を実現値へ引き戻す。復旧と静定を切り替え無しで両立させる狙い。
    P/D は firmware と同じ(Smith 予測子 pred を kp_inv / d_den で分ける)。"""
    def __init__(self, i_den=128, d_den=4, kp_inv=8, e_i=800, kbc=0.2,
                 outlier_ns=3000, outlier_max=12, lock_ns=1000, lock_hold=5, trim_max=3_000_000):
        self.i_den = i_den; self.d_den = d_den; self.kp_inv = kp_inv
        self.e_i = e_i; self.kbc = kbc
        self.ol = outlier_ns; self.olm = outlier_max
        self.ln = lock_ns; self.lh = lock_hold; self.tm = trim_max
        self.trim = 0; self.lc = 0; self.rej = 0; self.lp = 0; self.lpd = 0

    def is_locked(self):
        return self.lc >= self.lh

    def step(self, err, valid):
        ctrl = err; pred = ctrl - self.lpd; locked = self.is_locked(); p = d = 0; ap = False
        if valid:
            if locked and abs(ctrl) > self.ol and self.rej < self.olm:
                self.rej += 1                       # 外れ値だけは弾く(測定不良。モードではない)
            else:
                self.rej = 0; ap = True
                pin = max(-self.e_i, min(self.e_i, pred))   # 積分入力クランプ=連続 anti-windup
                want = self.trim - pin * 1000 // self.i_den
                self.trim = max(-self.tm, min(self.tm, want))
                self.trim -= int(self.kbc * (want - self.trim))  # back-calc: 飽和分を積分器へ戻す
                p = max(-10**8, min(10**8, pred // self.kp_inv))
                if locked:
                    d = max(-10**8, min(10**8, (pred - self.lp) // self.d_den))
                self.lc = min(self.lc + 1, self.lh) if abs(ctrl) < self.ln else 0
        if ap:
            self.lp = pred; self.lpd = p + d
        return self.trim, p + d


FACTORIES = {
    "linear_128":    lambda s: PidSmith(i_den=128),
    "linear_512":    lambda s: PidSmith(i_den=512),
    "integ_rework":  lambda s: IntegralRework(i_den=128, e_i=800, kbc=0.2),
    "lqg_linear":    lambda s: KalmanNL(gmin=0.12, gmax=0.12),
    "kalman_nl":     lambda s: KalmanNL(gmin=0.18, gmax=0.4, phi0=700.0),
}

METRICS = ["steady_rms", "resonance_peak", "reacq_1us_edge", "lock_edge",
           "step_settle_edge", "step_overshoot_ns", "step_zerocross", "drift_rms"]

if __name__ == "__main__":
    res = {n: evaluate(f) for n, f in FACTORIES.items()}
    # JSON も出す(後で REPORT に転記しやすいよう)
    print(json.dumps(res, ensure_ascii=False, indent=2))
    # 表(小さいほど良い。resonance_peak は 1 に近いほど良い)
    names = list(FACTORIES)
    w = max(len(m) for m in METRICS) + 1
    print("\n" + " " * w + "".join(f"{n:>14}" for n in names))
    for m in METRICS:
        row = "".join(f"{res[n][m]:>14.2f}" for n in names)
        print(f"{m:<{w}}{row}")
