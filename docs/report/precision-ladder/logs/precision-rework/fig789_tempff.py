#!/usr/bin/env python3
"""Regenerate fig7/8/9 (温度FF figures) from heat logs, best-effort.
Labels use 「loopback 位相」「温度FF」; no 'hwphase' / 'temp-FF'."""
import re, statistics as st
import matplotlib
matplotlib.use('Agg')
from matplotlib import font_manager
FONT = '/usr/share/fonts/noto-cjk/NotoSansCJK-Regular.ttc'
font_manager.fontManager.addfont(FONT)
FNAME = font_manager.FontProperties(fname=FONT).get_name()
matplotlib.rcParams['font.family'] = FNAME
matplotlib.rcParams['axes.unicode_minus'] = False
import matplotlib.pyplot as plt

OUT = '/home/sksat/prog/pico-gnss-rs/.claude/worktrees/review-precision/docs/report/precision-ladder/precision-figs'
S5 = 'logs/precision-rework/s5'

def temp_c(raw):
    return 27 - ((raw/256)*3.3/4096 - 0.706)/0.001721

def parse(path):
    rows = []
    with open(path, errors='replace') as f:
        for ln in f:
            if 'PPSGEN count=' not in ln: continue
            d = {k:int(v) for k,v in re.findall(r"(\w+)=(-?\d+)", ln)}
            if 'count' not in d: continue
            if d['count'] < 30: continue
            if abs(d.get('hwphase_ns',0)) > 2e6: continue
            rows.append(d)
    return rows

def smooth(xs, w=5):
    n=len(xs); out=[]
    for i in range(n):
        a=max(0,i-w//2); b=min(n,i+w//2+1); out.append(sum(xs[a:b])/(b-a))
    return out

def detect_events(rows, rise=0.5, drop=0.3, tail=15, step_gate=0.3, step_win=3):
    rawT=[temp_c(r['temp_raw']) for r in rows]; T=smooth(rawT,5); n=len(rows)
    events=[]; base_T=T[0]; base_i=0; i=1
    while i<n:
        if T[i] < base_T: base_T=T[i]; base_i=i
        if T[i] >= base_T+rise:
            peak_T=T[i]; peak_i=i; j=i+1
            while j<n:
                if T[j] > peak_T: peak_T=T[j]; peak_i=j
                if T[j] <= peak_T-drop: break
                j+=1
            end=min(n-1, peak_i+tail)
            maxstep=0.0
            for k in range(base_i, max(base_i,peak_i-step_win)+1):
                if k+step_win<n: maxstep=max(maxstep, rawT[k+step_win]-rawT[k])
            seg=rows[base_i:end+1]; dT=peak_T-base_T
            hw=[abs(r['hwphase_ns']) for r in seg]
            events.append(dict(base=base_i,peak=peak_i,end=end,
                c0=rows[base_i]['count'],cpk=rows[peak_i]['count'],dT=dT,
                peak_hw=max(hw),integ=sum(hw),peak_per=max(hw)/dT,integ_per=sum(hw)/dT,
                fast=(maxstep>=step_gate)))
            base_T=T[j] if j<n else peak_T; base_i=j if j<n else peak_i; i=j
        else: i+=1
    return events

# ---------- FIG 7 : ON fast heating event, bipolar transient ----------
rows5 = parse(f'{S5}/stage5-heat.log')
LO, HI = 248, 410
w = [r for r in rows5 if LO <= r['count'] <= HI]
cnt = [r['count'] for r in w]
hw  = [r['hwphase_ns'] for r in w]
tmp = [temp_c(r['temp_raw']) for r in w]
# transient metrics
pos_i = max(range(len(w)), key=lambda i: hw[i]); pos_pk = hw[pos_i]; pos_c = cnt[pos_i]
neg_i = min(range(len(w)), key=lambda i: hw[i]); neg_pk = hw[neg_i]; neg_c = cnt[neg_i]
onset_c = next(c for c,h in zip(cnt,hw) if h > 500)
ret_c = next((c for c,h in zip(cnt,hw) if c > neg_c and abs(h) <= 100), None)
ret_s = ret_c - onset_c
# durable settle (last out-of-band before window end)
settle_c = max((c for c,h in zip(cnt,hw) if abs(h) > 100), default=None)

fig, ax = plt.subplots(figsize=(10,5.2))
ax.axhspan(-100, 100, color='#cdebcd', alpha=0.7, label='≤100 ns 帯', zorder=0)
ax.plot(cnt, hw, color='#1f5fd6', lw=1.4, label='loopback 位相 [ns] (左軸)', zorder=3)
ax.axhline(0, color='0.6', lw=0.7, zorder=1)
ax.set_xlabel('起動からの経過 [秒]')
ax.set_ylabel('loopback 位相 [ns]', color='#1f5fd6')
ax.tick_params(axis='y', labelcolor='#1f5fd6')
ax.set_xlim(LO, HI)
ax2 = ax.twinx()
ax2.plot(cnt, tmp, color='#d62728', lw=1.3, label='ダイ温度 [℃] (右軸)')
ax2.set_ylabel('ダイ温度 [℃]', color='#d62728')
ax2.tick_params(axis='y', labelcolor='#d62728')
ax.annotate(f'速い加熱 (+{tmp[pos_i]-tmp[0]:.1f}℃ 級) で loopback 位相が\n'
            f'+{pos_pk} / {neg_pk} ns に双極性に振れ、\n≤100 ns 帯へ ~{ret_s}s で復帰',
            xy=(neg_c, neg_pk), xytext=(0.40, 0.18), textcoords='axes fraction',
            color='#c00', fontsize=10, ha='left', va='center',
            bbox=dict(boxstyle='round', fc='white', ec='#c00', alpha=0.9),
            arrowprops=dict(arrowstyle='->', color='#c00'))
ax.set_title('最悪の速いステップでの過渡 (温度フィードフォワードあり)')
l1,lab1 = ax.get_legend_handles_labels(); l2,lab2 = ax2.get_legend_handles_labels()
ax.legend(l1+l2, lab1+lab2, loc='upper right', fontsize=9)
fig.tight_layout(); fig.savefig(f'{OUT}/fig7-tempff-heat.png', dpi=120); plt.close(fig)
print(f"FIG7: window count {LO}..{HI}  pos_peak=+{pos_pk}ns@{pos_c} neg_peak={neg_pk}ns@{neg_c}")
print(f"      onset@{onset_c} return-to-band@{ret_c} => {ret_s}s ; durable settle last-out@{settle_c} (~{settle_c-onset_c}s)")

# ---------- FIG 8 : ON/OFF per-event scatter + median ----------
ev_on  = [e for e in detect_events(rows5) if e['fast']]
rows4a = parse(f'{S5}/stage4-heat.log');  ev_4a = [e for e in detect_events(rows4a) if e['fast']]
rows4b = parse(f'{S5}/stage4-heat2.log'); ev_4b = [e for e in detect_events(rows4b) if e['fast']]
ev_off = ev_4a + ev_4b
on_pk=[e['peak_per'] for e in ev_on]; off_pk=[e['peak_per'] for e in ev_off]
on_ig=[e['integ_per'] for e in ev_on]; off_ig=[e['integ_per'] for e in ev_off]
GRN='#2ca02c'; RED='#d62728'
fig, ax = plt.subplots(figsize=(10,5))
def draw_events(rows, evs, col, label):
    lab = label
    for e in evs:
        seg = rows[e['base']:e['end']+1]
        t0 = seg[0]['count']
        ax.plot([r['count']-t0 for r in seg], [r['hwphase_ns']/e['dT'] for r in seg],
                color=col, lw=1.4, alpha=0.8, label=lab)
        lab = None
draw_events(rows4b, ev_4b, RED, f'温度フィードフォワードなし ({len(ev_off)} 回)')
draw_events(rows4a, ev_4a, RED, None)
draw_events(rows5, ev_on, GRN, f'温度フィードフォワードあり ({len(ev_on)} 回)')
ax.axhline(0, color='0.6', lw=0.8)
ax.set_xlabel('加熱イベント開始からの経過 [秒]')
ax.set_ylabel('loopback 位相 / そのイベントの温度上昇 [ns/℃]')
ax.set_title('速い加熱への応答 (イベントごとの温度上昇で正規化して重ねた)')
ax.legend(loc='lower right', fontsize=10)
ax.grid(ls=':', alpha=0.4)
fig.tight_layout(); fig.savefig(f'{OUT}/fig8-tempff-ab.png', dpi=120); plt.close(fig)
print(f"FIG8: n_ON={len(on_pk)} n_OFF={len(off_pk)} (stage4-heat fast={len(ev_4a)}, stage4-heat2 fast={len(ev_4b)})")
print(f"      ON  peak/C med={st.median(on_pk):.0f} range {min(on_pk):.0f}..{max(on_pk):.0f} | int/C med={st.median(on_ig):.0f} range {min(on_ig):.0f}..{max(on_ig):.0f}")
print(f"      OFF peak/C med={st.median(off_pk):.0f} range {min(off_pk):.0f}..{max(off_pk):.0f} | int/C med={st.median(off_ig):.0f} range {min(off_ig):.0f}..{max(off_ig):.0f}")

# ---------- FIG 9 : feedforward over-reaction ----------
LO9, HI9 = 255, 345
w9=[r for r in rows5 if LO9 <= r['count'] <= HI9]
c9=[r['count'] for r in w9]; ff=[r.get('ff_delta',0)/1000 for r in w9]
pk_i=min(range(len(w9)), key=lambda i: ff[i]); pk=ff[pk_i]; pk_c=c9[pk_i]
REAL=-0.773; ratio=abs(pk)/abs(REAL)
fig, ax = plt.subplots(figsize=(10,5))
ax.plot(c9, ff, color='#a01010', lw=1.6, label='適用したフィードフォワード [ppb]')
ax.axhline(REAL, color='#1a7a1a', ls='--', lw=1.6, label=f'実際の水晶周波数変化 ≈ {REAL} ppb')
ax.axhline(0, color='0.7', lw=0.7)
ax.set_xlabel('起動からの経過 [秒]'); ax.set_ylabel('フィードフォワード [ppb]')
ax.set_xlim(LO9, HI9)
ax.annotate(f'ピーク {pk:.0f} ppb\n= 実変化 ({REAL} ppb) の ~{ratio:.0f} 倍 (過反応)',
            xy=(pk_c, pk), xytext=(0.42, 0.30), textcoords='axes fraction',
            color='#c00', fontsize=10, ha='left',
            bbox=dict(boxstyle='round', fc='white', ec='#c00', alpha=0.9),
            arrowprops=dict(arrowstyle='->', color='#c00'))
ax.set_title(f'フィードフォワードの過反応 (実変化の ~{ratio:.0f} 倍)')
ax.legend(loc='lower right', fontsize=9)
fig.tight_layout(); fig.savefig(f'{OUT}/fig9-overreact.png', dpi=120); plt.close(fig)
print(f"FIG9: window count {LO9}..{HI9}  ff peak={pk:.2f} ppb @count{pk_c}  real={REAL} ppb  ratio=~{ratio:.0f}x")

# ---------- FIG 10 : steering clamp A/B (SAME fast-step detector as FIG8) ----------
def parse_stop(path, stop_count):
    """Stream-parse a large log, stop once PPSGEN count passes stop_count."""
    rows=[]
    with open(path, errors='replace') as f:
        for ln in f:
            if 'PPSGEN count=' not in ln: continue
            d={k:int(v) for k,v in re.findall(r"(\w+)=(-?\d+)", ln)}
            if 'count' not in d: continue
            c=d['count']
            if c>stop_count and c<10**8: break
            if c<30: continue
            if abs(d.get('hwphase_ns',0))>2e6: continue
            rows.append(d)
    return rows

# unclamped = stage5-heat (identical events/metric as FIG8 ON)
unc = [e['peak_per'] for e in ev_on]
# clamped = clamped-heat.log; all fast heating steps are early (count<500), stop at 2000
CL_STOP = 2000
rows_cl = parse_stop(f'{S5}/clamped-heat.log', CL_STOP)
ev_cl = [e for e in detect_events(rows_cl) if e['fast']]
clp = [e['peak_per'] for e in ev_cl]
cl_c0 = min(e['c0'] for e in ev_cl); cl_cend = max(rows_cl[e['peak']]['count'] for e in ev_cl)
med_u=st.median(unc); med_c=st.median(clp); worst_u=max(unc); worst_c=max(clp)
r_med=med_u/med_c; r_worst=worst_u/worst_c

fig, ax = plt.subplots(figsize=(8.6,5.6))
ax.set_yscale('log')
RED='#d15b5b'; GRN='#5aa86a'
ax.bar(1, med_u, width=0.62, color=RED, alpha=0.9, zorder=1)
ax.bar(2, med_c, width=0.62, color=GRN, alpha=0.9, zorder=1)
for vals,x,col in [(unc,1,'#7a1010'),(clp,2,'#0e5a0e')]:
    xs=[x+(i-(len(vals)-1)/2)*0.08 for i in range(len(vals))]
    ax.scatter(xs, vals, color=col, s=58, zorder=5, edgecolor='white', lw=0.8)
ax.text(1.33, med_u, f'中央値 {med_u:.0f}', va='center', ha='left', color='#7a1010', fontsize=9)
ax.text(2.33, med_c, f'中央値 {med_c:.0f}', va='center', ha='left', color='#0e5a0e', fontsize=9)
ax.text(1.33, worst_u, f'worst {worst_u:.0f}', va='center', ha='left', color='#7a1010', fontsize=9)
ax.text(2.33, worst_c, f'worst {worst_c:.0f}', va='center', ha='left', color='#0e5a0e', fontsize=9)
ax.annotate('', xy=(2.62, med_c), xytext=(2.62, med_u),
            arrowprops=dict(arrowstyle='<->', color='0.3', lw=1.4))
ax.text(2.68, (med_u*med_c)**0.5, f'中央値で ~{r_med:.1f}倍\n(worst では ~{r_worst:.0f}倍)',
        va='center', ha='left', fontsize=10,
        bbox=dict(boxstyle='round', fc='#fdf6e3', ec='0.5', alpha=0.95))
ax.set_xticks([1,2]); ax.set_xticklabels(['clamp なし','clamp あり'])
ax.set_xlim(0.45, 3.4); ax.set_ylim(150, 14000)
ax.set_ylabel('加熱過渡の peak loopback 位相/℃ [ns/℃] (対数軸)')
ax.set_title('±100 ppb の clamp による加熱過渡の変化')
fig.tight_layout(); fig.savefig(f'{OUT}/fig10-clamp-ab.png', dpi=120); plt.close(fig)
print(f"FIG10: unclamped(stage5-heat) n={len(unc)} peak/C={[round(v) for v in sorted(unc)]} "
      f"med={med_u:.1f} worst={worst_u:.1f}")
print(f"       clamped(clamped-heat, count<={CL_STOP}, events {cl_c0}..{cl_cend}) n={len(clp)} "
      f"peak/C={[round(v) for v in sorted(clp)]} med={med_c:.1f} worst={worst_c:.1f}")
print(f"       ratio median={r_med:.2f}x  worst={r_worst:.2f}x")
print(f"       CHECK unclamped median ({med_u:.1f}) == FIG8 ON median: {'OK' if abs(med_u-st.median(on_pk))<1e-6 else 'MISMATCH'}")
print(f"FONT used: {FNAME}")
