import sys, json
import numpy as np
NS_CAP=16; LOCK_NS=1000; LOCK_HOLD=5
def cap16(x): return int(round(x/NS_CAP)*NS_CAP)
def gen_reception(n, srw, sw, seed):
    rng=np.random.default_rng(seed)
    return np.cumsum(rng.normal(0,srw,n))+rng.normal(0,sw,n)
SIGMA_RW=3.0; SIGMA_W=12.0
def run_plant(ctrl, n, rec, drift_accel=0.0, step_at=None, step_amp=0, init_off=0.0):
    out=float(init_off); drift=0.0; rate=0.0
    hw=np.empty(n); locked=np.zeros(n,bool)
    has=hasattr(ctrl,"is_locked"); cnt=0
    for k in range(n):
        rate+=drift_accel; drift+=rate
        if step_at is not None and k==step_at: out+=step_amp
        err=out-(rec[k]+drift); eq=cap16(err)
        trim,pc=ctrl.step(eq,True)
        hw[k]=err
        if has: locked[k]=ctrl.is_locked()
        else:
            cnt=cnt+1 if abs(eq)<LOCK_NS else 0; locked[k]=cnt>=LOCK_HOLD
        out+=trim/1000.0-pc
    return hw,locked
def _peak(hw):
    x=np.asarray(hw,float)-np.mean(hw)
    X=np.abs(np.fft.rfft(x*np.hanning(len(x)))); f=np.fft.rfftfreq(len(x),1.0)
    band=[(i,1/f[i]) for i in range(len(f)) if f[i]>0 and 30<=1/f[i]<=300]
    if not band: return 0.0
    i,_=max(band,key=lambda t:X[t[0]]); base=np.median([X[j] for j,_ in band])
    return float(X[i]/(base+1e-9))
def evaluate(fac, seeds=6):
    M={}; sds=[];pks=[]
    for s in range(seeds):
        rec=gen_reception(6000,SIGMA_RW,SIGMA_W,1000+s)
        hw,_=run_plant(fac(s),6000,rec); h=hw[1500:]
        sds.append(np.std(h)); pks.append(_peak(h))
    M["steady_rms"]=float(np.mean(sds)); M["resonance_peak"]=float(np.mean(pks))
    r5=[];r1=[];rl=[]
    for s in range(seeds):
        rec=gen_reception(1500,SIGMA_RW,SIGMA_W,2000+s)
        hw,lk=run_plant(fac(s),1500,rec,init_off=50000.0); a=np.abs(hw)
        r5.append(int(np.argmax(a<5000)) if np.any(a<5000) else 1500)
        r1.append(int(np.argmax(a<1000)) if np.any(a<1000) else 1500)
        rl.append(int(np.argmax(lk)) if np.any(lk) else 1500)
    M["reacq_5us_edge"]=float(np.mean(r5)); M["reacq_1us_edge"]=float(np.mean(r1)); M["lock_edge"]=float(np.mean(rl))
    st=[];ov=[];zc=[]
    for s in range(seeds):
        rec=gen_reception(2000,SIGMA_RW,SIGMA_W,3000+s)
        hw,_=run_plant(fac(s),2000,rec,step_at=800,step_amp=2000); seg=hw[800:1600]; a=np.abs(seg)
        st.append(int(np.argmax(a<200)) if np.any(a<200) else 800)
        ov.append(float(np.max(a))); zc.append(int(np.sum(np.sign(seg[:-1])!=np.sign(seg[1:]))))
    M["step_settle_edge"]=float(np.mean(st)); M["step_overshoot_ns"]=float(np.mean(ov)); M["step_zerocross"]=float(np.mean(zc))
    dr=[]
    for s in range(seeds):
        rec=gen_reception(3000,SIGMA_RW,SIGMA_W,4000+s)
        hw,_=run_plant(fac(s),3000,rec,drift_accel=0.002); dr.append(float(np.std(hw[1500:])))
    M["drift_rms"]=float(np.mean(dr))
    return {k:round(v,2) for k,v in M.items()}
class PidSmith:
    def __init__(self,i_den=128,d_den=4,kp_inv=8,i_enable_ns=5000,outlier_ns=3000,outlier_max=12,
                 lock_ns=1000,lock_hold=5,calm_ns=1000,shift=0,trim_max=3_000_000):
        self.i_den=i_den;self.d_den=d_den;self.kp_inv=kp_inv;self.i_en=i_enable_ns;self.ol=outlier_ns
        self.olm=outlier_max;self.ln=lock_ns;self.lh=lock_hold;self.cn=calm_ns;self.sh=shift;self.tm=trim_max
        self.trim=0;self.lc=0;self.rej=0;self.lp=0;self.lpd=0
    def is_locked(self): return self.lc>=self.lh
    def step(self,err,valid):
        ctrl=err; pred=ctrl-self.lpd; locked=self.is_locked(); p=d=0; ap=False
        if valid:
            if locked and abs(ctrl)>self.ol and self.rej<self.olm: self.rej+=1
            else:
                self.rej=0; ap=True
                if abs(ctrl)<self.i_en:
                    iden=self.i_den if abs(pred)<self.cn else self.i_den<<min(self.sh,20)
                    self.trim=max(-self.tm,min(self.tm,self.trim-pred*1000//iden))
                p=max(-10**8,min(10**8,pred//self.kp_inv))
                if locked: d=max(-10**8,min(10**8,(pred-self.lp)//self.d_den))
                self.lc=min(self.lc+1,self.lh) if abs(ctrl)<self.ln else 0
        if ap: self.lp=pred; self.lpd=p+d
        return self.trim,p+d
BASELINES={"linear_128":lambda s:PidSmith(i_den=128),"linear_256":lambda s:PidSmith(i_den=256),"linear_512":lambda s:PidSmith(i_den=512)}
if __name__=="__main__":
    if len(sys.argv)>1:
        import importlib
        mod,cls=sys.argv[1].split(":"); m=importlib.import_module(mod); f=getattr(m,cls)
        fac=(lambda s:f()) if isinstance(f,type) else f
        print(json.dumps({sys.argv[1]:evaluate(fac)},ensure_ascii=False))
    else:
        print(json.dumps({n:evaluate(f) for n,f in BASELINES.items()},ensure_ascii=False,indent=2))
