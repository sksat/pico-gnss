import numpy as np
class Kalman2:
    def __init__(self, q_phi=0.5, q_f=0.02, r=300.0, adapt=12.0):
        self.x=np.array([0.0,0.0]); self.P=np.array([[1e7,0.0],[0.0,1e4]])
        self.F=np.array([[1.0,1.0],[0.0,1.0]]); self.H=np.array([1.0,0.0])
        self.Q=np.array([[q_phi,0.0],[0.0,q_f]]); self.R=r; self.adapt=adapt; self.lc=0
    def is_locked(self): return self.lc>=5
    def step(self, err_ns, valid):
        z=float(err_ns)
        self.x=self.F@self.x; self.P=self.F@self.P@self.F.T+self.Q
        y=z-self.H@self.x; S=self.H@self.P@self.H+self.R
        if y*y/S>self.adapt:
            self.P=self.P*(y*y/S/self.adapt); S=self.H@self.P@self.H+self.R
        K=self.P@self.H/S; self.x=self.x+K*y; self.P=self.P-np.outer(K,self.H@self.P)
        phi,f=self.x
        self.lc=min(self.lc+1,5) if abs(z)<1000 else 0
        trim=int(max(-3_000_000,min(3_000_000,-f*1000)))
        pcorr=int(max(-10**8,min(10**8,phi)))
        return trim,pcorr
class KalmanNL:
    def __init__(self, q_phi=0.5, q_f=0.02, r=300.0, adapt=12.0, gmin=0.06, gmax=0.4, phi0=2000.0):
        self.x=np.array([0.0,0.0]); self.P=np.array([[1e7,0.0],[0.0,1e4]])
        self.F=np.array([[1.0,1.0],[0.0,1.0]]); self.H=np.array([1.0,0.0])
        self.Q=np.array([[q_phi,0.0],[0.0,q_f]]); self.R=r; self.adapt=adapt
        self.gmin=gmin; self.gmax=gmax; self.phi0=phi0; self.lc=0
    def is_locked(self): return self.lc>=5
    def step(self, err_ns, valid):
        z=float(err_ns)
        self.x=self.F@self.x; self.P=self.F@self.P@self.F.T+self.Q
        y=z-self.H@self.x; S=self.H@self.P@self.H+self.R
        if y*y/S>self.adapt:
            self.P=self.P*(y*y/S/self.adapt); S=self.H@self.P@self.H+self.R
        K=self.P@self.H/S; self.x=self.x+K*y; self.P=self.P-np.outer(K,self.H@self.P)
        phi,f=self.x
        self.lc=min(self.lc+1,5) if abs(z)<1000 else 0
        g=self.gmin+(self.gmax-self.gmin)*(phi*phi/(phi*phi+self.phi0*self.phi0))
        trim=int(max(-3_000_000,min(3_000_000,-f*1000)))
        pcorr=int(max(-10**8,min(10**8,phi*g)))
        return trim,pcorr
