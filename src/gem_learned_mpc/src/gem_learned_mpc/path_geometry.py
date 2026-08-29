import numpy as np
from .angles import wrap_to_pi

def load_waypoints(path):
 a=np.loadtxt(path,delimiter=',',ndmin=2)
 if a.shape[1]<2 or not np.isfinite(a).all(): raise ValueError('CSV requires finite x,y columns')
 return a

def preprocess(a,ds=0.25):
 p=np.asarray(a[:,:2],float); keep=np.r_[True,np.linalg.norm(np.diff(p,axis=0),axis=1)>1e-3]; p=p[keep]
 if len(p)<2: raise ValueError('path has fewer than two distinct points')
 s=np.r_[0.,np.cumsum(np.linalg.norm(np.diff(p,axis=0),axis=1))]; sq=np.arange(0,s[-1]+1e-9,ds)
 x=np.interp(sq,s,p[:,0]); y=np.interp(sq,s,p[:,1]); yaw=np.unwrap(np.arctan2(np.gradient(y),np.gradient(x))); kappa=np.gradient(yaw,sq)
 return dict(s=sq,x=x,y=y,yaw=yaw,kappa=kappa)

def nearest_index(p,x,y,last=None,window=100):
 lo=0 if last is None else max(0,last-window); hi=len(p['x']) if last is None else min(len(p['x']),last+window+1)
 return lo+int(np.argmin((p['x'][lo:hi]-x)**2+(p['y'][lo:hi]-y)**2))

def frenet_error(x,y,yaw,xr,yr,yawr):
 dx=x-xr;dy=y-yr
 return float(-np.sin(yawr)*dx+np.cos(yawr)*dy),float(np.cos(yawr)*dx+np.sin(yawr)*dy),float(wrap_to_pi(yaw-yawr))

def speed_profile(kappa,vmin=.8,vmax=5.5556,aymax=1.5,kmin=1e-3): return np.clip(np.sqrt(aymax/np.maximum(np.abs(kappa),kmin)),vmin,vmax)
