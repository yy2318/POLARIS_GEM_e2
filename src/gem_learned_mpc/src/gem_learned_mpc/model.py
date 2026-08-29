import json,numpy as np
class MLPModel:
 def __init__(self,path=None):
  self.layers=[]; self.dt=.1; self.im=np.zeros(7);self.isd=np.ones(7);self.om=np.zeros(5);self.osd=np.ones(5)
  if path:
   d=json.load(open(path));self.dt=float(d.get('dt',.1));self.im=np.array(d['input_mean']);self.isd=np.array(d['input_std']);self.om=np.array(d['output_mean']);self.osd=np.array(d['output_std']);self.layers=[(np.array(v['weight']),np.array(v['bias'])) for v in d['layers']]
 def delta(self,z,u):
  q=np.r_[z,u]
  if not self.layers:
   vx,vy,r,d,a=z;dc,ac=u;dt=self.dt
   return np.array([a*dt,-.8*vy*dt,(vx/2.57*np.tan(d)-r)*.8*dt,(dc-d)*2*dt,(ac-a)*2*dt])
  q=(q-self.im)/np.maximum(self.isd,1e-9)
  for i,(w,b) in enumerate(self.layers): q=w@q+b; q=np.tanh(q) if i<len(self.layers)-1 else q
  return q*self.osd+self.om

def step(x,u,m,dt=None):
 dt=m.dt if dt is None else dt;x=np.asarray(x,float);z=x[3:8];zn=z+m.delta(z,u);X,Y,p=x[:3];vx,vy,r=z[:3]
 return np.r_[X+(vx*np.cos(p)-vy*np.sin(p))*dt,Y+(vx*np.sin(p)+vy*np.cos(p))*dt,float((p+r*dt+np.pi)%(2*np.pi)-np.pi),zn]
