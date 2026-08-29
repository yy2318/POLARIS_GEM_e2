import time,json,numpy as np
from .path_geometry import frenet_error
class Solver:
 def __init__(self,cfg,model_json=''):
  self.c=cfg;self.last=np.zeros(2);self.data=json.load(open(model_json)) if model_json else None
 def solve(self,x0,ref):
  try: import casadi as ca
  except Exception:return self.fallback(x0,ref),'NO_CASADI',0.
  N=int(self.c['horizon']);dt=float(self.c['dt']);L=float(self.c['wheelbase']);q=self.c['weights'];lim=self.c['limits'];o=ca.Opti();X=o.variable(8,N+1);U=o.variable(2,N);E=o.variable(N+1);o.subject_to(X[:,0]==x0);o.subject_to(E>=0);J=0;up=ca.DM(self.last);t=time.time()
  from .casadi_mlp import symbolic_delta
  for k in range(N):
   z=X[3:8,k];u=U[:,k];vx,vy,r,d,a=[z[i] for i in range(5)]
   dz=symbolic_delta(ca,ca.vertcat(z,u),self.data) if self.data else ca.vertcat(a*dt,-.8*vy*dt,(vx/L*ca.tan(d)-r)*.8*dt,(u[0]-d)*2*dt,(u[1]-a)*2*dt)
   p=X[2,k];nx=ca.vertcat(X[0,k]+(vx*ca.cos(p)-vy*ca.sin(p))*dt,X[1,k]+(vx*ca.sin(p)+vy*ca.cos(p))*dt,p+r*dt,z+dz);o.subject_to(X[:,k+1]==nx)
   xr,yr,pr,vr=ref[k,:4];ey=-ca.sin(pr)*(X[0,k]-xr)+ca.cos(pr)*(X[1,k]-yr);ep=ca.atan2(ca.sin(p-pr),ca.cos(p-pr));du=u-up
   J+=q['cte']*ey**2+q['heading']*ep**2+q['speed']*(vx-vr)**2+q['yaw_rate']*r**2+q['lateral_velocity']*vy**2+q['steering']*u[0]**2+q['acceleration']*u[1]**2+q['steering_rate']*du[0]**2+q['acceleration_rate']*du[1]**2+q['cte_slack']*E[k]**2
   o.subject_to(o.bounded(-lim['cte']-E[k],ey,lim['cte']+E[k]));up=u
  o.subject_to(o.bounded(0,X[3,:],lim['speed_max']));o.subject_to(o.bounded(-lim['steering_max'],U[0,:],lim['steering_max']));o.subject_to(o.bounded(lim['acceleration_min'],U[1,:],lim['acceleration_max']));o.minimize(J);o.solver('ipopt',{'print_time':False},{'print_level':0,'max_iter':int(self.c['ipopt_max_iter']),'max_cpu_time':float(self.c['max_cpu_time'])})
  try:s=o.solve();u=np.asarray(s.value(U[:,0])).reshape(2);self.last=u;return u,'SOLVED',(time.time()-t)*1000
  except Exception:return self.fallback(x0,ref),'SOLVER_FAILED',(time.time()-t)*1000
 def fallback(self,x,r):
  ey,_,ep=frenet_error(x[0],x[1],x[2],r[0,0],r[0,1],r[0,2]);l=self.c['limits'];return np.array([np.clip(-.7*ey-1.2*ep,-l['steering_max'],l['steering_max']),np.clip(r[0,3]-x[3],l['acceleration_min'],l['acceleration_max'])])
