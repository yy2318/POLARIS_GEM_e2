#!/usr/bin/env python3
import argparse,json
from pathlib import Path
import numpy as np

def interp_cont(tq,t,x):return np.interp(tq,t,x)
def zoh(tq,t,x):
    idx=np.searchsorted(t,tq,side='right')-1;idx=np.clip(idx,0,len(t)-1);return x[idx]
def main():
    ap=argparse.ArgumentParser();ap.add_argument('input');ap.add_argument('--out',required=True);ap.add_argument('--dt',type=float,default=.05);ap.add_argument('--history',type=int,default=4);ap.add_argument('--reset-distance',type=float,default=5.0);a=ap.parse_args()
    z=np.load(a.input);S=z['states'];U=z['commands'];runs=[]
    for rid in sorted(set(S[:,0].astype(int))):
        s=S[S[:,0]==rid];u=U[U[:,0]==rid]
        if len(s)<a.history+2 or len(u)<2:continue
        order=np.argsort(s[:,1]);s=s[order];u=u[np.argsort(u[:,1])]
        # Split large pose jumps inside a bag.
        jumps=np.where(np.hypot(np.diff(s[:,2]),np.diff(s[:,3]))>a.reset_distance)[0]+1
        for part in np.split(s,jumps):
            if len(part)<a.history+2:continue
            lo=max(part[0,1],u[0,1]);hi=min(part[-1,1],u[-1,1])
            tq=np.arange(lo,hi+a.dt*.25,a.dt)
            if len(tq)<a.history+2:continue
            yaw=np.unwrap(part[:,4])
            dyn=np.column_stack([interp_cont(tq,part[:,1],part[:,5]),interp_cont(tq,part[:,1],part[:,6]),interp_cont(tq,part[:,1],part[:,7]),interp_cont(tq,part[:,1],part[:,8])])
            pose=np.column_stack([interp_cont(tq,part[:,1],part[:,2]),interp_cont(tq,part[:,1],part[:,3]),interp_cont(tq,part[:,1],yaw)])
            cmd=np.column_stack([zoh(tq,u[:,1],u[:,2]),zoh(tq,u[:,1],u[:,3])])
            runs.append((rid,tq,pose,dyn,cmd))
    X=[];Y=[];G=[]
    for gid,(rid,t,pose,dyn,cmd) in enumerate(runs):
        for k in range(a.history-1,len(t)-1):
            X.append(np.r_[dyn[k],cmd[k-a.history+1:k+1].reshape(-1)])
            Y.append(dyn[k+1]);G.append(gid)
    X=np.asarray(X);Y=np.asarray(Y);G=np.asarray(G)
    if len(X)==0:raise RuntimeError('No training samples generated')
    rng=np.random.default_rng(42);unique=np.unique(G);rng.shuffle(unique);n=len(unique);ntr=max(1,int(.70*n));nval=max(1,int(.15*n)) if n>=3 else 0
    split=np.full(len(G),2);split[np.isin(G,unique[:ntr])]=0
    if nval:split[np.isin(G,unique[ntr:ntr+nval])]=1
    xm=X[split==0].mean(0);xs=X[split==0].std(0);ym=Y[split==0].mean(0);ys=Y[split==0].std(0);xs=np.maximum(xs,1e-8);ys=np.maximum(ys,1e-8)
    np.savez_compressed(a.out,X=X,Y=Y,groups=G,split=split,xm=xm,xs=xs,ym=ym,ys=ys,dt=a.dt,history=a.history)
    Path(str(a.out)+'.json').write_text(json.dumps({'samples':len(X),'runs':len(unique),'train':int((split==0).sum()),'validation':int((split==1).sum()),'test':int((split==2).sum())},indent=2))
    print('saved',a.out,'X',X.shape,'Y',Y.shape,'runs',len(unique))
if __name__=='__main__':main()
