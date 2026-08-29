#!/usr/bin/env python3
import argparse,numpy as np
from gem_learned_mpc.model import MLPModel
p=argparse.ArgumentParser();p.add_argument('run');p.add_argument('model');p.add_argument('--horizons',nargs='+',type=int,default=[10,20]);a=p.parse_args();d=np.load(a.run);m=MLPModel(a.model)
for H in a.horizons:
 es=[]
 for i in range(len(d['z'])-H):
  z=d['z'][i].copy()
  for j in range(H):z=z+m.delta(z,d['u'][i+j])
  es.append(np.linalg.norm(z[:2]-d['z'][i+H,:2]))
 print(H,'step local-state RMSE',np.sqrt(np.mean(np.square(es))))
