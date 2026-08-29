#!/usr/bin/env python3
import argparse,numpy as np
from gem_learned_mpc.model import MLPModel
p=argparse.ArgumentParser();p.add_argument('dataset');p.add_argument('model');a=p.parse_args();d=np.load(a.dataset);m=MLPModel(a.model);pred=np.array([m.delta(x[:5],x[5:]) for x in d['x']]);print('RMSE',np.sqrt(np.mean((pred-d['y'])**2,axis=0)))
