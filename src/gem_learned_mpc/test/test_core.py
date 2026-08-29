#!/usr/bin/env python3
import numpy as np
from gem_learned_mpc.angles import wrap_to_pi
from gem_learned_mpc.path_geometry import preprocess,frenet_error,speed_profile
from gem_learned_mpc.model import MLPModel,step
assert abs(float(wrap_to_pi(3*np.pi))+np.pi)<1e-9;p=preprocess(np.array([[0,0],[0,0],[1,0],[2,0.]],float),.25);assert len(p['x'])==9;assert abs(frenet_error(0,1,0,0,0,0)[0]-1)<1e-9;assert speed_profile(np.array([1.]))[0]<5.5556;x=step(np.array([0,0,0,1,0,0,0,0.]),[0,0],MLPModel());assert x.shape==(8,) and np.isfinite(x).all();print('PASS: gem_learned_mpc core tests')
