#!/usr/bin/env python3
import argparse,numpy as np
p=argparse.ArgumentParser();p.add_argument('input');p.add_argument('output');a=p.parse_args();d=np.load(a.input);z=d['z'];u=d['u'];dz=z[1:]-z[:-1];np.savez(a.output,x=np.c_[z[:-1],u[:-1]],y=dz);print('saved',len(dz),'samples')
