#!/usr/bin/env python3
import argparse, numpy as np

def main():
    p=argparse.ArgumentParser(); p.add_argument('csv'); p.add_argument('--speed-limit',type=float,default=5.5556); a=p.parse_args()
    d=np.genfromtxt(a.csv,delimiter=',',names=True)
    checks={
      'all_finite': np.isfinite(np.column_stack([d[n] for n in d.dtype.names])).all(),
      's_strictly_increasing': (np.diff(d['s'])>0).all(),
      'speed_nonnegative': (d['reference_speed']>=0).all(),
      'speed_below_limit': (d['reference_speed']<=a.speed_limit+1e-6).all(),
      'yaw_continuous': (np.abs(np.diff(d['yaw']))<np.pi).all(),
    }
    for k,v in checks.items(): print(f"{'PASS' if v else 'FAIL'} {k}")
    raise SystemExit(0 if all(checks.values()) else 1)
if __name__=='__main__': main()
