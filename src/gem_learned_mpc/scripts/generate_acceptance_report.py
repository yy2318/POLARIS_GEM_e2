#!/usr/bin/env python3
import argparse, json, yaml
from pathlib import Path
import matplotlib.pyplot as plt
import numpy as np
from gem_learned_mpc.validation import load_log,calculate_metrics,evaluate,write_json
p=argparse.ArgumentParser();p.add_argument('csv');p.add_argument('--config',required=True);p.add_argument('--output-dir',required=True);a=p.parse_args()
rows=load_log(a.csv);cfg=yaml.safe_load(open(a.config));metrics=calculate_metrics(rows,cfg['speed_limit_mps']);checks,passed=evaluate(metrics,cfg);out=Path(a.output_dir);out.mkdir(parents=True,exist_ok=True)
write_json(out/'metrics.json',metrics);write_json(out/'acceptance.json',{'passed':passed,'checks':checks})
t=np.array([float(r['timestamp']) for r in rows]);t=t-t[0];cte=np.array([float(r['cte']) for r in rows]);speed=np.array([float(r['speed']) for r in rows]);solve=np.array([float(r['solver_time_ms']) for r in rows])
for y,title,label,name,limits in [(cte,'Cross-track error','CTE [m]','cross_track_error.png',[1,-1]),(speed,'Vehicle speed','Speed [m/s]','speed_constraint.png',[cfg['speed_limit_mps']]),(solve,'MPC solve time','Time [ms]','solver_time.png',[cfg['solver_p95_time_max_ms']])]:
 plt.figure();plt.plot(t,y);[plt.axhline(v,color='r',linestyle='--') for v in limits];plt.xlabel('Time [s]');plt.ylabel(label);plt.title(title);plt.grid(True);plt.tight_layout();plt.savefig(out/name,dpi=160);plt.close()
md=['# MPC Test and Acceptance Report','','Overall result: **%s**'%('PASS' if passed else 'FAIL'),'','## Metrics']+[f'- {k}: {v}' for k,v in metrics.items()]+['','## Checks']+[f'- [{"x" if v else " "}] {k}' for k,v in checks.items()]
(out/'acceptance_report.md').write_text('\n'.join(md),encoding='utf-8');print('PASS' if passed else 'FAIL');print(json.dumps(metrics,indent=2))
