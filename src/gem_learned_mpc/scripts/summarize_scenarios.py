#!/usr/bin/env python3
import argparse,json
from pathlib import Path
p=argparse.ArgumentParser();p.add_argument('--results-root',required=True);a=p.parse_args();root=Path(a.results_root);names=['nominal','left_offset','right_offset','heading_error','combined','high_speed','solver_fault'];rows=[]
for n in names:
 f=root/(n+'_report')/'acceptance.json';m=root/(n+'_report')/'metrics.json'
 if f.exists() and m.exists():
  aa=json.load(open(f));mm=json.load(open(m));rows.append((n,aa['passed'],mm.get('cte_rmse_m'),mm.get('cte_max_m'),mm.get('max_speed_mps'),mm.get('solver_success_pct'),mm.get('p95_solver_time_ms')))
 else:rows.append((n,False,None,None,None,None,None))
out=root/'scenario_summary.md';lines=['# Simulation Scenario Summary','','| Scenario | Result | CTE RMSE | Max CTE | Max speed | Solver success | P95 solver time |','|---|---:|---:|---:|---:|---:|---:|']
for r in rows:lines.append('| %s | %s | %s | %s | %s | %s | %s |'%((r[0],'PASS' if r[1] else 'FAIL')+tuple('N/A' if v is None else ('%.4f'%v) for v in r[2:])))
out.write_text('\n'.join(lines),encoding='utf-8');print(out)
