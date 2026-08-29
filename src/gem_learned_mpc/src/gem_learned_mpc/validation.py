import csv, json, math
from pathlib import Path
import numpy as np

REQUIRED_COLUMNS = (
    'timestamp','x','y','yaw','speed','nearest_path_index','reference_x',
    'reference_y','reference_yaw','reference_speed','cte','heading_error',
    'speed_error','steering_command','acceleration_command','solver_status',
    'solver_time_ms','iteration_count','state_valid','fallback_active'
)

def finite(name, value):
    value=float(value)
    if not math.isfinite(value): raise ValueError('%s is not finite' % name)
    return value

def load_log(path):
    with open(path, newline='') as f:
        rows=list(csv.DictReader(f))
    if not rows: raise ValueError('validation log is empty')
    missing=[c for c in REQUIRED_COLUMNS if c not in rows[0]]
    if missing: raise ValueError('missing columns: %s' % ', '.join(missing))
    return rows

def calculate_metrics(rows, cte_limit=1.0, speed_limit=5.5556):
    valid=[r for r in rows if str(r['state_valid']).lower() in ('1','true','yes')]
    if not valid: raise ValueError('no valid state samples')
    cte=np.array([abs(finite('cte',r['cte'])) for r in valid])
    speed=np.array([finite('speed',r['speed']) for r in valid])
    solve=np.array([finite('solver_time_ms',r['solver_time_ms']) for r in valid])
    ok=np.array([r['solver_status']=='SOLVED' for r in valid],dtype=float)
    idx=np.array([int(float(r['nearest_path_index'])) for r in valid])
    completion=100.0 if idx.max()==0 else 100.0*(idx[-1]-idx[0])/max(1,idx.max()-idx[0])
    return {
        'sample_count':len(valid),
        'cte_rmse_m':float(np.sqrt(np.mean(cte**2))),
        'cte_mae_m':float(np.mean(cte)),
        'cte_max_m':float(np.max(cte)),
        'cte_p95_m':float(np.percentile(cte,95)),
        'samples_within_1m_pct':float(100*np.mean(cte<=cte_limit)),
        'max_speed_mps':float(np.max(speed)),
        'mean_solver_time_ms':float(np.mean(solve)),
        'p95_solver_time_ms':float(np.percentile(solve,95)),
        'solver_success_pct':float(100*np.mean(ok)),
        'solver_failure_count':int(np.sum(1-ok)),
        'path_completion_pct':float(np.clip(completion,0,100)),
        'nan_or_inf_count':0,
        'fallback_count':sum(str(r['fallback_active']).lower() in ('1','true','yes') for r in valid),
        'speed_limit_mps':speed_limit,
    }

def evaluate(metrics, cfg):
    checks={
        'max_speed': metrics['max_speed_mps'] <= cfg['speed_limit_mps']+cfg['speed_tolerance_mps'],
        'cte_95pct': metrics['samples_within_1m_pct'] >= cfg['samples_within_cte_pct'],
        'transient_cte': metrics['cte_max_m'] <= cfg['transient_cte_max_m'],
        'cte_rmse': metrics['cte_rmse_m'] <= cfg['cte_rmse_max_m'],
        'completion': metrics['path_completion_pct'] >= cfg['path_completion_min_pct'],
        'solver_success': metrics['solver_success_pct'] >= cfg['solver_success_min_pct'],
        'solver_time': metrics['p95_solver_time_ms'] < cfg['solver_p95_time_max_ms'],
        'finite': metrics['nan_or_inf_count']==0,
    }
    return checks, all(checks.values())

def write_json(path, obj):
    Path(path).parent.mkdir(parents=True,exist_ok=True)
    with open(path,'w') as f: json.dump(obj,f,indent=2,sort_keys=True)
