import numpy as np
from gem_learned_mpc.path_geometry import preprocess,nearest_index
raw=np.array([[0,0,0,0,0],[0,0,0,0,0],[1,0,0,0,0],[2,1,0,0,0]],float)
p=preprocess(raw,.25);assert np.isfinite(p['kappa']).all();assert np.max(np.abs(np.diff(p['yaw'])))<np.pi;assert nearest_index(p,0,0)==0;assert np.all(np.diff(p['s'])>0)
print('PASS path validation')
