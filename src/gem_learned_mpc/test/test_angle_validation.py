import numpy as np
from gem_learned_mpc.angles import wrap_to_pi
assert abs(float(wrap_to_pi(np.pi+0.1))-(-np.pi+0.1))<1e-9;assert abs(float(wrap_to_pi(-np.pi-0.1))-(np.pi-0.1))<1e-9
print('PASS angle validation')
