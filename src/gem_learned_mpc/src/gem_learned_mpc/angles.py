import numpy as np
def wrap_to_pi(a): return (np.asarray(a)+np.pi)%(2*np.pi)-np.pi
