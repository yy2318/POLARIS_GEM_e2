import math
from gem_learned_mpc.scenario_geometry import pose_with_frenet_offset
x,y,p=pose_with_frenet_offset(0,0,0,.5,-10);assert abs(x)<1e-12 and abs(y-.5)<1e-12 and abs(p-math.radians(-10))<1e-12
x,y,p=pose_with_frenet_offset(0,0,math.pi/2,.5,0);assert abs(x+.5)<1e-12 and abs(y)<1e-12
print('PASS scenario geometry')
