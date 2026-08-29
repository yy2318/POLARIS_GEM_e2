import math
import numpy as np

def pose_with_frenet_offset(x_ref,y_ref,yaw_ref,lateral_offset_m,heading_offset_deg):
    """Positive offset places the vehicle on the path's left side."""
    x=x_ref-math.sin(yaw_ref)*lateral_offset_m
    y=y_ref+math.cos(yaw_ref)*lateral_offset_m
    yaw=yaw_ref+math.radians(heading_offset_deg)
    yaw=(yaw+math.pi)%(2*math.pi)-math.pi
    return float(x),float(y),float(yaw)

def load_start_pose(csv_path,index=0,offset=0.0,heading_deg=0.0):
    a=np.loadtxt(csv_path,delimiter=',',ndmin=2)
    if a.shape[1]<2 or len(a)<2: raise ValueError('wps.csv requires at least two rows and x,y columns')
    index=max(0,min(int(index),len(a)-2));x,y=a[index,:2]
    if a.shape[1]>=3 and np.isfinite(a[index,2]): yaw=float(a[index,2])
    else: yaw=math.atan2(a[index+1,1]-y,a[index+1,0]-x)
    if not np.isfinite([x,y,yaw]).all(): raise ValueError('non-finite start pose')
    return pose_with_frenet_offset(x,y,yaw,offset,heading_deg)
