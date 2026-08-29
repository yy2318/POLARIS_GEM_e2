#!/usr/bin/env python3
import csv, math, os
import rospy
from gazebo_msgs.msg import ModelStates
from gem_learned_mpc.msg import VehicleState, ControlCommand, MPCStatus

class Logger:
    def __init__(self):
        self.state=None; self.cmd=None; self.status=None
        path=rospy.get_param('~output_csv','/tmp/gem_mpc_acceptance.csv')
        os.makedirs(os.path.dirname(path) or '.',exist_ok=True)
        self.f=open(path,'w',newline=''); self.w=csv.writer(self.f)
        self.w.writerow(['timestamp','x','y','yaw','speed','nearest_path_index','reference_x','reference_y','reference_yaw','reference_speed','cte','heading_error','speed_error','steering_command','acceleration_command','solver_status','solver_time_ms','iteration_count','state_valid','fallback_active'])
        rospy.Subscriber('/gem_learned_mpc/state',VehicleState,self.state_cb,queue_size=1)
        rospy.Subscriber('/gem_learned_mpc/control_cmd',ControlCommand,self.cmd_cb,queue_size=1)
        rospy.Subscriber('/gem_learned_mpc/status',MPCStatus,self.status_cb,queue_size=1)
        rospy.on_shutdown(self.close)
    def state_cb(self,m): self.state=m
    def cmd_cb(self,m): self.cmd=m
    def status_cb(self,m):
        self.status=m
        if self.state is None or self.cmd is None:return
        s=self.state;c=self.cmd;q=self.status
        speed=math.hypot(s.vx,s.vy)
        self.w.writerow([rospy.Time.now().to_sec(),s.x,s.y,s.yaw,speed,q.nearest_path_index,'nan','nan','nan',q.reference_speed,q.cross_track_error,q.heading_error,q.reference_speed-speed,c.steering_angle,c.longitudinal_acceleration,q.solver_status,q.solve_time_ms,getattr(q,'iteration_count',0),s.valid,q.fallback_active])
        self.f.flush()
    def close(self):
        if not self.f.closed:self.f.close()
rospy.init_node('acceptance_logger');Logger();rospy.spin()
