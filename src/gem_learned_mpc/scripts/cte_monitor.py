#!/usr/bin/env python3
import rospy,csv
from gem_learned_mpc.msg import MPCStatus
f=open(rospy.get_param('~output_csv','/tmp/gem_mpc_cte.csv'),'w');w=csv.writer(f);w.writerow(['time','cte','heading_error','reference_speed','solve_time_ms','status'])
def cb(m):w.writerow([m.header.stamp.to_sec(),m.cross_track_error,m.heading_error,m.reference_speed,m.solve_time_ms,m.solver_status]);f.flush()
rospy.init_node('cte_monitor');rospy.Subscriber('~status',MPCStatus,cb);rospy.spin()
