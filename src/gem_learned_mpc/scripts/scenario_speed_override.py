#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64
rospy.init_node('scenario_speed_override');v=float(rospy.get_param('~target_speed_mps'));pub=rospy.Publisher('/gem_learned_mpc/scenario_target_speed',Float64,latch=True,queue_size=1);pub.publish(v);rospy.loginfo('Published scenario target speed %.3f m/s',v);rospy.spin()
