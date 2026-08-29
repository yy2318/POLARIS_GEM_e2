#!/usr/bin/env python3
import rospy
from std_msgs.msg import Bool,String
rospy.init_node('mpc_fault_injector');kind=rospy.get_param('~fault_type','solver_failure');start=float(rospy.get_param('~fault_start_s',20));duration=float(rospy.get_param('~fault_duration_s',5));fp=rospy.Publisher('/gem_learned_mpc/fault_active',Bool,latch=True,queue_size=1);tp=rospy.Publisher('/gem_learned_mpc/fault_type',String,latch=True,queue_size=1);tp.publish(kind);fp.publish(False);rospy.sleep(start);rospy.logwarn('Injecting %s for %.2f seconds',kind,duration);fp.publish(True);rospy.sleep(duration);fp.publish(False);rospy.logwarn('Fault cleared');rospy.spin()
