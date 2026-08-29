#!/usr/bin/env python3
import rospy
from ackermann_msgs.msg import AckermannDriveStamped
from gem_learned_mpc.msg import ControlCommand
pub=None
def cb(m):
 o=AckermannDriveStamped();o.header=m.header;o.drive.steering_angle=m.steering_angle;o.drive.acceleration=m.longitudinal_acceleration;o.drive.speed=max(0.,min(5.5556,m.desired_speed));pub.publish(o)
rospy.init_node('command_adapter');pub=rospy.Publisher(rospy.get_param('~output_topic','/ackermann_cmd'),AckermannDriveStamped,queue_size=1);rospy.Subscriber('~control',ControlCommand,cb,queue_size=1);rospy.spin()
