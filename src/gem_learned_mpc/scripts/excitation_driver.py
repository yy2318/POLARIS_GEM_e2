#!/usr/bin/env python3
import rospy,numpy as np
from gem_learned_mpc.msg import ControlCommand
rospy.init_node('excitation_driver');pub=rospy.Publisher('~control',ControlCommand,queue_size=1);rate=rospy.Rate(rospy.get_param('~rate',20));t=0.;rng=np.random.RandomState(rospy.get_param('~seed',1));dc=ac=0.
while not rospy.is_shutdown():
 if int(t*2)!=int((t-.05)*2):dc=rng.uniform(-.35,.35);ac=rng.uniform(-1.,1.)
 m=ControlCommand();m.header.stamp=rospy.Time.now();m.steering_angle=dc;m.longitudinal_acceleration=ac;m.desired_speed=rospy.get_param('~desired_speed',3.0);m.enable=True;pub.publish(m);t+=1./20;rate.sleep()
