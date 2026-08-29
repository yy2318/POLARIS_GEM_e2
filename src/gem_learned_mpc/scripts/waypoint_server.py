#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from tf.transformations import quaternion_from_euler
from gem_learned_mpc.path_geometry import load_waypoints,preprocess
rospy.init_node('waypoint_server');p=preprocess(load_waypoints(rospy.get_param('~waypoint_file')),rospy.get_param('~ds',.25));m=Path();m.header.frame_id=rospy.get_param('~frame_id','world')
for x,y,a in zip(p['x'],p['y'],p['yaw']):q=PoseStamped();q.header.frame_id=m.header.frame_id;q.pose.position.x=x;q.pose.position.y=y;z=quaternion_from_euler(0,0,a);q.pose.orientation.x,q.pose.orientation.y,q.pose.orientation.z,q.pose.orientation.w=z;m.poses.append(q)
pub=rospy.Publisher('~path',Path,latch=True,queue_size=1);pub.publish(m);rospy.loginfo('published %d points',len(m.poses));rospy.spin()
