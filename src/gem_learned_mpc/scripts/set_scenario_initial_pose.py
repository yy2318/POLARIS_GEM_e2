#!/usr/bin/env python3
import argparse,yaml,rospy
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState
from tf.transformations import quaternion_from_euler
from gem_learned_mpc.scenario_geometry import load_start_pose

def main():
 p=argparse.ArgumentParser();p.add_argument('--config',required=True);p.add_argument('--scenario',required=True);a=rospy.myargv()[1:];args=p.parse_args(a)
 cfg=yaml.safe_load(open(args.config));c=cfg['common'];s=cfg['scenarios'][args.scenario]
 x,y,yaw=load_start_pose(c['waypoint_file'],c.get('start_waypoint_index',0),s['lateral_offset_m'],s['heading_offset_deg'])
 rospy.init_node('set_scenario_initial_pose',anonymous=True);service=c.get('set_model_state_service','/gazebo/set_model_state');rospy.wait_for_service(service,timeout=20)
 m=ModelState();m.model_name=c['vehicle_model_name'];m.reference_frame=c.get('reference_frame','world');m.pose.position.x=x;m.pose.position.y=y;m.pose.position.z=0.0;q=quaternion_from_euler(0,0,yaw);m.pose.orientation.x,m.pose.orientation.y,m.pose.orientation.z,m.pose.orientation.w=q;m.twist.linear.x=0;m.twist.linear.y=0;m.twist.angular.z=0
 r=rospy.ServiceProxy(service,SetModelState)(m)
 if not r.success: raise RuntimeError(r.status_message)
 rospy.loginfo('Scenario %s pose set: x=%.6f y=%.6f yaw=%.6f speed=%.2f',args.scenario,x,y,yaw,s['target_speed_mps'])
if __name__=='__main__':main()
