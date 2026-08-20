#!/usr/bin/env python3
import math, rospy
from gazebo_msgs.msg import ModelStates
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from tf.transformations import euler_from_quaternion
from gem_learned_mpc.msg import VehicleState

class Adapter:
    def __init__(self):
        self.model=rospy.get_param('~model_name')
        self.model_topic=rospy.get_param('~model_states_topic','/gazebo/model_states')
        self.joint_topic=rospy.get_param('~joint_states_topic','/joint_states')
        self.output=rospy.get_param('~output_topic','/system_id/vehicle_state')
        self.joints=rospy.get_param('~steering_joint_names',[])
        self.steer=float('nan'); self.cmd_steer=0.0
        self.pub=rospy.Publisher(self.output,VehicleState,queue_size=50)
        rospy.Subscriber(self.model_topic,ModelStates,self.on_model,queue_size=5)
        rospy.Subscriber(self.joint_topic,JointState,self.on_joint,queue_size=20)
        rospy.Subscriber('/system_id/commanded_steering',Float64,self.on_cmd,queue_size=20)
    def on_cmd(self,m): self.cmd_steer=float(m.data)
    def on_joint(self,m):
        vals=[]
        for n in self.joints:
            if n in m.name:
                i=m.name.index(n)
                if i<len(m.position) and math.isfinite(m.position[i]): vals.append(m.position[i])
        if vals:self.steer=sum(vals)/len(vals)
    def on_model(self,m):
        o=VehicleState();o.header.stamp=rospy.Time.now();o.header.frame_id='world';o.valid=False
        if self.model not in m.name:
            rospy.logwarn_throttle(5.0,"Model '%s' not found",self.model);self.pub.publish(o);return
        i=m.name.index(self.model);p=m.pose[i];t=m.twist[i];q=p.orientation
        _,_,yaw=euler_from_quaternion([q.x,q.y,q.z,q.w]);c=math.cos(yaw);s=math.sin(yaw)
        vals=[p.position.x,p.position.y,yaw,c*t.linear.x+s*t.linear.y,-s*t.linear.x+c*t.linear.y,t.angular.z,self.steer if math.isfinite(self.steer) else self.cmd_steer]
        if not all(math.isfinite(v) for v in vals):self.pub.publish(o);return
        o.x,o.y,o.yaw,o.vx,o.vy,o.yaw_rate,o.steering_angle=vals;o.valid=True;self.pub.publish(o)
if __name__=='__main__':
    rospy.init_node('state_adapter');Adapter();rospy.spin()
