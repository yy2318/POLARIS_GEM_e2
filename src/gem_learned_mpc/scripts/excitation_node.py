#!/usr/bin/env python3
import math, random, rospy
from ackermann_msgs.msg import AckermannDrive,AckermannDriveStamped
from std_msgs.msg import Float64
class Exciter:
    def __init__(self):
        self.topic=rospy.get_param('~command_topic','/ackermann_cmd')
        self.typ=rospy.get_param('~command_type','ackermann_msgs/AckermannDrive')
        self.mode=rospy.get_param('~mode','prbs').lower();self.rate=float(rospy.get_param('~rate',20.0))
        self.duration=float(rospy.get_param('~duration',300.0));self.delay=float(rospy.get_param('~start_delay',3.0))
        self.vmax=min(5.5556,float(rospy.get_param('~speed_max',2.0)));self.dmax=abs(float(rospy.get_param('~steering_max',0.15)))
        self.v_period=float(rospy.get_param('~speed_period',8.0));self.d_period=float(rospy.get_param('~steering_period',3.0))
        self.freq=float(rospy.get_param('~frequency',0.08));random.seed(int(rospy.get_param('~seed',7)))
        cls=AckermannDrive if self.typ.endswith('/AckermannDrive') else AckermannDriveStamped
        self.pub=rospy.Publisher(self.topic,cls,queue_size=1);self.mon=rospy.Publisher('/system_id/commanded_steering',Float64,queue_size=20)
        self.v=0.0;self.d=0.0;self.tv=-1e9;self.td=-1e9;rospy.on_shutdown(self.stop)
    def signal(self,t):
        if self.mode=='step':
            levels=[0.0,.5,1.5,2.5,4.0,min(5.0,self.vmax),0.0];v=min(self.vmax,levels[int(t/self.v_period)%len(levels)])
            ds=[0.0,.05,0.0,-.05,0.0];d=max(-self.dmax,min(self.dmax,ds[int(t/self.d_period)%len(ds)]))
        elif self.mode=='brake':
            phase=t%12.0;v=min(3.0,self.vmax) if phase<7.0 else 0.0;d=0.0
        elif self.mode=='sine':
            v=min(self.vmax,max(.2,.65*self.vmax));d=self.dmax*math.sin(2*math.pi*self.freq*t)
        else:
            if t-self.tv>=self.v_period:self.v=random.choice([0.5,min(1.5,self.vmax),min(2.5,self.vmax),self.vmax]);self.tv=t
            if t-self.td>=self.d_period:self.d=random.choice([-self.dmax,-self.dmax/2,0,self.dmax/2,self.dmax]);self.td=t
            v,d=self.v,self.d
        return max(0,min(self.vmax,v)),max(-self.dmax,min(self.dmax,d))
    def send(self,v,d):
        if self.typ.endswith('/AckermannDrive'):
            m=AckermannDrive();m.speed=v;m.steering_angle=d;m.acceleration=1.0;m.steering_angle_velocity=.5
        else:
            m=AckermannDriveStamped();m.header.stamp=rospy.Time.now();m.drive.speed=v;m.drive.steering_angle=d;m.drive.acceleration=1.0;m.drive.steering_angle_velocity=.5
        self.pub.publish(m);self.mon.publish(Float64(d))
    def stop(self):
        try:
            for _ in range(5):self.send(0,0);rospy.sleep(.02)
        except:pass
    def run(self):
        r=rospy.Rate(self.rate);start=rospy.Time.now()
        while not rospy.is_shutdown() and (rospy.Time.now()-start).to_sec()<self.delay:self.send(0,0);r.sleep()
        start=rospy.Time.now()
        while not rospy.is_shutdown():
            t=(rospy.Time.now()-start).to_sec()
            if t>=self.duration:break
            self.send(*self.signal(t));r.sleep()
        self.stop()
if __name__=='__main__':rospy.init_node('excitation_node');Exciter().run()
