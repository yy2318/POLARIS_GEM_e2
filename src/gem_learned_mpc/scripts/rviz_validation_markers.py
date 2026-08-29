#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
from gem_learned_mpc.msg import VehicleState, MPCStatus
class N:
 def __init__(self):
  self.s=None;self.q=None;self.pub=rospy.Publisher('/gem_learned_mpc/cte_marker',Marker,queue_size=1);rospy.Subscriber('/gem_learned_mpc/state',VehicleState,self.scb);rospy.Subscriber('/gem_learned_mpc/status',MPCStatus,self.qcb)
 def scb(self,m):self.s=m;self.draw()
 def qcb(self,m):self.q=m
 def draw(self):
  if self.s is None or self.q is None:return
  m=Marker();m.header=self.s.header;m.ns='cte';m.id=0;m.type=Marker.ARROW;m.action=Marker.ADD;m.scale.x=.08;m.scale.y=.15;m.color.r=.7;m.color.b=1;m.color.a=1
  p=Point(self.s.x,self.s.y,0.2);e=Point(self.s.x-self.q.cross_track_error*0.0,self.s.y-self.q.cross_track_error,0.2);m.points=[p,e];self.pub.publish(m)
rospy.init_node('rviz_validation_markers');N();rospy.spin()
