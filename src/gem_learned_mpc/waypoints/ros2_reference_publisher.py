#!/usr/bin/env python3
"""ROS 2 publisher for processed waypoint path and profile.
Publishes nav_msgs/Path on /mpc/reference_path and Float64MultiArray on
/mpc/reference_profile. The profile is flattened row-major as [yaw,kappa,v]*N.
"""
import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64MultiArray, MultiArrayDimension
from wps_processor import process


def yaw_to_quaternion(yaw):
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)

class ReferencePublisher(Node):
    def __init__(self):
        super().__init__('wps_reference_publisher')
        self.declare_parameter('csv_path', 'wps.csv')
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('min_distance', 0.02)
        self.declare_parameter('spacing', 0.25)
        self.declare_parameter('max_lateral_accel', 2.0)
        self.declare_parameter('speed_limit', 5.5556)
        self.declare_parameter('max_accel', 1.5)
        self.declare_parameter('max_decel', 2.0)
        self.declare_parameter('smooth_window', 5)
        q = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.path_pub = self.create_publisher(Path, '/mpc/reference_path', q)
        self.profile_pub = self.create_publisher(Float64MultiArray, '/mpc/reference_profile', q)
        self.publish_reference()
        self.timer = self.create_timer(1.0, self.publish_reference)

    def publish_reference(self):
        profile, stats = process(
            self.get_parameter('csv_path').value,
            self.get_parameter('min_distance').value,
            self.get_parameter('spacing').value,
            self.get_parameter('max_lateral_accel').value,
            self.get_parameter('speed_limit').value,
            self.get_parameter('max_accel').value,
            self.get_parameter('max_decel').value,
            self.get_parameter('smooth_window').value)
        stamp = self.get_clock().now().to_msg()
        frame = self.get_parameter('frame_id').value
        path = Path(); path.header.stamp = stamp; path.header.frame_id = frame
        for row in profile:
            pose = PoseStamped(); pose.header = path.header
            pose.pose.position.x = float(row[1]); pose.pose.position.y = float(row[2])
            qx,qy,qz,qw = yaw_to_quaternion(float(row[3]))
            pose.pose.orientation.x=qx; pose.pose.orientation.y=qy
            pose.pose.orientation.z=qz; pose.pose.orientation.w=qw
            path.poses.append(pose)
        values = profile[:, [3,4,7]].astype(float)
        msg = Float64MultiArray()
        msg.layout.dim = [MultiArrayDimension(label='points', size=len(values), stride=len(values)*3),
                          MultiArrayDimension(label='yaw_curvature_speed', size=3, stride=3)]
        msg.data = values.ravel().tolist()
        self.path_pub.publish(path); self.profile_pub.publish(msg)
        self.get_logger().info(f"Published {len(profile)} points; vmax={stats['max_reference_speed_mps']:.3f} m/s", throttle_duration_sec=5.0)

def main():
    rclpy.init(); node=ReferencePublisher()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally: node.destroy_node(); rclpy.shutdown()

if __name__ == '__main__': main()
