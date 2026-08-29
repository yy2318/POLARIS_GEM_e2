#!/usr/bin/env python3
"""Publish a unified GEM VehicleState from Gazebo /gazebo/model_states.

Input:
    gazebo_msgs/ModelStates, default /gazebo/model_states
Output:
    gem_learned_mpc/VehicleState, default /gem_learned_mpc/state

The linear velocity in gazebo_msgs/ModelStates is treated as world-frame
velocity and rotated into the vehicle body frame before publishing vx and vy.
"""

import math
from typing import Optional

import rospy
from gazebo_msgs.msg import ModelStates
from tf.transformations import euler_from_quaternion

from gem_learned_mpc.msg import VehicleState


class GazeboModelStatesAdapter:
    """Convert one model entry in gazebo_msgs/ModelStates to VehicleState."""

    def __init__(self) -> None:
        self.model_states_topic = rospy.get_param(
            "~model_states_topic", "/gazebo/model_states"
        )
        self.state_topic = rospy.get_param(
            "~state_topic", "/gem_learned_mpc/state"
        )
        self.model_name = rospy.get_param("~model_name", "gem_e2")
        self.frame_id = rospy.get_param("~frame_id", "world")

        self.acceleration_filter_alpha = float(
            rospy.get_param("~acceleration_filter_alpha", 0.8)
        )
        if not 0.0 <= self.acceleration_filter_alpha < 1.0:
            raise ValueError("~acceleration_filter_alpha must be in [0.0, 1.0)")

        self.use_commanded_steering_fallback = bool(
            rospy.get_param("~use_commanded_steering_fallback", False)
        )
        self.commanded_steering_angle = float(
            rospy.get_param("~initial_steering_angle", 0.0)
        )

        self.previous_time: Optional[rospy.Time] = None
        self.previous_vx: Optional[float] = None
        self.filtered_acceleration = 0.0
        self.last_missing_model_warning = rospy.Time(0)
        self.last_invalid_state_warning = rospy.Time(0)

        self.publisher = rospy.Publisher(
            self.state_topic,
            VehicleState,
            queue_size=10,
        )
        self.subscriber = rospy.Subscriber(
            self.model_states_topic,
            ModelStates,
            self.model_states_callback,
            queue_size=1,
            tcp_nodelay=True,
        )

        rospy.loginfo(
            "Gazebo state adapter started: input=%s model=%s output=%s frame=%s",
            self.model_states_topic,
            self.model_name,
            self.state_topic,
            self.frame_id,
        )

    @staticmethod
    def quaternion_to_yaw(orientation) -> float:
        """Convert geometry_msgs/Quaternion to yaw in radians."""
        return float(
            euler_from_quaternion(
                [
                    orientation.x,
                    orientation.y,
                    orientation.z,
                    orientation.w,
                ]
            )[2]
        )

    @staticmethod
    def world_to_body_velocity(
        vx_world: float,
        vy_world: float,
        yaw: float,
    ):
        """Rotate planar world-frame velocity into the vehicle body frame."""
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)

        vx_body = cos_yaw * vx_world + sin_yaw * vy_world
        vy_body = -sin_yaw * vx_world + cos_yaw * vy_world
        return vx_body, vy_body

    def estimate_acceleration(self, timestamp: rospy.Time, vx: float) -> float:
        """Estimate longitudinal acceleration using filtered finite difference."""
        if self.previous_time is None or self.previous_vx is None:
            self.previous_time = timestamp
            self.previous_vx = vx
            self.filtered_acceleration = 0.0
            return 0.0

        dt = (timestamp - self.previous_time).to_sec()
        self.previous_time = timestamp

        if dt <= 1.0e-4 or dt > 1.0:
            self.previous_vx = vx
            return self.filtered_acceleration

        raw_acceleration = (vx - self.previous_vx) / dt
        self.previous_vx = vx

        alpha = self.acceleration_filter_alpha
        self.filtered_acceleration = (
            alpha * self.filtered_acceleration
            + (1.0 - alpha) * raw_acceleration
        )
        return self.filtered_acceleration

    def warn_missing_model(self, names) -> None:
        now = rospy.Time.now()
        if (now - self.last_missing_model_warning).to_sec() >= 5.0:
            rospy.logwarn(
                "Model '%s' was not found in %s. Available models: [%s]",
                self.model_name,
                self.model_states_topic,
                ", ".join(names),
            )
            self.last_missing_model_warning = now

    def warn_invalid_state(self, values) -> None:
        now = rospy.Time.now()
        if (now - self.last_invalid_state_warning).to_sec() >= 2.0:
            rospy.logwarn("Discarding non-finite GEM state: %s", values)
            self.last_invalid_state_warning = now

    def model_states_callback(self, message: ModelStates) -> None:
        try:
            index = message.name.index(self.model_name)
        except ValueError:
            self.warn_missing_model(message.name)
            return

        if index >= len(message.pose) or index >= len(message.twist):
            rospy.logerr_throttle(
                2.0,
                "Invalid ModelStates arrays: model index=%d pose=%d twist=%d",
                index,
                len(message.pose),
                len(message.twist),
            )
            return

        pose = message.pose[index]
        twist = message.twist[index]
        timestamp = rospy.Time.now()

        yaw = self.quaternion_to_yaw(pose.orientation)
        vx_body, vy_body = self.world_to_body_velocity(
            twist.linear.x,
            twist.linear.y,
            yaw,
        )
        acceleration = self.estimate_acceleration(timestamp, vx_body)

        # ModelStates does not contain steering-joint position. Keep zero unless
        # a separate steering feedback subscriber is added later.
        steering_angle = (
            self.commanded_steering_angle
            if self.use_commanded_steering_fallback
            else 0.0
        )

        values = [
            pose.position.x,
            pose.position.y,
            yaw,
            vx_body,
            vy_body,
            twist.angular.z,
            steering_angle,
            acceleration,
        ]
        valid = all(math.isfinite(value) for value in values)
        if not valid:
            self.warn_invalid_state(values)
            return

        state = VehicleState()
        state.header.stamp = timestamp
        state.header.frame_id = self.frame_id
        state.x = float(pose.position.x)
        state.y = float(pose.position.y)
        state.yaw = yaw
        state.vx = float(vx_body)
        state.vy = float(vy_body)
        state.yaw_rate = float(twist.angular.z)
        state.steering_angle = float(steering_angle)
        state.longitudinal_acceleration = float(acceleration)
        state.valid = True
        self.publisher.publish(state)


def main() -> None:
    rospy.init_node("state_adapter")
    GazeboModelStatesAdapter()
    rospy.spin()


if __name__ == "__main__":
    main()
