#!/usr/bin/env python3
"""ROS interface for learned MPC with scenario speed and fault injection.

Subscriptions:
  /gem_learned_mpc/state                 gem_learned_mpc/VehicleState
  /gem_learned_mpc/scenario_target_speed std_msgs/Float64
  /gem_learned_mpc/fault_active          std_msgs/Bool
  /gem_learned_mpc/fault_type            std_msgs/String (optional)

Publications:
  /gem_learned_mpc/control_cmd            gem_learned_mpc/ControlCommand
  /gem_learned_mpc/status                 gem_learned_mpc/MPCStatus
"""

import math
import threading
import time

import numpy as np
import rospy
import yaml
from std_msgs.msg import Bool, Float64, String

from gem_learned_mpc.msg import ControlCommand, MPCStatus, VehicleState
from gem_learned_mpc.mpc_solver import Solver
from gem_learned_mpc.path_geometry import (
    frenet_error,
    load_waypoints,
    nearest_index,
    preprocess,
    speed_profile,
)


class LearnedMPCNode:
    def __init__(self):
        self.lock = threading.RLock()

        config_path = rospy.get_param("~config")
        waypoint_file = rospy.get_param("~waypoint_file")
        model_json = rospy.get_param("~model_json", "")

        with open(config_path, "r") as stream:
            self.config = yaml.safe_load(stream)

        self.limits = self.config["limits"]
        self.horizon = int(self.config["horizon"])
        self.dt = float(self.config["dt"])

        self.state_timeout_s = float(
            rospy.get_param("~state_timeout_s", 0.25)
        )
        self.safe_deceleration = float(
            rospy.get_param("~safe_deceleration", -1.0)
        )
        self.safe_steering_angle = float(
            rospy.get_param("~safe_steering_angle", 0.0)
        )
        self.recovery_hold_s = float(
            rospy.get_param("~recovery_hold_s", 0.5)
        )
        self.control_rate_hz = float(
            rospy.get_param("~control_rate_hz", 1.0 / self.dt)
        )

        self.safe_deceleration = float(
            np.clip(
                self.safe_deceleration,
                self.limits["acceleration_min"],
                0.0,
            )
        )
        self.safe_steering_angle = float(
            np.clip(
                self.safe_steering_angle,
                -self.limits["steering_max"],
                self.limits["steering_max"],
            )
        )

        raw_path = load_waypoints(waypoint_file)
        self.path = preprocess(
            raw_path,
            float(rospy.get_param("~path_ds", 0.25)),
        )
        self.base_speed_profile = speed_profile(
            self.path["kappa"],
            vmax=float(self.limits["speed_max"]),
        )

        self.solver = Solver(self.config, model_json)

        self.latest_state = None
        self.latest_state_receive_time = None
        self.nearest_path_index = None

        self.scenario_speed_limit = None
        self.injected_fault_active = False
        self.injected_fault_type = "solver_failure"
        self.fault_clear_time = None

        self.last_control = np.array([0.0, 0.0], dtype=float)

        self.control_publisher = rospy.Publisher(
            "/gem_learned_mpc/control_cmd",
            ControlCommand,
            queue_size=1,
        )
        self.status_publisher = rospy.Publisher(
            "/gem_learned_mpc/status",
            MPCStatus,
            queue_size=1,
        )

        rospy.Subscriber(
            "/gem_learned_mpc/state",
            VehicleState,
            self.state_callback,
            queue_size=1,
            tcp_nodelay=True,
        )
        rospy.Subscriber(
            "/gem_learned_mpc/scenario_target_speed",
            Float64,
            self.scenario_speed_callback,
            queue_size=1,
        )
        rospy.Subscriber(
            "/gem_learned_mpc/fault_active",
            Bool,
            self.fault_callback,
            queue_size=1,
        )
        rospy.Subscriber(
            "/gem_learned_mpc/fault_type",
            String,
            self.fault_type_callback,
            queue_size=1,
        )

        self.timer = rospy.Timer(
            rospy.Duration(1.0 / self.control_rate_hz),
            self.control_timer_callback,
        )

        rospy.loginfo(
            "Learned MPC interface ready: speed_max=%.4f m/s, "
            "state_timeout=%.3f s, safe_deceleration=%.3f m/s^2",
            self.limits["speed_max"],
            self.state_timeout_s,
            self.safe_deceleration,
        )

    def state_callback(self, message):
        with self.lock:
            self.latest_state = message
            self.latest_state_receive_time = rospy.Time.now()

    def scenario_speed_callback(self, message):
        requested_speed = float(message.data)
        if not math.isfinite(requested_speed):
            rospy.logwarn("Ignoring non-finite scenario speed: %s", requested_speed)
            return

        speed_max = float(self.limits["speed_max"])
        bounded_speed = max(0.0, min(requested_speed, speed_max))

        with self.lock:
            self.scenario_speed_limit = bounded_speed

        if bounded_speed != requested_speed:
            rospy.logwarn(
                "Scenario speed %.4f was clamped to %.4f m/s",
                requested_speed,
                bounded_speed,
            )
        else:
            rospy.loginfo("Scenario speed limit set to %.4f m/s", bounded_speed)

    def fault_callback(self, message):
        now = rospy.Time.now()
        with self.lock:
            previous = self.injected_fault_active
            self.injected_fault_active = bool(message.data)
            if previous and not self.injected_fault_active:
                self.fault_clear_time = now

        if self.injected_fault_active:
            rospy.logwarn(
                "Injected MPC fault active: type=%s",
                self.injected_fault_type,
            )
        elif previous:
            rospy.logwarn(
                "Injected MPC fault cleared; holding safe control for %.2f s",
                self.recovery_hold_s,
            )

    def fault_type_callback(self, message):
        fault_type = str(message.data).strip() or "solver_failure"
        with self.lock:
            self.injected_fault_type = fault_type

    def state_is_fresh(self, now):
        if self.latest_state is None or self.latest_state_receive_time is None:
            return False
        age = (now - self.latest_state_receive_time).to_sec()
        return 0.0 <= age <= self.state_timeout_s

    @staticmethod
    def state_to_array(state):
        values = np.array(
            [
                state.x,
                state.y,
                state.yaw,
                state.vx,
                state.vy,
                state.yaw_rate,
                state.steering_angle,
                state.longitudinal_acceleration,
            ],
            dtype=float,
        )
        if not np.isfinite(values).all():
            raise ValueError("VehicleState contains NaN or Inf")
        return values

    def build_reference(self, current_state):
        self.nearest_path_index = nearest_index(
            self.path,
            current_state[0],
            current_state[1],
            self.nearest_path_index,
        )

        indices = np.clip(
            np.arange(
                self.nearest_path_index,
                self.nearest_path_index + self.horizon,
            ),
            0,
            len(self.path["x"]) - 1,
        )

        reference_speed = self.base_speed_profile[indices].copy()
        if self.scenario_speed_limit is not None:
            reference_speed = np.minimum(
                reference_speed,
                self.scenario_speed_limit,
            )

        reference_speed = np.minimum(
            reference_speed,
            float(self.limits["speed_max"]),
        )

        return np.column_stack(
            [
                self.path["x"][indices],
                self.path["y"][indices],
                self.path["yaw"][indices],
                reference_speed,
            ]
        )

    def recovery_hold_active(self, now):
        if self.fault_clear_time is None:
            return False
        return (now - self.fault_clear_time).to_sec() < self.recovery_hold_s

    def solve_or_fallback(self, current_state, reference, now):
        if self.injected_fault_active:
            return (
                np.array(
                    [self.safe_steering_angle, self.safe_deceleration],
                    dtype=float,
                ),
                "INJECTED_FAILURE",
                0.0,
                True,
            )

        if self.recovery_hold_active(now):
            return (
                np.array(
                    [self.safe_steering_angle, self.safe_deceleration],
                    dtype=float,
                ),
                "FAULT_RECOVERY_HOLD",
                0.0,
                True,
            )

        try:
            control, solver_status, solve_time_ms = self.solver.solve(
                current_state,
                reference,
            )
            control = np.asarray(control, dtype=float).reshape(2)
            if not np.isfinite(control).all():
                raise ValueError("MPC solver returned NaN or Inf")

            fallback_active = solver_status != "SOLVED"
            if fallback_active and control[1] > 0.0:
                control[1] = self.safe_deceleration

            return control, solver_status, solve_time_ms, fallback_active

        except Exception as exception:
            rospy.logerr_throttle(
                1.0,
                "MPC exception; applying safe fallback: %s",
                exception,
            )
            return (
                np.array(
                    [self.safe_steering_angle, self.safe_deceleration],
                    dtype=float,
                ),
                "SOLVER_EXCEPTION",
                0.0,
                True,
            )

    def apply_hard_limits(self, control):
        steering = float(
            np.clip(
                control[0],
                -self.limits["steering_max"],
                self.limits["steering_max"],
            )
        )
        acceleration = float(
            np.clip(
                control[1],
                self.limits["acceleration_min"],
                self.limits["acceleration_max"],
            )
        )
        return np.array([steering, acceleration], dtype=float)

    def publish_control(self, control, target_speed, enable=True):
        command = ControlCommand()
        command.header.stamp = rospy.Time.now()
        command.steering_angle = float(control[0])
        command.longitudinal_acceleration = float(control[1])
        command.desired_speed = float(
            np.clip(target_speed, 0.0, self.limits["speed_max"])
        )
        command.enable = bool(enable)
        self.control_publisher.publish(command)
        self.last_control = control.copy()

    def publish_status(
        self,
        current_state,
        reference,
        solver_status,
        solve_time_ms,
        fallback_active,
    ):
        cte, _, heading_error = frenet_error(
            current_state[0],
            current_state[1],
            current_state[2],
            reference[0, 0],
            reference[0, 1],
            reference[0, 2],
        )

        status = MPCStatus()
        status.header.stamp = rospy.Time.now()
        status.solver_status = str(solver_status)
        status.solve_time_ms = float(solve_time_ms)
        status.cross_track_error = float(cte)
        status.heading_error = float(heading_error)
        status.reference_speed = float(reference[0, 3])
        status.nearest_path_index = int(self.nearest_path_index or 0)
        status.fallback_active = bool(fallback_active)

        # Support an extended MPCStatus.msg without breaking the current one.
        if hasattr(status, "iteration_count"):
            status.iteration_count = 0
        if hasattr(status, "reference_x"):
            status.reference_x = float(reference[0, 0])
        if hasattr(status, "reference_y"):
            status.reference_y = float(reference[0, 1])
        if hasattr(status, "reference_yaw"):
            status.reference_yaw = float(reference[0, 2])

        self.status_publisher.publish(status)

    def publish_state_timeout_stop(self):
        control = self.apply_hard_limits(
            [self.safe_steering_angle, self.safe_deceleration]
        )
        self.publish_control(control, target_speed=0.0, enable=True)

        if self.latest_state is not None:
            try:
                current_state = self.state_to_array(self.latest_state)
            except ValueError:
                current_state = np.zeros(8, dtype=float)
        else:
            current_state = np.zeros(8, dtype=float)

        reference = np.array(
            [[current_state[0], current_state[1], current_state[2], 0.0]],
            dtype=float,
        )
        self.publish_status(
            current_state,
            reference,
            "STATE_TIMEOUT",
            0.0,
            True,
        )

    def control_timer_callback(self, _event):
        now = rospy.Time.now()
        with self.lock:
            if not self.state_is_fresh(now):
                self.publish_state_timeout_stop()
                rospy.logwarn_throttle(
                    1.0,
                    "Vehicle state missing or older than %.3f s; "
                    "publishing safe deceleration",
                    self.state_timeout_s,
                )
                return

            if not self.latest_state.valid:
                self.publish_state_timeout_stop()
                rospy.logwarn_throttle(
                    1.0,
                    "VehicleState.valid is false; publishing safe deceleration",
                )
                return

            try:
                current_state = self.state_to_array(self.latest_state)
                reference = self.build_reference(current_state)
            except Exception as exception:
                self.publish_state_timeout_stop()
                rospy.logerr_throttle(
                    1.0,
                    "Invalid MPC state/reference: %s",
                    exception,
                )
                return

            control, solver_status, solve_time_ms, fallback_active = (
                self.solve_or_fallback(current_state, reference, now)
            )
            control = self.apply_hard_limits(control)

            if fallback_active:
                control[1] = min(control[1], 0.0)

            self.publish_control(
                control,
                target_speed=reference[0, 3],
                enable=True,
            )
            self.publish_status(
                current_state,
                reference,
                solver_status,
                solve_time_ms,
                fallback_active,
            )


def main():
    rospy.init_node("learned_mpc_node")
    LearnedMPCNode()
    rospy.spin()


if __name__ == "__main__":
    main()
