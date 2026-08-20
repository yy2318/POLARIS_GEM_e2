#!/usr/bin/env python3
"""Extract GEM system-identification data from ROS1 bag files.

Preferred state source:
    /system_id/vehicle_state (gem_learned_mpc/VehicleState)

Automatic fallback:
    /gazebo/model_states (gazebo_msgs/ModelStates)

The fallback is necessary when the state adapter was not running during bag
recording. Gazebo world-frame velocities are rotated into the vehicle frame.
Steering angle is read from /joint_states when steering-joint names are given;
otherwise the most recent steering command is used as a clearly documented
proxy.
"""

import argparse
import json
import math
import sys
from pathlib import Path

import numpy as np

try:
    import rosbag
except ImportError as exc:
    raise SystemExit(
        "Run this script in the ROS Noetic Python environment: "
        "source /opt/ros/noetic/setup.bash"
    ) from exc

try:
    from tf.transformations import euler_from_quaternion
except ImportError as exc:
    raise SystemExit(
        "tf.transformations is unavailable. Install/source ros-noetic-tf."
    ) from exc


def message_stamp(message, bag_time):
    """Return message header time when valid; otherwise use bag-record time."""
    header = getattr(message, "header", None)
    stamp = getattr(header, "stamp", None)
    if stamp is not None and stamp.to_sec() > 0.0:
        return float(stamp.to_sec())
    return float(bag_time.to_sec())


def command_values(message):
    """Support AckermannDrive and AckermannDriveStamped."""
    drive = message.drive if hasattr(message, "drive") else message
    return float(drive.speed), float(drive.steering_angle)


def finite_row(values):
    return all(math.isfinite(float(value)) for value in values)


def discover_topics(bag):
    info = bag.get_type_and_topic_info()
    return set(info.topics.keys())


def model_state_row(message, bag_time, run_id, model_name, steering_angle):
    """Build normalized state row from gazebo_msgs/ModelStates."""
    if model_name not in message.name:
        return None

    index = message.name.index(model_name)
    if index >= len(message.pose) or index >= len(message.twist):
        return None

    pose = message.pose[index]
    twist = message.twist[index]
    q = pose.orientation
    _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])

    cosine = math.cos(yaw)
    sine = math.sin(yaw)

    # gazebo_msgs/ModelStates reports twist in the world frame. Rotate to body.
    vx_body = cosine * twist.linear.x + sine * twist.linear.y
    vy_body = -sine * twist.linear.x + cosine * twist.linear.y

    row = [
        run_id,
        float(bag_time.to_sec()),
        float(pose.position.x),
        float(pose.position.y),
        float(yaw),
        float(vx_body),
        float(vy_body),
        float(twist.angular.z),
        float(steering_angle),
    ]
    return row if finite_row(row) else None


def joint_steering(message, steering_joint_names):
    values = []
    for name in steering_joint_names:
        if name in message.name:
            index = message.name.index(name)
            if index < len(message.position):
                value = float(message.position[index])
                if math.isfinite(value):
                    values.append(value)
    return sum(values) / len(values) if values else None


def parse_args():
    parser = argparse.ArgumentParser(
        description="Extract normalized GEM states and Ackermann commands."
    )
    parser.add_argument("bags", nargs="+", help="Input ROS1 bag files")
    parser.add_argument(
        "--state-topic",
        default="/system_id/vehicle_state",
        help="Preferred VehicleState topic",
    )
    parser.add_argument(
        "--gazebo-state-topic",
        default="/gazebo/model_states",
        help="Fallback gazebo_msgs/ModelStates topic",
    )
    parser.add_argument(
        "--joint-state-topic",
        default="/joint_states",
        help="sensor_msgs/JointState topic",
    )
    parser.add_argument(
        "--command-topic",
        required=True,
        help="Ackermann command topic, for example /ackermann_cmd",
    )
    parser.add_argument(
        "--model-name",
        default="gem_e2",
        help="Gazebo vehicle model name used by fallback extraction",
    )
    parser.add_argument(
        "--steering-joints",
        nargs="*",
        default=[],
        help="Optional steering joint names; command steering is fallback",
    )
    parser.add_argument(
        "--require-state-topic",
        action="store_true",
        help="Disable Gazebo fallback and fail if VehicleState is absent",
    )
    parser.add_argument("--out", required=True, help="Output NPZ path")
    return parser.parse_args()


def main():
    args = parse_args()
    states = []
    commands = []
    run_reports = []

    for run_id, bag_path in enumerate(args.bags):
        bag_path = str(Path(bag_path).expanduser())
        if not Path(bag_path).is_file():
            print("WARNING: bag does not exist: {}".format(bag_path), file=sys.stderr)
            continue

        bag_states = 0
        bag_commands = 0
        missing_model_messages = 0
        latest_command_steering = 0.0
        latest_joint_steering = None

        with rosbag.Bag(bag_path) as bag:
            available = discover_topics(bag)
            has_precomputed_state = args.state_topic in available
            has_gazebo_state = args.gazebo_state_topic in available
            has_command = args.command_topic in available
            has_joint = args.joint_state_topic in available

            if not has_command:
                print(
                    "WARNING: {} lacks command topic {}".format(
                        bag_path, args.command_topic
                    ),
                    file=sys.stderr,
                )

            if args.require_state_topic and not has_precomputed_state:
                print(
                    "WARNING: {} lacks required state topic {}".format(
                        bag_path, args.state_topic
                    ),
                    file=sys.stderr,
                )
                continue

            if not has_precomputed_state and not has_gazebo_state:
                print(
                    "WARNING: {} has neither {} nor {}".format(
                        bag_path, args.state_topic, args.gazebo_state_topic
                    ),
                    file=sys.stderr,
                )
                continue

            selected_topics = [args.command_topic]
            if has_precomputed_state:
                selected_topics.append(args.state_topic)
                state_source = "vehicle_state"
            else:
                selected_topics.append(args.gazebo_state_topic)
                state_source = "gazebo_model_states"
            if has_joint:
                selected_topics.append(args.joint_state_topic)

            for topic, message, bag_time in bag.read_messages(topics=selected_topics):
                if topic == args.command_topic:
                    speed, steering = command_values(message)
                    latest_command_steering = steering
                    row = [run_id, message_stamp(message, bag_time), speed, steering]
                    if finite_row(row):
                        commands.append(row)
                        bag_commands += 1

                elif topic == args.joint_state_topic:
                    value = joint_steering(message, args.steering_joints)
                    if value is not None:
                        latest_joint_steering = value

                elif topic == args.state_topic:
                    if not getattr(message, "valid", False):
                        continue
                    row = [
                        run_id,
                        message_stamp(message, bag_time),
                        message.x,
                        message.y,
                        message.yaw,
                        message.vx,
                        message.vy,
                        message.yaw_rate,
                        message.steering_angle,
                    ]
                    if finite_row(row):
                        states.append(row)
                        bag_states += 1

                elif topic == args.gazebo_state_topic:
                    steering = (
                        latest_joint_steering
                        if latest_joint_steering is not None
                        else latest_command_steering
                    )
                    row = model_state_row(
                        message,
                        bag_time,
                        run_id,
                        args.model_name,
                        steering,
                    )
                    if row is None:
                        missing_model_messages += 1
                    else:
                        states.append(row)
                        bag_states += 1

        report = {
            "run": run_id,
            "bag": bag_path,
            "state_source": state_source,
            "state_samples": bag_states,
            "command_samples": bag_commands,
            "joint_feedback_used": bool(args.steering_joints and latest_joint_steering is not None),
            "missing_model_messages": missing_model_messages,
        }
        run_reports.append(report)
        print(
            "run={run} states={state_samples} commands={command_samples} "
            "source={state_source} bag={bag}".format(**report)
        )

    state_array = np.asarray(states, dtype=float)
    command_array = np.asarray(commands, dtype=float)

    if state_array.ndim != 2 or state_array.shape[0] < 2:
        raise RuntimeError(
            "Insufficient state samples. The bags do not contain the preferred "
            "VehicleState topic, and Gazebo fallback produced no states. Check "
            "--model-name against /gazebo/model_states/name."
        )
    if command_array.ndim != 2 or command_array.shape[0] < 2:
        raise RuntimeError(
            "Insufficient command samples. Check --command-topic against rosbag info."
        )

    output = Path(args.out).expanduser()
    output.parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(output, states=state_array, commands=command_array)

    metadata = {
        "state_columns": [
            "run", "time", "x", "y", "yaw", "vx", "vy",
            "yaw_rate", "steering_angle",
        ],
        "command_columns": [
            "run", "time", "speed_cmd", "steering_cmd",
        ],
        "model_name": args.model_name,
        "preferred_state_topic": args.state_topic,
        "gazebo_state_topic": args.gazebo_state_topic,
        "joint_state_topic": args.joint_state_topic,
        "command_topic": args.command_topic,
        "steering_joints": args.steering_joints,
        "steering_fallback": "latest command when joint feedback is unavailable",
        "runs": run_reports,
        "state_shape": list(state_array.shape),
        "command_shape": list(command_array.shape),
    }
    Path(str(output) + ".json").write_text(
        json.dumps(metadata, indent=2), encoding="utf-8"
    )

    print("saved:", output)
    print("states:", state_array.shape)
    print("commands:", command_array.shape)


if __name__ == "__main__":
    main()
