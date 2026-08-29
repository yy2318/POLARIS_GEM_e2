# gem_learned_mpc

Complete ROS Noetic package for GEM e2 system identification and learned nonlinear MPC.

## Important interface assumptions
- State input: `nav_msgs/Odometry`.
- Default odometry: `/gem/base_footprint/odom`.
- Command adapter output: `ackermann_msgs/AckermannDriveStamped`. Change `command_adapter.py` if the simulator uses PACMod or another topic.
- State: `[X,Y,psi,vx,vy,r,delta,a]`. Control: `[delta_cmd,a_cmd]`.
- The fallback dynamics only validates integration. Formal experiments must pass a trained `model.json`.

## Install
```bash
cd /home/ll/gem_ws/src
unzip ~/Downloads/gem_learned_mpc.zip
cp /path/to/wps.csv /home/ll/gem_ws/src/gem_learned_mpc/waypoints/wps.csv
sudo apt install ros-noetic-ackermann-msgs python3-pip
python3 -m pip install --user 'setuptools<71' 'importlib-metadata>=6.8,<9' casadi numpy pyyaml matplotlib
cd /home/ll/gem_ws
source /opt/ros/noetic/setup.bash
rosdep install --from-paths src --ignore-src -r -y
rm -rf build/gem_learned_mpc
catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3
source devel/setup.bash
```

The CMake order is deliberately: `find_package`, `catkin_python_setup`, `add_message_files`, `generate_messages`, `catkin_package`.

## Test
```bash
python3 /home/ll/gem_ws/src/gem_learned_mpc/test/test_core.py
rosmsg package gem_learned_mpc
```

## Inspect actual simulator interfaces
```bash
rostopic list | grep -E 'odom|ackermann|pacmod|steer|cmd'
rostopic type /gem/base_footprint/odom
rostopic echo -n 1 /gem/base_footprint/odom
```

## Run
```bash
roslaunch gem_learned_mpc full_demo.launch \
 waypoint_file:=/home/ll/gem_ws/src/gem_learned_mpc/waypoints/wps.csv \
 odom_topic:=/gem/base_footprint/odom \
 model_json:=/home/ll/gem_ws/models/mlp/model.json
```

## Train
Prepare NPZ with `x` shape Nx7 and `y` shape Nx5, then:
```bash
python3 scripts/train_dynamics.py data/processed/train.npz --epochs 300 --output-dir models/mlp
python3 scripts/validate_one_step.py data/processed/test.npz models/mlp/model.json
python3 scripts/validate_rollout.py data/processed/test_run.npz models/mlp/model.json --horizons 10 20
```

## Safety
Start at 2 m/s. Validate steering sign, coordinate frame, command topic, model normalization and 10/20-step rollout before raising speed. The configured maximum is 5.5556 m/s.
