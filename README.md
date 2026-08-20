# Polaris GEM e2 System Identification and MPC Operations

**System Identification, Learned-Dynamics MPC, and CSV Path Tracking**

> Minimum Hardware requirements:    
> Recommended Software Environment: Ubuntu 20.04 | ROS Noetic | Gazebo 11 | Python 3.8.10

# Contents

- 1. Overall system architecture
- 2. wps.csv inspection results
- 3. Project directory design
- 4. Software dependencies
- 5. Launching the GEM simulator
- 6. Verifying the live ROS interfaces
- 7. Step 1: System identification
- 8. Step 2: Learned-model MPC path tracking
- 9. Step 3: Read wps.csv and publish the path
- 10. Initial-pose and coordinate-frame alignment
- 11. Complete execution procedure
- 12. Acceptance method
- 13. Recommended debugging sequence
- 14. Key engineering risks
- 15. Final recommended configuration

# 1 Overall System Architecture

The system forms a closed loop from simulation-data collection through dynamics identification, MPC path tracking, and CSV path execution. The learned model predicts the vehicle dynamics used inside MPC, while known motion equations propagate the global pose.

```text
Step 1: System Identification
Excitation Node -> Ackermann Command -> GEM e2 Gazebo
                                           |
                                           v
                         ModelStates / JointState -> State Adapter
                                           |
                                           v
                                     rosbag Recorder
                                           |
                                           v
                              Dataset Processing and Training
                                           |
                                           v
                                Learned Dynamics Model
                                           |
Step 2: MPC Path Tracking                 v
wps.csv Loader -> nav_msgs/Path -> MPC Controller -> Ackermann Command
                                           |
                                           v
                                      GEM e2 Gazebo
```

> Safety and acceptance  
> Implement the 5.5556 m/s limit both inside the optimizer and immediately before publishing the command. Measure tracking accuracy as signed distance to the path centerline, not merely distance to the nearest waypoint.

# 2 wps.csv Inspection Results

| Item | Result |
| --- | --- |
| Rows | 3,822 |
| Columns | 5 |
| Column 0 range | -150.595 to 150.216 |
| Column 1 range | -2.593 to 199.323 |
| Column 2 range | -3.140 to 3.141 |
| Column 3 range | -7.679 to 8.216 |
| Column 4 range | 0.000762 to 8.226 |
| XY path length | Approximately 834.76 m |
| Start | Approximately (-0.014, -2.000) |
| End | Approximately (2.968, -1.770) |
| Duplicate XY rows | 0 |

# 2.1 Recommended column interpretation

> column 0: x  
> column 1: y  
> column 2: yaw  
> column 3: signed longitudinal speed or vx  
> column 4: speed magnitude

Because the CSV has no header, this interpretation must remain configurable. The first two columns are authoritative for geometry. Treat column 2 as yaw only after checking continuity. Do not publish columns 3 or 4 directly as commands. Any speed derived from the file must be clipped to 5.5556 m/s.

- Remove non-finite rows.
- Remove near-identical consecutive points.
- Resample by cumulative arc length before reference generation.
- Recompute yaw and curvature from the resampled geometry.
- Use curvature and tracking error to reduce speed.
# 3 Project Directory Design

```text
gem_learned_mpc/
├── CMakeLists.txt
├── package.xml
├── config/
│   ├── topics.yaml
│   ├── data_collection.yaml
│   ├── model.yaml
│   └── mpc.yaml
├── launch/
│   ├── collect_system_id.launch
│   ├── path_tracking.launch
│   └── all_in_one.launch
├── msg/
│   ├── VehicleState.msg
│   └── MPCStatus.msg
├── scripts/
│   ├── state_adapter.py
│   ├── excitation_node.py
│   ├── bag_to_dataset.py
│   ├── preprocess_dataset.py
│   ├── train_dynamics.py
│   ├── validate_dynamics.py
│   ├── waypoint_path_node.py
│   ├── learned_model.py
│   └── mpc_controller_node.py
├── models/
├── data/
└── paths/wps.csv
```

Create the ROS package

```bash
cd ~/gem_ws/src
catkin_create_pkg gem_learned_mpc \
  rospy std_msgs geometry_msgs nav_msgs sensor_msgs \
  gazebo_msgs ackermann_msgs tf message_generation
cd gem_learned_mpc
mkdir -p config launch msg scripts models data paths
cp /path/to/wps.csv paths/wps.csv
```

# 4 Software Dependencies

> **Dependency correction:** ROS Noetic does not provide an APT package named `ros-noetic-tf-transformations`. Install `ros-noetic-tf`; the Python imports `tf.transformations.euler_from_quaternion` and `tf.transformations.quaternion_from_euler` remain valid.

```bash
sudo apt update
sudo apt install -y \
  ros-noetic-ackermann-msgs \
  ros-noetic-gazebo-ros-pkgs \
  ros-noetic-tf \
  ros-noetic-tf2-ros \
  ros-noetic-tf2-geometry-msgs \
  python3-pip
python3 -m pip install --user \
  numpy pandas scipy scikit-learn matplotlib \
  pyyaml joblib torch casadi
python3 -c "import numpy, pandas, scipy, torch, casadi"
```

> Python environment  
> Use the Python 3 environment associated with ROS Noetic. Avoid mixing system and virtual-environment packages without documenting the active interpreter.

# 5 Launching the GEM Simulator

Simple-track world

```bash
cd ~/gem_ws
source devel/setup.bash
roslaunch gem_gazebo gem_gazebo_rviz.launch velodyne_points:="false"
```

Highbay validation world

```bash
roslaunch gem_gazebo gem_gazebo_rviz.launch \
  world_name:="highbay_track.world" \
  x:=-5.5 y:=-21 velodyne_points:="false"
```

Verify simulation time

```bash
rosparam get /use_sim_time
rostopic hz /clock
```

# 6 Verifying the Live ROS Interfaces

```bash
rostopic list | sort
rostopic list | grep -Ei "ackermann|cmd|odom|model_states|joint_states"
rostopic find ackermann_msgs/AckermannDrive
rostopic find ackermann_msgs/AckermannDriveStamped
rostopic info /gazebo/model_states
rostopic echo -n 1 /gazebo/model_states
```

Print the Gazebo model names

```python
python3 - <<'PY'
import rospy
from gazebo_msgs.msg import ModelStates
rospy.init_node("print_gazebo_models", anonymous=True)
msg = rospy.wait_for_message("/gazebo/model_states", ModelStates)
print(msg.name)
PY
```

The examples assume the command topic is /gem/ackermann_cmd, the type is AckermannDriveStamped, and the Gazebo model name is gem. If the live system differs, change the YAML and adapter only, not the training or MPC core.

# 7 Step 1: System Identification

# 7.1 State, input, and learned output

```text
State x = [X, Y, yaw, vx, vy, yaw_rate, steering_angle]
Input u = [speed_cmd, steering_cmd]
Learned output = [vx_next, vy_next, yaw_rate_next, steering_next]
```

Use known equations to update the global pose:

```text
X_next   = X + dt * (vx*cos(yaw) - vy*sin(yaw))
Y_next   = Y + dt * (vx*sin(yaw) + vy*cos(yaw))
yaw_next = yaw + dt*yaw_rate
```

# 7.2 Unified VehicleState message

msg/VehicleState.msg

```text
std_msgs/Header header
float64 x
float64 y
float64 yaw
float64 vx
float64 vy
float64 yaw_rate
float64 steering_angle
bool valid
```

CMakeLists.txt additions

```bash
find_package(catkin REQUIRED COMPONENTS
  rospy std_msgs geometry_msgs nav_msgs sensor_msgs
  gazebo_msgs ackermann_msgs message_generation)
add_message_files(FILES VehicleState.msg MPCStatus.msg)
generate_messages(DEPENDENCIES std_msgs)
catkin_package(CATKIN_DEPENDS message_runtime)
```

# 7.3 State Adapter

scripts/state_adapter.py

```python
#!/usr/bin/env python3
import math, rospy
from gazebo_msgs.msg import ModelStates
from sensor_msgs.msg import JointState
from tf.transformations import euler_from_quaternion
from gem_learned_mpc.msg import VehicleState
class StateAdapter:
    def __init__(self):
        self.model_name=rospy.get_param("~model_name","gem")
        self.steer_names=rospy.get_param("~steering_joint_names",[])
        self.steering=float("nan")
        self.pub=rospy.Publisher("/system_id/vehicle_state",VehicleState,queue_size=20)
        rospy.Subscriber("/gazebo/model_states",ModelStates,self.on_state,queue_size=5)
        rospy.Subscriber("/joint_states",JointState,self.on_joints,queue_size=20)
    def on_joints(self,msg):
        v=[msg.position[msg.name.index(n)] for n in self.steer_names if n in msg.name]
        if v:self.steering=sum(v)/len(v)
    def on_state(self,msg):
        out=VehicleState();out.header.stamp=rospy.Time.now();out.header.frame_id="world"
        if self.model_name not in msg.name:
            out.valid=False;self.pub.publish(out);return
        i=msg.name.index(self.model_name);p=msg.pose[i];t=msg.twist[i];q=p.orientation
        _,_,yaw=euler_from_quaternion([q.x,q.y,q.z,q.w]);c=math.cos(yaw);s=math.sin(yaw)
        out.x=p.position.x;out.y=p.position.y;out.yaw=yaw
        out.vx=c*t.linear.x+s*t.linear.y;out.vy=-s*t.linear.x+c*t.linear.y
        out.yaw_rate=t.angular.z;out.steering_angle=self.steering;out.valid=True
        self.pub.publish(out)
if __name__=="__main__":
    rospy.init_node("state_adapter");StateAdapter();rospy.spin()
```

config/topics.yaml

> model_name: "gem"  
> model_states_topic: "/gazebo/model_states"  
> output_topic: "/system_id/vehicle_state"  
> steering_joint_names:  
>   - "left_steering_joint"  
>   - "right_steering_joint"

# 7.4 Sampling and excitation

```yaml
sample_time: 0.05
sample_rate: 20.0
history_length: 4
```

| Experiment | Configuration | Purpose |
| --- | --- | --- |
| Longitudinal | Steering 0; speed levels 0.5, 1.5, 2.5, 4.0, 5.0 m/s | Speed dynamics |
| Lateral | Speeds 1.5, 3.0, 4.5 m/s; sine, step, PRBS steering | Steering and yaw dynamics |
| Combined | Speed and steering vary together | Coupled dynamics |
| Normal tracking | Pure Pursuit or Stanley output | Operating-distribution data |

# 7.5 rosbag recording

```bash
mkdir -p ~/gem_sysid_bags
RUN=prbs_$(date +%Y%m%d_%H%M%S)
rosbag record --lz4 --split --size=2048 \
  -O ~/gem_sysid_bags/$RUN \
  /clock /gazebo/model_states /joint_states \
  /gem/ackermann_cmd /system_id/vehicle_state
rosbag info ~/gem_sysid_bags/<run>.bag
rosbag check ~/gem_sysid_bags/<run>.bag
```

# 7.6 Alignment, model structure, and training

- Resample to a fixed 20 Hz grid.
- Linearly interpolate continuous states.
- Apply zero-order hold to commands.
- Unwrap yaw before interpolation.
- Split sequences at every Gazebo reset.
- Partition complete runs into 70% training, 15% validation, and 15% test sets.
```text
feature = [vx, vy, yaw_rate, steering,
           u(k-3), u(k-2), u(k-1), u(k)]
target  = [vx(k+1), vy(k+1), yaw_rate(k+1), steering(k+1)]
```

Recommended residual MLP

```python
model = torch.nn.Sequential(
    torch.nn.Linear(input_dim,64), torch.nn.Tanh(),
    torch.nn.Linear(64,64), torch.nn.Tanh(),
    torch.nn.Linear(64,4))
```

Train first with one-step loss, then add a 10-to-20-step rollout loss. Save the TorchScript model, normalization statistics, and metadata together.

> models/dynamics_model.pt  
> models/dynamics_model.norm.npz  
> models/dynamics_model.json

# 8 Step 2: Learned-Model MPC Path Tracking

# 8.1 Constraints and tracking thresholds

| Constraint | Value |
| --- | --- |
| Speed | 0 to 5.5556 m/s |
| Acceleration | -2.0 to 1.5 m/s^2 |
| Steering angle | -0.45 to 0.45 rad |
| Steering rate | -0.5 to 0.5 rad/s |
| Nominal CTE target | Below 0.5 m |
| Warning CTE | 0.8 m |
| Failure threshold | Above 1.0 m |

# 8.2 MPC configuration

config/mpc.yaml

```yaml
mpc:
  sample_time: 0.05
  prediction_horizon: 20
  control_horizon: 10
  maximum_solver_time: 0.040
  maximum_iterations: 40
  warm_start: true
model:
  model_path: "models/dynamics_model.pt"
  normalization_path: "models/dynamics_model.norm.npz"
  metadata_path: "models/dynamics_model.json"
  history_length: 4
vehicle:
  maximum_speed_mps: 5.5555556
  minimum_speed_mps: 0.0
  maximum_acceleration: 1.5
  maximum_deceleration: 2.0
  maximum_steering_angle: 0.45
  maximum_steering_rate: 0.5
tracking:
  target_cross_track_error: 0.5
  warning_cross_track_error: 0.8
  maximum_cross_track_error: 1.0
safety:
  state_timeout: 0.20
  path_timeout: 1.00
  stop_after_solver_failures: 3
  stop_when_cross_track_exceeded: true
  fallback_model: "kinematic"
```

# 8.3 Cross-Track Error and Reference Generation

```python
def signed_cross_track_error(position,p0,p1):
    p=np.asarray(position,float);a=np.asarray(p0,float);b=np.asarray(p1,float)
    seg=b-a;l2=np.dot(seg,seg)
    if l2<1e-12:return 0.0,a
    t=np.clip(np.dot(p-a,seg)/l2,0.0,1.0)
    proj=a+t*seg;e=p-proj
    cte=(seg[0]*e[1]-seg[1]*e[0])/np.sqrt(l2)
    return cte,proj
```

At each cycle, find the nearest path segment, project the vehicle onto the segment, and sample the next N+1 references by arc length. Bound reference speed by the CSV profile, the 20 km/h limit, curvature, and current cross-track error.

```text
v_curve = sqrt(a_lat_max / (abs(curvature) + 1e-3))
v_ref = min(5.5556, v_csv, v_curve)
if abs(cte) >= 0.8: v_ref = min(v_ref, 1.5)
elif abs(cte) >= 0.5: v_ref = min(v_ref, 3.0)
if abs(cte) > 1.0: publish_zero_speed()
```

# 8.4 Cost and history handling

> J = sum(e.T @ Q @ e + u.T @ R @ u + du.T @ Rd @ du)  
>     + terminal_error.T @ Qf @ terminal_error

Wrap yaw error with atan2(sin(error), cos(error)). Every optimization candidate must use its own copy of the command-history window, so candidate evaluations never contaminate one another.

# 8.5 Solver Strategy

| Phase | Solver | Starting settings |
| --- | --- | --- |
| Commissioning | SciPy SLSQP | Np=10, Nc=5, speed limit 2 m/s |
| Production NMPC | CasADi + IPOPT | Reconstruct the Tanh MLP symbolically |
| Production LTV MPC | Online linearization + OSQP | Use model Jacobians and QP warm start |

```python
def casadi_mlp(z,weights,ca):
    W1,b1,W2,b2,W3,b3=weights
    h1=ca.tanh(ca.DM(W1)@z+ca.DM(b1))
    h2=ca.tanh(ca.DM(W2)@h1+ca.DM(b2))
    return ca.DM(W3)@h2+ca.DM(b3)
```

# 9 Step 3: Read wps.csv and Publish the Path

# 9.1 Processing pipeline

> Read CSV -> extract x/y and optional yaw/speed  
> -> remove non-finite and near-duplicate points  
> -> compute cumulative arc length  
> -> resample every 0.25 m  
> -> recompute yaw and curvature  
> -> curvature-limit and clip speed  
> -> publish nav_msgs/Path and reference profile

# 9.2 Waypoint path-node core

scripts/waypoint_path_node.py core

```python
data=pd.read_csv(csv_path,header=None).to_numpy(float)
x_raw,y_raw=data[:,0],data[:,1]
speed_raw=np.abs(data[:,4]) if data.shape[1]>=5 else np.full(len(data),max_speed)
# Remove points closer than 0.02 m.
dist=np.hypot(np.diff(x_raw),np.diff(y_raw));keep=np.r_[True,dist>=0.02]
x_raw,y_raw,speed_raw=x_raw[keep],y_raw[keep],speed_raw[keep]
seg=np.hypot(np.diff(x_raw),np.diff(y_raw));s=np.r_[0.0,np.cumsum(seg)]
sq=np.arange(0.0,s[-1],0.25);sq=np.r_[sq,s[-1]] if sq[-1]<s[-1] else sq
x=np.interp(sq,s,x_raw);y=np.interp(sq,s,y_raw);v_csv=np.interp(sq,s,speed_raw)
dx=np.gradient(x,sq);dy=np.gradient(y,sq);yaw=np.unwrap(np.arctan2(dy,dx))
d2x=np.gradient(dx,sq);d2y=np.gradient(dy,sq)
kappa=(dx*d2y-dy*d2x)/np.maximum((dx*dx+dy*dy)**1.5,1e-6)
v_curve=np.sqrt(2.0/(np.abs(kappa)+1e-3))
speed=np.clip(np.minimum.reduce([v_csv,v_curve,np.full(len(x),5.5556)]),0.0,5.5556)
```

Publish PoseStamped entries with the recomputed yaw quaternion to /mpc/reference_path. Publish flattened [yaw, curvature, speed] triples to /mpc/reference_profile.

# 9.3 Path-Tracking Launch File

launch/path_tracking.launch

```xml
<launch>
  <arg name="csv_path" default="$(find gem_learned_mpc)/paths/wps.csv"/>
  <arg name="command_topic" default="/gem/ackermann_cmd"/>
  <rosparam command="load" file="$(find gem_learned_mpc)/config/mpc.yaml"/>
  <node pkg="gem_learned_mpc" type="state_adapter.py"
        name="state_adapter" output="screen">
    <param name="model_name" value="gem"/>
  </node>
  <node pkg="gem_learned_mpc" type="waypoint_path_node.py"
        name="waypoint_path" output="screen" required="true">
    <param name="csv_path" value="$(arg csv_path)"/>
    <param name="frame_id" value="world"/>
    <param name="resample_distance" value="0.25"/>
    <param name="maximum_speed" value="5.5555556"/>
    <param name="maximum_lateral_acceleration" value="2.0"/>
  </node>
  <node pkg="gem_learned_mpc" type="mpc_controller_node.py"
        name="mpc_controller" output="screen" required="true">
    <param name="command_topic" value="$(arg command_topic)"/>
    <param name="model_path" value="$(find gem_learned_mpc)/models/dynamics_model.pt"/>
    <param name="normalization_path" value="$(find gem_learned_mpc)/models/dynamics_model.norm.npz"/>
  </node>
</launch>
```

# 10 Initial Pose and Coordinate-Frame Alignment

The CSV starts near x=-0.014 m, y=-2.000 m, yaw approximately zero. If supported by the simulator launch file, initialize the vehicle near that pose.

```bash
roslaunch gem_gazebo gem_gazebo_rviz.launch \
  x:=-0.014 y:=-2.0 yaw:=0.0 velodyne_points:="false"
```

If yaw is not a supported launch argument, set the model state through Gazebo or apply a rigid transform to the path.

```python
def transform_path(x,y,rotation,tx,ty):
    c=np.cos(rotation);s=np.sin(rotation)
    return c*x-s*y+tx, s*x+c*y+ty
```

> Do not tune around a frame error  
> If the path and vehicle do not overlap in RViz, correct the transform first. Changing MPC weights cannot repair an incorrect coordinate frame.

# 11 Complete Execution Procedure

# 11.1 Collect and train

```bash
# Terminal 1
source ~/gem_ws/devel/setup.bash
roslaunch gem_gazebo gem_gazebo_rviz.launch velodyne_points:="false"
# Terminal 2
rosrun gem_learned_mpc state_adapter.py _model_name:=gem
# Terminal 3
rosbag record --lz4 -O ~/gem_sysid_bags/run_001 \
  /clock /gazebo/model_states /joint_states \
  /gem/ackermann_cmd /system_id/vehicle_state
# Terminal 4
rosrun gem_learned_mpc excitation_node.py \
  _mode:=prbs _duration:=300 _speed_max:=5.0
# Offline
python3 scripts/bag_to_dataset.py ~/gem_sysid_bags/run_001.bag --out data/run_001_raw.npz
python3 scripts/preprocess_dataset.py data/run_001_raw.npz --dt 0.05 --history 4 --out data/run_001_ready.npz
python3 scripts/train_dynamics.py data/train_*_ready.npz --output models/dynamics_model.pt
```

# 11.2 Execute CSV path tracking

```bash
# Terminal 1
source ~/gem_ws/devel/setup.bash
roslaunch gem_gazebo gem_gazebo_rviz.launch \
  x:=-0.014 y:=-2.0 velodyne_points:="false"
# Terminal 2
source ~/gem_ws/devel/setup.bash
roslaunch gem_learned_mpc path_tracking.launch \
  csv_path:=$(rospack find gem_learned_mpc)/paths/wps.csv
# Monitor
rostopic hz /system_id/vehicle_state
rostopic hz /gem/ackermann_cmd
rostopic echo /mpc/cross_track_error
rostopic echo /mpc/status
```

# 12 Acceptance Method

# 12.1 Dynamics-model acceptance

| Horizon | Required report |
| --- | --- |
| One step | RMSE of vx, vy, yaw_rate, steering_angle |
| 10-step rollout | Position, yaw, and speed error |
| 20-step rollout | Position, yaw, speed, and divergence behavior |

Initial engineering targets: 1.0 s position RMSE below 0.20 m, yaw RMSE below 3 degrees, speed RMSE below 0.20 m/s, and no unstable rollout divergence.

# 12.2 Path-tracking acceptance

- Maximum absolute CTE
- 95th-percentile absolute CTE
- Mean absolute CTE and CTE RMSE
- Time above 1.0 m
- Maximum vehicle and command speed
- Mean and maximum solver time
- Solver failure count
| Mandatory condition | Requirement |
| --- | --- |
| Maximum speed | No greater than 5.5556 m/s |
| Maximum absolute CTE | No greater than 1.0 m |
| Preferred margin | 95% of absolute CTE at or below 0.5 m |
| Controller rate | Continuous operation at 20 Hz |
| Validity | No invalid state or unbounded command |

# 13 Recommended Debugging Sequence

| Stage | Model | Maximum speed | Primary check |
| --- | --- | --- | --- |
| 1 | Kinematic | 1.0 m/s | Frames, path direction, steering sign |
| 2 | Kinematic | 2.0 m/s | MPC cost, constraints, CTE |
| 3 | Learned | 2.0 m/s | Compare learned and kinematic predictions |
| 4 | Learned | 3.5 m/s | Enable curvature speed limiting |
| 5 | Learned | 5.5556 m/s | Formal 1 m CTE acceptance |

Save the YAML configuration, rosbag, error CSV, source commit, and model version for every stage.

# 14 Key Engineering Risks

| Risk | Consequence | Mitigation |
| --- | --- | --- |
| CSV and Gazebo frame mismatch | Immediate path departure | Verify overlay in RViz first |
| Direct use of CSV speed | May exceed 20 km/h | Dual optimizer and publisher limits |
| Uneven path spacing | Unstable nearest-point search | Resample every 0.25 m |
| Command/state misalignment | False learned delay | Use simulation time and 20 Hz alignment |
| Training pair crosses reset | Artificial huge velocity | Segment at resets and pose jumps |
| Command steering used as actual steering | Lower yaw prediction accuracy | Prefer /joint_states |
| Random sample split | Overly optimistic test result | Split complete runs |
| One-step-only loss | Multi-step MPC divergence | Add rollout loss |
| SLSQP too slow | Missed 20 Hz deadline | Use CasADi or LTV QP |
| High speed near 1 m error | Tracking requirement violation | Slow at 0.5/0.8 m; stop at 1 m |

# 15 Final Recommended Configuration

```yaml
sample_time: 0.05
prediction_horizon: 20
control_horizon: 10
path_resample_distance: 0.25
maximum_speed: 5.5555556
initial_test_speed: 1.0
maximum_steering_angle: 0.45
maximum_steering_rate: 0.5
maximum_acceleration: 1.5
maximum_deceleration: 2.0
maximum_lateral_acceleration: 2.0
target_cross_track_error: 0.5
warning_cross_track_error: 0.8
maximum_cross_track_error: 1.0
learned_outputs:
  - vx_next
  - vy_next
  - yaw_rate_next
  - steering_angle_next
known_equations:
  - x_position_update
  - y_position_update
  - yaw_update
solver:
  prototype: scipy_slsqp
  production: casadi_ipopt_or_ltv_qp
```

> Final implementation principle  
> First verify the complete pipeline with the kinematic model and low speed. Introduce the learned model only after coordinates, topics, signs, limits, and reference generation are proven correct.
