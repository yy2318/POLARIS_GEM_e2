# Polaris GEM e2 系统辨识实验：一步一步实际操作指南

本指南对应“系统辨识实验”方案：20 Hz采样、4步输入历史、静止偏置、纵向阶跃、制动、横向阶跃、正弦和PRBS实验；随后完成bag提取、20 Hz对齐、按完整run拆分、Ridge/MLP训练、RMSE图和预测对比图。实验参数与原方案保持一致。 

## 0. 解压和复制

```bash
cd /home/yy/gem_ws/src
unzip /下载路径/gem_system_identification_package.zip
cp -a gem_system_identification_package/. gem_learned_mpc/
chmod +x gem_learned_mpc/scripts/*.py
```

## 1. 填写实际接口

启动GEM仿真器后运行：

```bash
rostopic echo -n 1 /gazebo/model_states/name
rostopic find ackermann_msgs/AckermannDrive
rostopic find ackermann_msgs/AckermannDriveStamped
rostopic info /ackermann_cmd
rostopic echo -n 1 /joint_states/name
```

编辑`config/system_id.yaml`，替换模型名、命令话题、消息类型和转向关节名。

## 2. 编译

```bash
cd /home/yy/gem_ws
source /opt/ros/noetic/setup.bash
rm -rf build devel
catkin_make
source devel/setup.bash
rosmsg show gem_learned_mpc/VehicleState
```

## 3. 启动仿真器

```bash
find "$(rospack find gem_launch)" -type f -name '*.launch' | sort
roslaunch gem_launch ACTUAL_FILE.launch vehicle_name:=e2
```

只传递实际launch文件支持的参数。

## 4. 验证状态适配器

```bash
rosrun gem_learned_mpc state_adapter.py _model_name:=ACTUAL_MODEL_NAME
rostopic hz /system_id/vehicle_state
rostopic echo -n 1 /system_id/vehicle_state
```

必须看到`valid: True`。

## 5. 建立实验目录

```bash
mkdir -p /home/yy/gem_ws/bags /home/yy/gem_ws/data /home/yy/gem_ws/results
```

## 6. 每次实验先开始录包

```bash
RUN=prbs_$(date +%Y%m%d_%H%M%S)
rosbag record --lz4 --split --size=2048 \
  -O /home/yy/gem_ws/bags/$RUN \
  /clock /gazebo/model_states /joint_states \
  ACTUAL_COMMAND_TOPIC /system_id/vehicle_state
```

## 7. 执行激励实验

PRBS初次低速测试：

```bash
rosrun gem_learned_mpc excitation_node.py \
  _command_topic:=ACTUAL_COMMAND_TOPIC \
  _command_type:=ackermann_msgs/AckermannDrive \
  _mode:=prbs _duration:=300 _speed_max:=2.0 _steering_max:=0.15
```

其他实验：

```bash
# 纵向/混合阶跃
rosrun gem_learned_mpc excitation_node.py _command_topic:=ACTUAL_COMMAND_TOPIC _mode:=step _duration:=240 _speed_max:=5.0
# 制动重复
rosrun gem_learned_mpc excitation_node.py _command_topic:=ACTUAL_COMMAND_TOPIC _mode:=brake _duration:=120 _speed_max:=3.0
# 正弦转向
rosrun gem_learned_mpc excitation_node.py _command_topic:=ACTUAL_COMMAND_TOPIC _mode:=sine _duration:=120 _speed_max:=3.0 _steering_max:=0.2 _frequency:=0.08
```

每个实验单独录包。开始时速度上限用1至2 m/s，接口验证后再按方案增大。

## 8. 检查bag

```bash
rosbag info /home/yy/gem_ws/bags/ACTUAL_FILE.bag
```

必须包含命令与`/system_id/vehicle_state`。

## 9. 从bag提取原始数组

```bash
python3 /home/yy/gem_ws/src/gem_learned_mpc/scripts/bag_to_dataset.py \
  /home/yy/gem_ws/bags/*.bag \
  --command-topic ACTUAL_COMMAND_TOPIC \
  --out /home/yy/gem_ws/data/raw_system_id.npz
```

## 10. 预处理为20 Hz监督数据

```bash
python3 /home/yy/gem_ws/src/gem_learned_mpc/scripts/preprocess_dataset.py \
  /home/yy/gem_ws/data/raw_system_id.npz \
  --dt 0.05 --history 4 \
  --out /home/yy/gem_ws/data/ready_system_id.npz
```

该脚本执行线性状态插值、控制零阶保持、Yaw展开、reset分段和按完整run划分。

## 11. 训练Ridge基线

```bash
python3 /home/yy/gem_ws/src/gem_learned_mpc/scripts/train_dynamics.py \
  /home/yy/gem_ws/data/ready_system_id.npz \
  --model ridge \
  --output-dir /home/yy/gem_ws/models/ridge
```

## 12. 训练MLP和导出TorchScript

```bash
python3 /home/yy/gem_ws/src/gem_learned_mpc/scripts/train_dynamics.py \
  /home/yy/gem_ws/data/ready_system_id.npz \
  --model mlp --epochs 300 \
  --output-dir /home/yy/gem_ws/models/mlp
```

输出包括`dynamics_model.pt`、`dynamics_model.norm.npz`、`metrics.json`和验证预测。

## 13. 生成作业要求的图

```bash
python3 /home/yy/gem_ws/src/gem_learned_mpc/scripts/validate_dynamics.py \
  /home/yy/gem_ws/models/mlp/validation_predictions.npz \
  --out-dir /home/yy/gem_ws/results/model_validation
```

输出：

- `input_output_prediction.png`
- `rmse.png`
- `rmse.json`

## 14. 验收检查

```bash
cat /home/yy/gem_ws/models/mlp/metrics.json
ls -lh /home/yy/gem_ws/results/model_validation
```

必须报告`vx`、`vy`、`yaw_rate`和`steering_angle`的RMSE。原方案还要求10步和20步rollout位置RMSE；本包的一步模型训练和图表管线可直接运行，长时域rollout应在MPC接入前基于保存的连续run补充评估。

## 15. 故障处理

- `valid: False`：模型名错误，检查`/gazebo/model_states/name`。
- 车辆不动：命令话题或消息类型错误，检查`rostopic info`的Subscriber。
- steering始终为0：填写真实转向关节名，或使用命令转角备用值。
- bag中无命令：录包命令中的`ACTUAL_COMMAND_TOPIC`未替换。
- 样本数为0：状态和命令时间区间不重叠，或发生频繁reset。
- MLP导出失败：确认Python 3环境中可导入PyTorch。
