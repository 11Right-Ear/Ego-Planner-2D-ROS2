# Ackermann EGO-Planner RViz / Gazebo Simulation

基于 **Ego-Planner-2D-ROS2** 的 ROS 2 Humble 阿克曼小车仿真工程，提供两种运行模式：

1. **RViz 假定位模式**：不需要 Gazebo，车模沿 B 样条轨迹在 RViz 中运动。
2. **Gazebo 真实底盘模式**：使用 Gazebo Classic + `gazebo_ros2_control` + `ackermann_steering_controller`，由物理引擎驱动真实底盘。

两种模式共享同一套规划流程：

- RViz `2D Goal Pose` 设置目标
- EGO Planner 生成完整 A* 全局路线
- B 样条（B-spline）优化局部轨迹
- 阿克曼底盘跟踪 B 样条轨迹

> Gazebo 底盘接入参考了 AgileX 官方仓库：
> <https://github.com/agilexrobotics/ugv_gazebo_sim>
>
> 官方 Hunter 包是 ROS 1/Melodic 方案，本工程将其控制思想迁移为 ROS 2 Humble 原生方案：
> `gazebo_ros2_control` + `ackermann_steering_controller` + `joint_state_broadcaster`。
> 底盘尺寸采用 Hunter 参数，外观仍使用精确尺寸的简化几何。

---

## 1. 环境要求

- Ubuntu 22.04
- ROS 2 Humble
- Gazebo Classic 11
- colcon、RViz2、xacro、robot_state_publisher
- 以下 ROS 2 包：
  - `gazebo_ros`
  - `gazebo_ros2_control`
  - `controller_manager`
  - `joint_state_broadcaster`
  - `ackermann_steering_controller`

如果使用 Docker，请进入已配置好的 Humble 容器后，在工作区根目录执行命令。

---

## 2. 编译

```bash
source /opt/ros/humble/setup.bash
cd <仓库根目录>
colcon build --packages-select ego_planner ackermann_car_sim
source install/setup.bash
```

---

## 3. 运行模式一：RViz 假定位

```bash
ros2 launch ackermann_car_sim bringup.launch.py
```

或：

```bash
./start.sh
```

只启动节点、不打开 RViz：

```bash
ros2 launch ackermann_car_sim bringup.launch.py use_rviz:=false
```

### 操作步骤

1. 点击 RViz 工具栏 **2D Pose Estimate**，设置初始位置和朝向。
2. 点击 **2D Goal Pose**，设置目标点。
3. 规划器自动生成 A* 全局路线和 B 样条轨迹。
4. 假定位节点驱动车模沿 B 样条轨迹运动。

### 可选：添加障碍物

点击 **Publish Point** 添加障碍物点，规划器会重新规划并绕开障碍物。

---

## 4. 运行模式二：Gazebo 真实底盘

```bash
ros2 launch ackermann_car_sim gazebo.launch.py
```

无 Gazebo GUI（适合服务器或无显示环境）：

```bash
ros2 launch ackermann_car_sim gazebo.launch.py use_gui:=false
```

无 Gazebo GUI 且无 RViz：

```bash
ros2 launch ackermann_car_sim gazebo.launch.py use_gui:=false use_rviz:=false
```

或使用脚本：

```bash
./start_gazebo.sh
```

### Gazebo 模式操作步骤

1. Gazebo 在原点生成 Hunter 底盘。
2. `/ackermann_steering_controller/odometry` 自动向规划器提供当前位姿，因此不需要 `2D Pose Estimate`。
3. 在 RViz 中点击 **2D Goal Pose** 设置目标。
4. `gazebo_path_follower` 将 B 样条轨迹转换为 `TwistStamped` 速度指令。
5. `ackermann_steering_controller` 根据 Ackermann 运动学控制前轮转角和后轮速度。
6. Gazebo 物理引擎驱动底盘运动，里程计反馈给规划器形成闭环。

### Gazebo 控制链

```text
EGO Planner B-spline
        ↓ /visual_local_trajectory
gazebo_path_follower
        ↓ /ackermann_steering_controller/reference  (TwistStamped)
ackermann_steering_controller
        ↓ ros2_control command interfaces
gazebo_ros2_control / Gazebo Classic
        ↓
Hunter chassis physics
        ↓ odometry + TF
planner / RViz
```

### 控制器检查

```bash
ros2 control list_controllers
```

正常输出应包含：

```text
joint_state_broadcaster         active
ackermann_steering_controller   active
```

### 主要 Gazebo 话题

| 话题 | 类型 | 说明 |
| --- | --- | --- |
| `/ackermann_steering_controller/reference` | `geometry_msgs/msg/TwistStamped` | 路径跟踪节点输出的线速度/角速度 |
| `/ackermann_steering_controller/odometry` | `nav_msgs/msg/Odometry` | Gazebo 底盘里程计 |
| `/ackermann_steering_controller/tf_odometry` | `tf2_msgs/msg/TFMessage` | 控制器里程计 TF |
| `/ackermann_steering_controller/controller_state` | `control_msgs/msg/SteeringControllerStatus` | 转向角与车轮速度状态 |
| `/joint_states` | `sensor_msgs/msg/JointState` | 四轮及前轮转向关节状态 |
| `/current_pose` | `geometry_msgs/msg/PoseStamped` | 转换到 `map` 坐标系的当前位姿 |

---

## 5. RViz 显示说明

| 显示项 | 颜色/类型 | 话题 |
| --- | --- | --- |
| Reference Path | 蓝色线 | `/visual_global_path` |
| A* Global Route | 橙红色线 | `/trajectories` |
| B-spline Optimized Path | 绿色线 | `/visual_local_trajectory` |
| Ackermann Model | URDF 机器人模型 | `/robot_description` + `/joint_states` |
| Obstacles | 白色点云 | `/visual_obstacles` |

Gazebo 模式下需要保证以下 TF 链存在：

```text
map -> odom -> base_footprint -> base_link -> 车轮/转向节
```

---

## 6. 阿克曼模型与 ros2_control

URDF/xacro 文件：

```text
src/ackermann_car_sim/urdf/ackermann_hunter.urdf.xacro
```

主要参数：

| 参数 | 默认值 |
| --- | --- |
| 轴距 wheelbase | 0.65142 m |
| 轮距 track | 0.584 m |
| 车轮半径 wheel_radius | 0.164 m |
| 车轮宽度 wheel_width | 0.142 m |
| 最大转向角 | ±0.50 rad（仿真控制） |
| 前轮转向关节 | `front_left_steer_joint` / `front_right_steer_joint` |
| 后轮驱动关节 | `rear_left_wheel_joint` / `rear_right_wheel_joint` |
| 前轮被动滚动关节 | `front_left_wheel_joint` / `front_right_wheel_joint` |

URDF 中已经包含：

- 惯性、碰撞体、视觉几何
- 前轮转向 `revolute` 关节
- 四轮 `continuous` 滚动关节
- `gazebo_ros2_control/GazeboSystem` 硬件插件
- `ros2_control` 状态接口与命令接口

控制配置：

```text
src/ackermann_car_sim/config/gazebo_controllers.yaml
```

Gazebo 世界：

```text
src/ackermann_car_sim/worlds/ackermann_ground.world
```

---

## 7. 工程结构

```text
src/
├── EgoPlanner-ROS2/
│   ├── planner/                         # EGO Planner 2D、A*、B 样条优化
│   └── src/trajectory_publisher.cpp
└── ackermann_car_sim/
    ├── config/ackermann.rviz
    ├── config/gazebo_controllers.yaml
    ├── launch/bringup.launch.py          # RViz 假定位
    ├── launch/gazebo.launch.py           # Gazebo 真实底盘
    ├── src/ackermann_sim_node.cpp        # RViz 假定位与关节动画
    ├── src/gazebo_path_follower.cpp      # Gazebo 路径跟踪与里程计桥接
    ├── urdf/ackermann_hunter.urdf.xacro
    └── worlds/ackermann_ground.world
```

---

## 8. 验证命令

```bash
# URDF 合法性
xacro src/ackermann_car_sim/urdf/ackermann_hunter.urdf.xacro > /tmp/ackermann.urdf
check_urdf /tmp/ackermann.urdf

# 规划与轨迹
ros2 topic echo /trajectories --once
ros2 topic echo /visual_local_trajectory --once

# Gazebo 底盘
ros2 control list_controllers
ros2 topic echo /ackermann_steering_controller/odometry --once
ros2 topic echo /joint_states --once
ros2 run tf2_ros tf2_echo map base_footprint
```

---

## 9. 当前限制

- 使用 Gazebo Classic 11，不是 Ignition/Gazebo Sim。
- 当前未接入激光雷达/IMU/相机等传感器插件。
- 当前障碍物只用于规划器可视化；Gazebo 世界中尚未自动生成对应碰撞障碍物。
- Gazebo 模式不依赖 `2D Pose Estimate`，初始位姿来自 Gazebo 里程计；如需重置车辆位置，重新启动 Gazebo 即可。
- 简化车身几何用于快速验证控制与规划；后续可替换为官方 Hunter STL/DAE 网格。
