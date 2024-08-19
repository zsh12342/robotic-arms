# 基础环境准备

1. **安装 ROS1 的 noetic 版本** - 确认 `/opt/ros/noetic/setup.bash` 文件是否存在，以验证是否已安装。
2. **新建 ROS1 的 workspace** - 在 `~/catkin_ws/src` 下克隆 kuavo 仓库：
   ```
   cd ~/catkin_ws && mkdir -p src && git clone https://gitee.com/leju-robot/kuavo_opensource.git kuavo
   ```
3. **编译工程** - 在 `~/catkin_ws`，执行 `source ~/catkin_ws/devel/setup.bash` 并使用 `catkin_make` 编译。编译错误时请参考错误信息。
4. **启动机器人主节点** - 成功编译后，运行 `source ~/catkin_ws/devel/setup.bash` 并使用 `roslaunch dynamic_biped highly_dynamic_robot.launch` 启动。

## 依赖的库

1. geometry_msgs

  1.1 检查库是否安装：`rospack find geometry_msgs`。
  
  1.2 安装库：
  ```
  sudo apt-get update
  sudo apt-get install ros-noetic-geometry-msgs
  ```

# Topics

## /robot_hand_position

```bash
rostopic info /robot_hand_position

Output:
Type: dynamic_biped/robotHandPosition

Publishers:
 * /HDrobot_node

Subscribers: None
```

* 运行示例，能够控制机器人头部运动, 机器人头部会先水平转动，再垂直转动
```bash
source devel/setup.bash
python3 src/kuavo/ros_package/scripts/head_motion_control_demo.py 
```
* 自行修改`src/kuavo/ros_package/scripts/head_motion_control_demo.py`中的`msg.joint_data`数据，可以控制机器人头部的运动方向，修改`time.sleep()`的时间间隔，可以控制机器人头部运动的速度。

* 主程序会收到头部关节数据
```bash
[ INFO] [1712991040.763163738]: Received robot head motion data joint_data: [-20.000000, 0.000000]
[ INFO] [1712991040.863136754]: Received robot head motion data joint_data: [-17.000000, 0.000000]
[ INFO] [1712991040.963065995]: Received robot head motion data joint_data: [-14.000000, 0.000000]
[ INFO] [1712991041.063127669]: Received robot head motion data joint_data: [-11.000000, 0.000000]
[ INFO] [1712991041.163072952]: Received robot head motion data joint_data: [-8.000000, 0.000000]
[ INFO] [1712991041.263052537]: Received robot head motion data joint_data: [-5.000000, 0.000000]
```

### ROS 话题详情
* 话题名称：`/robot_head_motion_data`
* MSG 类型：`dynamic_biped/robotHeadMotionData`
* MSG 请求：`float64[] joint_data`
* MSG 说明： 
    * `joint_data`：头部关节数据，数组长度为 2
    * `joint_data[0]`: yaw 关节角度，范围为-30 到 30
    * `joint_data[1]`: pitch 关节角度，范围为-25 到 25

### 注意事项
* 请确已经`source devel/setup.bash`。
* 请确保节点运行在正确的 ROS 工作空间下。
* 请确保相关依赖已经安装。

## /robot_hand_position

```bash
rostopic info /robot_hand_position

Output:
Type: dynamic_biped/robotHandPosition

Publishers: 
 * /HDrobot_node (http://rongman-ubuntu:46459/)

Subscribers: None
```

* 运行示例，能够收到机器人手掌位置信息
```bash
source devel/setup.bash
python3 src/kuavo/ros_package/src/get_end_hand_demo.py 

Output:
[INFO] [1712989416.423139]: left hand position: [80, 0, 0, 0, 0, 0]
[INFO] [1712989416.424122]: right hand position: [100, 0, 0, 0, 0, 0]
[INFO] [1712989417.423166]: left hand position: [80, 0, 0, 0, 0, 0]
[INFO] [1712989417.424141]: right hand position: [100, 0, 0, 0, 0, 0]
[INFO] [1712989418.423142]: left hand position: [80, 0, 0, 0, 0, 0]
[INFO] [1712989418.424029]: right hand position: [100, 0, 0, 0, 0, 0]
```

* 使用 ROS 命令行工具
```bash
rostopic echo /robot_hand_position

Output:
left_hand_position: [0, 0, 0, 0, 0, 0]
right_hand_position: [0, 0, 0, 0, 0, 0]
---
left_hand_position: [0, 0, 0, 0, 0, 0]
right_hand_position: [0, 0, 0, 0, 0, 0]
```

### ROS 话题详情
* 话题名称：`/robot_hand_position`
* MSG 类型：`dynamic_biped/robotHandPosition`
* MSG 请求：`uint8[] left_hand_position, uint8[] right_hand_position`
* MSG 说明： 
    * `left_hand_position`：
       * 左手掌位置时，数组长度为 6，对应大拇指关节，拇指外展肌，食指关节, 中指关节，无名指关节，小指关节
       * 夹爪位置时， left_hand_position[0]表示夹爪， 其他值 unused
    * `right_hand_position`：
       * 右手掌位置时，数组长度为 6，对应大拇指关节，拇指外展肌，食指关节, 中指关节，无名指关节，小指关节
       * 夹爪位置时，  right_hand_position[0]表示夹爪， 其他值 unused
    * 位置范围：灵巧手：0-100，夹爪：0-255 0 为打开，255 为关闭


### 注意事项
* 请确已经`source devel/setup.bash`。
* 请确保节点运行在正确的 ROS 工作空间下。
* 请确保相关依赖已经安装。

## robot_q_v_tau

- **数据格式**
  ```
  float64[] q
  float64[] v
  float64[] tau
  ```
- **说明**：`q` 的前 4 个值为躯干四元数位置，`v` 和 `tau` 分别表示速度和力矩。

## robot_torso_state

- **数据格式**
  ```
  geometry_msgs/Vector3 torsoR
  geometry_msgs/Vector3 torsoRd
  geometry_msgs/Vector3 torsoRdd
  geometry_msgs/Vector3 r
  geometry_msgs/Vector3 rd
  geometry_msgs/Vector3 rdd
  ```
- **说明**：torsoR 躯干旋转角度，torsoRd 躯干旋转速度，torsoRdd 躯干旋转加速度。
- **说明**：r 质心旋转角度， rd 质心旋转速度， rdd 质心旋转加速度

## kuavo_arm_traj

- **数据格式**：参考 [sensor_msgs/JointState](https://docs.ros.org/en/api/sensor_msgs/html/msg/JointState.html)。
- **说明**：控制手臂关节位置。目前仅使用位置信息，速度和力矩被忽略。

## kuavo_arm_target_poses

- **数据格式**：参考 [std_msgs/Float64MultiArray](https://docs.ros.org/en/api/std_msgs/html/msg/Float64MultiArray.html)。
- **说明**：控制手臂目标位置。`times` 表示时间序列，`表示手臂和手掌关节的位置`, 位置的关系为 `[手臂左 手臂右 左手 右手]` 手臂的位置顺序请参考 topic  `\kuavo_arm_traj`。手指的位置顺序请参考 topic `\robot_hand_position`

## walkCommand

- **数据格式**
  ```
  # mode: 0->PositionCommand | 1->VelocityCommand
  uint8 mode
  float64[] values
  ```
- **说明**：建议使用速度控制。`values` 包含 x,y 轴速度和 yaw 角速度。

## leju_robot_phase
数据格式
* robotPhase.msg
```
uint8 mainPhase
uint8 subPhase
```

- **说明**：mainPhase 机器人主状态， subPhase 机器人子状态

## robot_imu_gyro_acc

**数据格式**
* robotImuGyroAcc.msg
```
geometry_msgs/Vector3 gyro
geometry_msgs/Vector3 acc
```

- **说明**：gyro 表示 IMU 的角速度，acc 表示 IMU 的加速度。

## robot_head_motor_position

**数据格式**
* robotHeadMotionData.msg
```
float64[] joint_data
```
* MSG 说明： 
    * `joint_data`：头部关节数据，数组长度为 2
    * `joint_data[0]`: yaw 关节角度，
    * `joint_data[1]`: pitch 关节角度

# Services

## setPhase

- **数据格式**
  ```
  uint8 masterID
  string stateReq
  string subState
  ---
  int16 stateRes
  ```
- **说明**：`masterID` 通常为 0，`stateReq` 和 `subState` 分别为主状态和子状态。

## change_arm_ctrl_mode

- **数据格式**
  ```
  bool control_mode
  ---
  bool result
  ```
- **说明**：在机器人稳定站立时，通过设置 `control_mode` 为 true 来使能手臂控制模式。

## update_center_of_mass_position 服务文档

### 服务概览

- **服务名称**：`update_center_of_mass_position`
- **服务类型**：自定义服务
  - **请求类型**：
    - `geometry_msgs/Vector3` position
    - `float32` time
    - `float32` acc
  - **响应类型**：
    - `bool` success

### 服务描述

该服务用于更新或检索机器人质心的位置。客户端通过发送包含质心位置的`geometry_msgs/Vector3`消息来请求更新或检索。服务将处理请求，并返回一个布尔值`success`，指示操作是否成功。

`time` 为执行的时间单位为秒，`acc` 为执行的加速度单位为 米/秒，当两个同时存在时，`time` 的优先级更高，会使用 `time` 。

### 使用说明

客户端需向服务端点发送包含质心位置的请求以使用该服务。服务将执行相应的操作，该操作负责更新或检索机器人质心的位置，并返回一个布尔值，表示操作的结果。
只能在站立模式下使用。

### 请求数据细节

- **position**：包含质心位置的`geometry_msgs/Vector3`消息。其中，`x`分量代表质心在 X 轴上的位置，`y`分量代表质心在 Y 轴上的位置，`z`分量代表质心在 Z 轴上的位置。
- **time**：执行的时间，单位为秒。默认值为 0，不起作用，由加速度控制插值，如果设定时间，则使用用户的时间来进行插值，忽略加速度。
- **acc**：执行的加速度，单位为米/秒²。默认值为 0.5。

### 响应数据细节

- **success**：一个布尔值，`true`表示操作成功，`false`表示操作失败。

### 示例用法

```bash
rosservice call /update_center_of_mass_position "position: {x: 0.0, y: 0.0, z: 0.6} time: 0.0 acc: 0.5"
```

请确保服务已正确集成到您的 ROS 包中，并且您的 `package.xml` 和 `CMakeLists.txt` 文件中正确定义了必要的依赖项。

## /get_center_of_mass_position 服务文档

### 服务概览

- **服务名称**：`/get_center_of_mass_position`
- **服务类型**：自定义服务，响应类型为 `geometry_msgs/Vector3`

### 服务描述

该服务用于返回机器人质心（COM）的位置。客户端通过发送请求来获取质心位置的`geometry_msgs/Vector3`消息。服务将处理请求，并返回一个包含质心位置的`geometry_msgs/Vector3`消息。

### 使用说明

客户端需向服务端点发送请求以使用该服务。服务将处理请求，并返回一个包含质心位置的`geometry_msgs/Vector3`消息。

### 响应数据细节

- **com_value**：包含质心位置的`geometry_msgs/Vector3`消息。其中，`x`分量代表质心在 X 轴上的位置，`y`分量代表质心在 Y 轴上的位置，`z`分量代表质心在 Z 轴上的位置。

### 示例用法


```bash
rosservice call /get_center_of_mass_position
```

请确保服务已正确集成到您的 ROS 包中，并且您的 `package.xml` 和 `CMakeLists.txt` 文件中正确定义了必要的依赖项。

## /control_jodell_claw_position 服务文档

### 服务概览

- **服务名称**：`/control_jodell_claw_position`
- **服务类型**：`uint left_claw_position` 左夹爪的位置，`uint right_claw_position` 右夹爪的位置。

### 服务描述

该服务用于控制均舵夹爪的开合位置。

### 使用说明

客户端需向服务端点发送请求以使用该服务，服务将处理请求，并返回`{bool:result}`处理结果。

注意：夹爪的开合位置范围是[0, 255]

### 响应数据细节

- **result**：执行的结果，`true`表示成功，`false`表示失败。

#### 示例用法

```bash
rosservice call /control_jodell_claw_position "{left_claw_position: 0, right_claw_position: 0}"

rosservice call /control_jodell_claw_position "{left_claw_position: 200, right_claw_position: 200}"
```
请确保服务已正确集成到您的 ROS 包中，并且您的 `package.xml` 和 `CMakeLists.txt` 文件中正确定义了必要的依赖项。

---

# kuavo机器人文件夹下的demo案例展示
## 文件结构如下
```bash
└── demo_action  # 机器人动作演示案例
    ├── arm_move_X.py           # 机械臂运动到指定位置-案例演示
    ├── arm_specific_actions.py # 机械臂特殊动作-案例演示
    ├── kuavoRobotSDK.py        # 机器人控制基类
    ├── utils.py                # 为arm_specific_actions.py提供moveit使用函数
    ├── walk_rectangle.py       # 机器人正方形行走-案例演示
    ├── traj       # 存放moveit预置好的不同动作的轨迹动作json文件
    │   ├── Bowing          # 作揖
    │   ├── SayHello        # 打招呼
    │   └── Sending_flowers # 送花    
└── ...update_new_demo...
```
---
## 启动功能脚本
### demo_action
* 首先确保`ros master`主节点已经启动
* 保证机器人节点的启动，具体ros启动机器人节点的方式请参考同目录下的`control_nuc_ros1.md`文档
* 之后来到对应的demo文件夹下, 并且通过source引入对应的devel环境, `source your_robot_workspace/devel/setup.bash`
```bash
# 在demo/demo_action下
python3 arm_move_X.py             # 机械臂运动到指定位置-案例演示
python3 arm_specific_actions.py   # 机械臂特殊动作-案例演示
python3 walk_rectangle.py         # 机器人正方形行走-案例演示