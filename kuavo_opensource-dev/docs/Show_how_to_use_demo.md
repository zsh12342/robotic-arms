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
```
### ...update_new_demo...