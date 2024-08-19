# ros节点与遥控器说明
本文件夹存放`pbcwalk2`的ros接口实现, 整个pbcwalk2可以作为一个ros包放于任意catkin工作空间中使用`catkin`工具进行编译, 也可以到`pbcwalk2/build/`目录下使用`cmake`编译, 无需其他配置。

### 遥控器使用说明
- 克隆相关仓库到ros的工作空间`catkin_ws/src`:
```shell
# 虚拟遥控器仓库
git clone ssh://git@www.lejuhub.com:10026/highlydynamic/virtual-joystick-node.git
# lcm2ros仓库，用于转发lcm消息到遥控器节点使用
git clone ssh://git@www.lejuhub.com:10026/fuyou/lcm2ros.git
# pbcwalk2 即本仓库
git clone ssh://git@www.lejuhub.com:10026/highlydynamic/pbcwalk2.git
cd pbcwalk2
git checkout <包含遥控器接口的分支>
git submodule update --init 
```
- 使用catkin编译即可：
```shell
cd <catkin_ws>
catkin_make
```
- 运行遥控器和机器人节点
`virtual-joystick-node`仓库中提供了launch文件用于启动机器人和遥控器节点：

source 工作空间的bash之后，使用`roslaunch`运行即可
```shell
source devel/setup.bash
## 仿真
roslaunch virtual-joystick-node virtual-joystick-sim.launch
## 实物
roslaunch virtual-joystick-node virtual-joystick-real.launch

```
当然也可以一个一个的开启机器人节点和遥控器节点：
```shell
## 机器人节点
rosrun dynamic_biped highlyDynamicRobot_node --dt=5e-4
## 遥控器节点
rosrun lcm2ros lcm2ros_node
rosrun virtual-joystick-node virtual-joystick-node.py 
```

- 运行程序之后，通过浏览器访问`http://<机器人ip>:8080`即可访问网页端的控制页面(默认登录密码是1234)

### 主题和服务
TODO:整理发布的主题和服务
