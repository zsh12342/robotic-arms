# log 转换工具

- 执行程序时添加`--log`参数之后, 会将通过`lcm_publish.h`中定义的`lcmPublishValue`,`lcmPublishVector`,`lcmPublishState`等接口发布的数据以比较精确的时间戳(0.1ms)记录到`xxx.bin`文件中
- 日志会先存放于内存映射目录的`/tmp/lcm_log.bin`中, 结束时会将最后一段数据写入/var/lcm_log/目录下, 以时间戳命名。
- 使用`./src/biped_v2/bin2csv`程序可以将 bin 格式的日志转换为 csv 格式，方便使用 plotjuggler 等工具进行可视化。
  - 用法:
  ```shell
  sudo ./src/biped_v2/bin2csv -i <binFilePath> -o <csvFilePath> -s <start_count> -t <end_count>
  ```
  - 其中的一个或者多个参数可以不指定
  - 直接执行时会将最后一段数据转换为 csv 格式, 不传递任何参数时默认是转换最后一次执行输出到`/tmp/lcm_log.bin`中的日志
  - 如日志文件很大时, 可以使用`-s`和`-t`参数指定转换的起始和结束位置, 避免转换整个文件, 节省时间和空间。
  
# log 分析
- plotjuggler 是一个开源的日志可视化工具，可以用来分析转换出来的 csv 格式的日志，用以分析机器人的运动
> ROS 环境请自行安装
```shell
sudo apt update
sudo apt install ros-noetic-plotjuggler* -y
```
- 安装完成之后，使用`rosrun plotjuggler plotjuggler`打开软件，点击加载按钮，加载 [2](#转换工具) 中转换出来的 csv 格式的日志文件。
- 从左侧树状列表中选择需要分析的状态量到右侧的图表中，可以看到该状态量随时间的变化曲线。
- 下面是一些常用的机器人状态量名字和物理量对应关系：
```bash
sensors/joint/q # 传感器中关节位置
sensors/joint/v # 传感器中关节速度
sensors/joint/vdot # 传感器中关节加速度
sensors/joint/current # 传感器中关节电流
sensors/imu/free_acc # 传感器中 imu 去除重力的加速度
sensors/imu/acc # 传感器中 imu 加速度
sensors/imu/gyro # 传感器中 imu 角速度
sensors/imu/quat # 传感器中 imu 四元数
desire/q # 期望的关节位置
desire/v # 期望的关节速度
desire/vdot # 期望的关节加速度
desire/avg_vel_ # 一个周期内躯干平均速度
desire/vel_des # 当前规划模块的行走指令速度
desire/arm/q # 期望的手臂关节位置
desire/arm/v # 期望的手臂关节速度
desire/com/x # 期望的质心位置
desire/com/u # Eigen::Vector3d 类型,前面两项是行走 MPC 输出的 xy 落足点位置，后面一项是线程当前周期的时间
desire/lfoot # 期望的左脚空间位置
desire/lfootv # 期望的左脚空间速度
desire/rfoot # 期望的右脚空间位置
desire/rfootv # 期望的右脚空间速度
desire/lfootvd # 期望的左脚空间加速度
desire/rfootvd # 期望的右脚空间加速度
desire/torsoR # 期望的躯干姿态，欧拉角
desire/torsoRd # 期望的躯干姿态的加速度
desire/L/com # 期望的质心动量
desire/tau # 期望的关节力矩
desire/tau_ratio # 期望的关节力矩比例
desire/tau_max # 期望的关节力矩最大值
desire/phase # 机器人当前主状态
desire/subphase # 机器人当前子状态
desire/walk_contact # 机器人当前行走的接触状态
desire/phase_time # 机器人当前状态持续时间
joint_cmd/q # 实际的下发的关节位置
joint_cmd/v # 实际的下发的关节速度
joint_cmd/vdot # 实际的下发的关节加速度
joint_cmd/torqueOffset # 下发的关节力矩补偿
state/est/contact_force # 状态估计中当前接触力
state/q # 状态估计中机器人关节位置
state/v # 状态估计中机器人关节速度
state/vdot # 状态估计中机器人关节加速度
state/arms_q # 状态估计中手臂关节位置
state/arms_v # 状态估计中手臂关节速度
state/com/x # 状态估计中质心位置
state/com/x_est # 状态估计中质心位置(经过高增益滤波，行走时使用)
state/torsoR # 状态估计中躯干姿态，欧拉角
state/torsoRd # 状态估计中躯干姿态的加速度
state/lfoot # 状态估计中左脚空间位置
state/lfootv # 状态估计中左脚空间速度
state/rfoot # 状态估计中右脚空间位置 
state/rfootv # 状态估计中右脚空间速度
state/L/com # 状态估计中质心动量
state/phase # 状态估计中机器人主状态
state/subphase # 状态估计中机器人子状态
state/phase_time # 状态估计中机器人主状态持续时间
state/est/acc_w # 状态估计中机器人加速度
state/est/gyro_w # 状态估计中机器人角速度
state/est/quatvec # 状态估计中机器人四元数
state/est/euler # 状态估计中机器人欧拉角
state/est/base_in_foot # 状态估计中机器人基座到脚的位置
wbc/cost # wbc 的 cost
wbc/qdd # wbc 输出的关节加速度
wbc/tau # wbc 输出的关节力矩
``` 

