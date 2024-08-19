# 内测版 beta 分支

## 待测试

- 尝试修复 IMU 小概率读取数据出错的问题。

## 文档相关

- 更新了文档中的 Kuavo 配置文件路径。
- 将首页的 readme.md 调整为文档入口目录表格，按大类分类文档的入口
- 增加 kuavo 电源板烧录文档
- VR 手套映射到灵巧手的手部关节名称

## 新增功能

- 在 ROS 参数中添加了末端执行器类型的配置。
- 启用了 Ruiwo Python & C++ 版本的速度控制机制。
- 新增头部控制中对速度的支持。
- 更新测试出厂测试工具 2024_06_27
- 新增夹爪控制的 ROS 服务接口
- 新增 ROBOT_VERSION=41 ，这个型号的机器人为髋电机型号为  PA100_18
- 配置文件会在程序启动的时候同步到 ~/.config 目录中，机器人的配置不会跟着仓库，方便随时切换不同的版本来测试机器人。
- 增加 ruiwo cpp 版本的 SDK，默认启用。会比 python 版本的 SDK 兼容性更好，如果遇到问题，可以回档到 python 版本的 SDK 。通过修改 USE_PYTHON_RUIWO_SDK 这个编译选项即可。
- 增加强脑灵巧手固件更新和左右手设置的工具。（来自: 龙博）
- 增加在 cali 状态下微调零点的功能，只要进入 cali 的时候不堵转可以通过微调零点来快速把零点调好。文档地址: [微调零点工具使用文档](docs/joint_offset_dynamic_reload.md)
- Log 记录中记录状态估计错误码
- ROS 增加 topic 广播当前头部电机的位置
- 添加 ecmaster 实时性检测打印
- ROS 增加一个通过传递 config 配置文件名来控制机器人执行配置文件动作，目前动作包含头部，如果没有头部动作则头部不运动
- 在 log 文件中记录 IMU SDK 上报的原始数据
- 启动的时候打印当前编译的时候 git describe 的版本输出
- 禁用舵机通信失败的提示
- 增加单独测试手掌的每个手指运动的功能，用于验证手掌的固件是不是正常工作
- 检测工具增加检测电源板 485 模块功能
- 增加 --log_lcm 调试日志上传工具，会上传到局域网的文件服务器上用于后续分析。需要用户执行部署一台可以支持 scp 拷贝的服务器
- 用于检测机器人硬件设备的检测工具, ./tools/check_tool/readme.md
- 控制质心位置的 ROS1 接口, docs/control_nuc_ros1.md#update_center_of_mass_position-服务文档
- 可以动态微调零点的功能，docs/joint_offset_dynamic_reload.md
- 手部运动实现插值变化，让手部动作更加流畅
- 手臂控制接口, docs/control_nuc_ros1.md#kuavo_arm_traj
- 支持 PA100-18 电机的配置
- 在生成 log 的时候带上当前 git 仓库的 git describe 信息，便于对照对应版本的代码
- 在 $HOME/.config/lejuconfig/robot_serial_number/robot_serial_number.json 文件夹下增加了一个记录当前机器人编号的配置文件，用于在生成 log 记录的时候根据机器人的编号自动命名 log 文件。

## 修复问题

- 修复了仓库配置文件有新的 `key-value` 无法同步到 `~/.config/lejuconfig` 时的错误。
- ruiwo 的 CPP 版本有问题，暂时切换回 python，目前已经修复，待开发自测完成再重新启用。
- 修复使用 cmake .. 的时候自动安装依赖包需要 root 权限的问题，改成使用 sudo 执行，会询问用户输入密码，不会导致生成的文件需要 root 权限才能修改。
- 头部电机数据判断长度有误导致配置了头部电机之后无法控制头部电机的问题。
- ruiwo 电机控制器问题
- 改进 MPC 计算，避免段错误
- 控制强脑灵巧手左右手数据传递顺序错误
- 修复读取 ruiwo 配置文件可能导致内存泄露的问题

## 配置更新

- `kuavo.json` 配置文件中添加了 `motor_velocities_factor` ，`joint_velocities_limit` 及 `ruiwo_2_joint_name` 相关内容。

# 开发版 dev 分支

