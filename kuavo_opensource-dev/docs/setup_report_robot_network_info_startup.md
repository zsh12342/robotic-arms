## KUAVO 机器人开机上报自身网络信息

### 概述

本工具为了解决机器人开机后，需要外接显示屏或者其他设备，才能获取机器人IP地址的问题。 因此使用企业微信机器人，机器人开机后，主动上报自身网络信息。

### 使用方法

1. 配置 `ENV` 环境变量文件(`dev_tools/report_robot_network_info_service/RRNIS.env`), 配置如下：

```shell
WEBHOOK_URL=https://qyapi.weixin.qq.com/cgi-bin/webhook/send?key=6485fb08-37fe-4485-ad5d-10860ea43291 # 企业微信机器人的webhook地址, 请根据实际情况配置
ROBOT_SERIAL_NUMBER=RONGMAN_TEST # 机器人序列号, 请根据实际机器人序列号配置
```

2. 执行配置脚本

```shell
cd dev_tools/report_robot_network_info_service
sudo chmod +x setup.sh
./setup.sh
```

3. 手动触发机器人上报网络信息

```shell
sudo systemctl start report_robot_network_info.service
```

4. 重启机器人，查看企业微信机器人是否收到机器人上报的网络信息

### 注意事项

1. 如果需要加入当前提供的默认企微机器人的 `webhook` 地址的目标群，请联系 [calors](huaixian.huang@gmail.com)
2. 如果需要自己创建企业微信机器人，请参考 [企业微信机器人官方文档](https://work.weixin.qq.com/api/doc/90000/90136/91770), 并将 `webhook` 地址配置到 `ENV` 环境变量文件中, 重新执行配置脚本
