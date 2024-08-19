#!/bin/bash
read -p "请输入想要绑定的串口号(数字):" dev
echo "你选择的串口为: /dev/ttyUSB$dev"
# dev=$1
read -p "请输入自定义的串口名(如需绑定灵巧手,请输入stark_serial_L或stark_serial_R):" rule_name
echo "自定义的串口名为: $rule_name"
serial=$(udevadm info --attribute-walk --name=/dev/ttyUSB$dev|grep ATTRS{serial} | cut -d= -f3 | sed 's/"//g'|head -n 1)
# 如果属性不为空，创建一个udev规则文件
if [ -n "$serial" ]; then
echo '正在为序列号为'$serial'的设备生成udev规则...'
echo 'KERNEL=="ttyUSB*", ATTRS{serial}=="'$serial'", MODE:="0777", SYMLINK+="'$rule_name'"' > /etc/udev/rules.d/$rule_name.rules
echo '生成成功! 请重启计算机或者插拔设备以使规则生效。'
else 
echo '未找到序列号，请检查设备是否已连接。'
fi
# 重新加载udev规则
udevadm control --reload-rules