#!/bin/sh
# 本脚本只能在root用户下运行，每次开机只运行一次
cd LinkOsLayer/Linux/atemsys
make
insmod atemsys.ko
cd ../../..
echo "insmod down!"

echo "0000:72:00.0" > /sys/bus/pci/drivers/igc/unbind
echo "igc unbind!"
