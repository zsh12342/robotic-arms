#!/bin/bash
###
 # @Author: dongdongmingming
 # @Date: 2024-05-17 09:56:55
 # @LastEditors: Please set LastEditors
 # @LastEditTime: 2024-05-17 10:00:24
 # @FilePath: /kuavo/tools/check_tool/ruiwo_negtive_set.sh
 # @Description: 瑞沃电机设置方向
### 


# 获取当前脚本所在文件夹的绝对路径
current_script_dir=$(dirname "$(realpath "$0")")


# 使用字符串操作来截取前部分路径
prefix="${current_script_dir%/tools/check_tool}"


setZeroPath="$prefix/lib/ruiwo_controller/"

cd $setZeroPath

./Negtive.sh
