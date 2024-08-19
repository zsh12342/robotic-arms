
###
 # @Author: dongdongmingming
 # @Date: 2024-04-18 14:45:05
 # @LastEditors: Please set LastEditors
 # @LastEditTime: 2024-04-20 14:48:38
 # @FilePath: /kuavo/tools/check_tool/compile_kuavo.sh
 # @Description: 编译kuavo_opensource
### 


#!/bin/bash


# 检查 $USER 是否为 "lab"
if [ "$USER" = "lab" ]; then
    echo "ROBOT_VERSION "$ROBOT_VERSION
else
    echo "当前用户不是 lab，编译请使用 \$ python tools/check_tool/Hardware_tool.py"
    exit
fi


prefix="/home/lab/kuavo_opensource/build"
opensource_path="/home/lab/kuavo_opensource/"

if [[ ! -d "${opensource_path}" ]]; then
    echo "没有找到 $opensource_path 目录"
    exit 1
fi


rm -rf $prefix
mkdir $prefix

cd $prefix
cmake .. && make -j8


