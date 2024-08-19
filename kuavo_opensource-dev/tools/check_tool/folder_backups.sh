###
 # @Author: dongdongmingming
 # @Date: 2024-04-18 14:45:05
 # @LastEditors: Please set LastEditors
 # @LastEditTime: 2024-04-18 16:56:38
 # @FilePath: /kuavo/tools/check_tool/folder_backups.sh
 # @Description: 保存配置文件同步到 kuavo_opensource
### 

#!/bin/bash



# 获取当前脚本所在文件夹的绝对路径
current_script_dir=$(dirname "$(realpath "$0")")


# 使用字符串操作来截取前部分路径
prefix="${current_script_dir%/tools/check_tool}"




source_offset_dir=/home/lab/.config/lejuconfig/offset.csv


source_config_dir="$prefix/src/biped_v2/config/kuavo_v3.4/"
source_urdf_dir="$prefix/models/biped_gen3.4/urdf/"

target_config_dir=/home/lab/kuavo_opensource/src/biped_v2/config/kuavo_v3.4/
target_urdf_dir=/home/lab/kuavo_opensource/models/biped_gen3.4/urdf/


ros_config_dir=/home/lab/kuavo_ros_catkin/src/kuavo_opensource/src/biped_v2/config/kuavo_v3.4/
ros_urdf_dir=/home/lab/kuavo_ros_catkin/src/kuavo_opensource/models/biped_gen3.4/urdf/


source_config_dir_4="$prefix/src/biped_v2/config/kuavo_v4.0/"
source_urdf_dir_4="$prefix/models/biped_gen4.0/urdf/"

target_config_dir_4=/home/lab/kuavo_opensource/src/biped_v2/config/kuavo_v4.0/
target_urdf_dir_4=/home/lab/kuavo_opensource/models/biped_gen4.0/urdf/


ros_config_dir_4=/home/lab/kuavo_ros_catkin/src/kuavo_opensource/src/biped_v2/config/kuavo_v4.0/
ros_urdf_dir_4=/home/lab/kuavo_ros_catkin/src/kuavo_opensource/models/biped_gen4.0/urdf/


# ----------------------------------- 打包备份 ----------------------------------- #
# 定义要打包的文件和目录列表
files=(
    $source_config_dir
    $source_urdf_dir
    $source_offset_dir
    $source_config_dir_4
    $source_urdf_dir_4
    $source_offset_dir_4
)

# 定义压缩文件的名称
zipfile=/home/lab/confirm_backups.zip

# 打包文件
zip -r $zipfile "${files[@]}"


rm -r /home/lab/ssh/*

# 提示打包完成
echo "文件已打包为 $zipfile ，已删除 ssh key"



# ----------------------------------同步文件 ----------------------------------- #


opensource_path="/home/lab/kuavo_opensource/"

if [[ ! -d "${opensource_path}" ]]; then
    echo "没有找到 $opensource_path 目录"
    exit 1
fi

# 使用rsync命令替换目标目录
rsync -av --delete $source_config_dir $target_config_dir
rsync -av --delete $source_urdf_dir $target_urdf_dir


rsync -av --delete $source_config_dir_4 $target_config_dir_4
rsync -av --delete $source_urdf_dir_4 $target_urdf_dir_4


# 提示替换完成
echo "kuavo_opensource文件同步完成"


opensource_path="/home/lab/kuavo_ros_catkin/src/kuavo_opensource/"

if [[ ! -d "${opensource_path}" ]]; then
    echo "没有找到 $opensource_path 目录"
    exit 1
fi

# 使用rsync命令替换目标目录
rsync -av --delete $source_config_dir $ros_config_dir
rsync -av --delete $source_urdf_dir $ros_urdf_dir

rsync -av --delete $source_config_dir_4 $ros_config_dir_4
rsync -av --delete $source_urdf_dir_4 $ros_urdf_dir_4

# 提示替换完成
echo "kuavo_ros_catkin kuavo_opensource文件同步完成"




