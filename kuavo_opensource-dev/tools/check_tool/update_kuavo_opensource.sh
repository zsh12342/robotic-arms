###
 # @Author: dongdongmingming
 # @Date: 2024-03-28 15:46:28
 # @LastEditors: Please set LastEditors
 # @LastEditTime: 2024-04-20 14:53:53
 # @FilePath: /kuavo/tools/check_tool/update_kuavo.sh
 # @Description: 更新 kuavo_opensource 目录程序
### 


#!/bin/bash



opensource_path="/home/lab/kuavo_opensource/"

if [[ ! -d "${opensource_path}" ]]; then
    echo "没有找到 $opensource_path 目录"
    exit 1
fi


# 切换到仓库目录
cd ${opensource_path}

# 强制更新到最新版本
git fetch origin
git reset --hard origin/dev

# 切换到 dev 分支
# echo "正在切换到 dev 分支..."
git checkout -f dev

git pull origin dev --allow-unrelated-histories


# 提示更新完成
echo "./kuavo_opensource 已将仓库中的 dev 分支内容更新到最新版本。"




opensource_path="/home/lab/kuavo_ros_catkin_ws/src/kuavo_opensource"

if [[ ! -d "${opensource_path}" ]]; then
    echo "没有找到 $opensource_path 目录"
    exit 1
fi


# 切换到仓库目录
cd ${opensource_path}

# 强制更新到最新版本
git fetch origin
git reset --hard origin/dev

# 切换到 dev 分支
# echo "正在切换到 dev 分支..."
git checkout -f dev

git pull origin dev --allow-unrelated-histories


sudo chown -R lab:lab .


# 提示更新完成
echo "./kuavo_opensource 已将仓库中的 dev 分支内容更新到最新版本。"


