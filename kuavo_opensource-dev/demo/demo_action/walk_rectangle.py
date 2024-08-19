#!/usr/bin/env python3
"""Kuavo机器人走一个正方形案例展示

这个案例演示了如何使用Kuavo机器人SDK让机器人走一个正方形。

功能描述：
- 初始化ROS节点
- 初始化Kuavo机器人实例
- 将机器人进入步态控制模式
- 控制机器人走一个正方形
"""

import rospy
import time
from kuavoRobotSDK import kuavo

def turn_left_90(robot_instance):
    for i in range(6):
        robot_instance.set_walk_speed(1, 0.0, 0.0, 15)
        time.sleep(1)

if __name__ == "__main__":
    # 初始化节点
    rospy.init_node('demo_test') 

    # 初始化机器人
    robot_instance = kuavo("3_7_kuavo")

    robot_instance.set_robot_Phases(0, 'P_walk', '')

    # 走一个正方形
    for i in range(4):
        # 直行
        robot_instance.set_walk_speed(1, 0.3, 0.0, 0.0)

        # 等待
        time.sleep(5)
        
        # 转弯90度
        turn_left_90(robot_instance)

    # 结束
    robot_instance.set_robot_Phases(0, 'P_stand', '')
