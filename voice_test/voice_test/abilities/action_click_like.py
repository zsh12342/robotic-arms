#!/usr/bin/env python3
import rospy
import rosbag
import numpy as np
import time
from typing import List
import requests
import json
from collections import defaultdict

from .registry import ability
from .utils import Replay
from .kuavoRobotSDK import kuavo

csv_data = {
    'click_like': [
        "hand_pose0 1 20 50   0   0 10   0 0 20 0 0 -30 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0",
        "hand_pose1 2.5 -20 80   0 -50 45 -40 0 20 0 0 -30 0 0 0 100 0 0 0 0 0 0 100 0 0 0 0",
        "hand_pose2 3.5 -50 50   0 -30  0 -50 0 20 0 0 -30 0 0 0 0 0 80 80 80 80 0 0 80 80 80 80",
        "hand_pose3 4.5 -50 10 -10 -60  0   0 0 20 0 0 -30 0 0 0 0 0 80 80 80 80 0 0 80 80 80 80",
        "hand_pose4 6 -20 80   0 -50 45 -40 0 20 0 0 -30 0 0 0 100 0 0 0 0 0 0 100 0 0 0 0",
        "hand_pose5 7.5 20 0 0 -30 0 0 0 20 0 0 -30 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0"
    ]
}

def parse_csv_data(data):
    time_data = []
    traj_array = []
    for row in data:
        columns = row.split()
        time_data.append(float(columns[1]))
        traj_array.extend([float(a) for a in columns[2:16]])
    return time_data, traj_array

def test_arm_click_like(robot_instance):
    """
        kuavo -- 手部 /kuavo_arm_target_poses 控制
    """
    data = csv_data['click_like']
    time_data, traj_array = parse_csv_data(data)
    robot_instance.pub_kuavo_arm_with_time(time_data, traj_array)
    print("robot arm traj publisher finish")
    time.sleep(1)

@ability(
    name="click_like",
    description="机器人点赞",
    parameters=[
        {
            "name": "obj_name",
            "description": "物品名称",
            "type": "str",
            "required": True
        }
    ],
    output_type="None",
)
async def click_like(obj_name: str):
    print("调用了click_like函数")
    # 初始化机器人
    robot_instance = kuavo("3_7_kuavo")
    
    # 等待1s
    time.sleep(1)
    
    # 机器人做动作
    test_arm_click_like(robot_instance)