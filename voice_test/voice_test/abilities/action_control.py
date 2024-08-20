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

from .action_give_five  import give_five
from .action_say_hello  import say_hello
from .action_click_like import click_like
import asyncio

@ability(
    name="action_control",
    description="调取不同的动作函数",
    parameters=[
        {
            "name": "obj_name",
            "description": "动作名称",
            "type": "str",
            "required": True
        }
    ],
    output_type="None",
)
async def action_control(obj_name: str):
    print("调取action_control动作控制节点")

    # action detect
    if obj_name == "action_say_hello":
        await say_hello("机器人打招呼")
    elif obj_name == "action_give_five":
        await give_five("机器人击掌")
    elif obj_name == "action_click_like":
        await click_like("机器人点赞")