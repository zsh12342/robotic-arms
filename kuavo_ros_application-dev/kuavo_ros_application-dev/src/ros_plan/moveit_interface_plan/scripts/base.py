#!/usr/bin/env python3
# -*- coding: UTF-8 -*-

import os
import queue
import moveit_commander

from exception import MoveitInitError
from utils import load_json


class Base(object):
    """ 初始化moveit配置 """

    # 从config.json中加载所有moveit配置
    _config = load_json(
        os.path.join(
            os.path.dirname(__file__), "config/config.json"
            )
        )
    
    _move_group_name = _config["move_group_name"]
    _gripper_name = _config["gripper_name"]
    _joint_name = _config["joint_name"]
    
    # 规划器基础参数
    _planner_id = _config["planner_id"]
    _num_planning = _config["num_planning"]
    _planning_time = _config["planning_time"]
    _planning_frame = _config["planning_frame"]
    
    # 速度、加速度比例系数
    _max_vel_scaling_factor = _config["max_vel_scaling_factor"]
    _max_acc_scaling_factor = _config["max_acc_scaling_factor"]
    
    # 规划容忍度
    _joint_tolerance = _config["joint_tolerance"]
    _orientation_tolerance = _config["orientation_tolerance"]
    _position_tolerance = _config["position_tolerance"]
    
    # 初始化机械臂、场景控制器、轨迹规划器
    try:
        _robot = moveit_commander.RobotCommander()
        _scene = moveit_commander.PlanningSceneInterface()
        _move_group = moveit_commander.MoveGroupCommander(_move_group_name)
    except:
        raise MoveitInitError("moveit初始化失败")
    

    # 设置轨迹规划器参数
    _move_group.set_planner_id(_planner_id)
    _move_group.set_planning_time(_planning_time)
    _move_group.set_pose_reference_frame(_planning_frame)
    _move_group.set_num_planning_attempts(_num_planning)
    
    _move_group.set_max_acceleration_scaling_factor(_max_acc_scaling_factor)
    _move_group.set_max_velocity_scaling_factor(_max_vel_scaling_factor)
    
    _move_group.set_goal_joint_tolerance(_joint_tolerance)
    _move_group.set_goal_position_tolerance(_position_tolerance)
    _move_group.set_goal_orientation_tolerance(_orientation_tolerance)
    
    # 获取常用配置参数
    _base_frame = _robot.get_planning_frame()
    _eef_link = _move_group.get_end_effector_link()
    
    # 常用参数设定
    _PUBLISH_RATE = _config["publish_rate"]
    _GRIPPER_MOTION_TIME = _config["gripper_motion_time"]
    
    # 规划轨迹队列
    _traj_queue = queue.Queue()
    
    
    

