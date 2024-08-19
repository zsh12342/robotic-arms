#!/usr/bin/env python3
"""
utils.py
- 这个模块提供了用于arm_specific_actions.py的工具函数和辅助功能。
- 它包含了加载配置、加载关节轨迹、存储轨迹、角度和弧度转换等功能。
- 还包括了针对机器人左右手对称映射、校准目标姿态和手抓位置的转换函数。
"""
import rospy
import json
import math
import os
import copy
import geometry_msgs.msg
import moveit_msgs.msg
import trajectory_msgs.msg
import rospy_message_converter.json_message_converter


def load_config(path: str) -> dict:
    """加载配置
    """
    path = os.path.join(os.path.dirname(__file__), path)
    with open(path, "r") as f:
        json_data = json.load(f)
    rospy.loginfo("配置已从{}中加载".format(path))
    return json_data


def load_joints(path: str) -> dict:
    """加载轨迹点
    """
    path = os.path.join(os.path.dirname(__file__), path)
    with open(path, "r") as f:
        json_data = json.load(f)
    rospy.loginfo("轨迹关节点已从{}中加载".format(path))
    return json_data


def load_traj(path: str) -> moveit_msgs.msg.RobotTrajectory:
    """加载轨迹
    """
    path = os.path.join(os.path.dirname(__file__), path)
    with open(path, "r") as f:
        traj = json.load(f)
    traj = rospy_message_converter.json_message_converter.convert_json_to_ros_message("moveit_msgs/RobotTrajectory", traj)
    rospy.loginfo("轨迹已从{}中加载".format(path))
    return traj


def dump_traj(path: str, traj: moveit_msgs.msg.RobotTrajectory) -> None:
    """存入轨迹
    """
    traj = rospy_message_converter.json_message_converter.convert_ros_message_to_json(traj)
    path = os.path.join(os.path.dirname(__file__), path)
    with open(path, "w") as f:
        json.dump(traj, f)
    rospy.loginfo("轨迹已保存到{}".format(path))


def rad_to_angle(rad_list: list) -> list:
    """弧度转变为角度
    """
    angle_list = [0 for _ in range(len(rad_list))]
    for i, rad in enumerate(rad_list):
        angle_list[i] = rad / math.pi * 180.0
    return angle_list


def angle_to_rad(angle_list: list) -> list:
    """角度转变为弧度
    """
    rad_list = [0 for _ in range(len(angle_list))]
    for i, angle in enumerate(angle_list):
        rad_list[i] = angle / 180.0 * math.pi
    return rad_list


def l_to_r(l_traj: moveit_msgs.msg.RobotTrajectory) -> moveit_msgs.msg.RobotTrajectory:
    """左手到右手轨迹对称映射
    """
    r_traj = moveit_msgs.msg.RobotTrajectory()
    r_traj.joint_trajectory.header = l_traj.joint_trajectory.header
    r_traj.joint_trajectory.joint_names = [
        "r_arm_pitch",
        "r_arm_roll",
        "r_arm_yaw",
        "r_forearm_pitch",
        "r_forearm_yaw",
        "r_hand_roll",
        "r_hand_pitch"
    ]
    
    for l_point in l_traj.joint_trajectory.points:
        r_point = trajectory_msgs.msg.JointTrajectoryPoint()
        r_point.time_from_start = l_point.time_from_start
        r_point.positions = [
             l_point.positions[0],
            -l_point.positions[1],
            -l_point.positions[2],
             l_point.positions[3],
             l_point.positions[4],
            -l_point.positions[5],
             l_point.positions[6]
        ]
        r_point.velocities = l_point.velocities
        r_point.accelerations = l_point.accelerations
        r_traj.joint_trajectory.points.append(r_point)
    
    return r_traj


def calibration_target_to_grasp(pose: geometry_msgs.msg.Pose) -> geometry_msgs.msg.Pose:
    pose.position.x = pose.position.x + 0.05
    pose.position.z = pose.position.z + 0.2
    return pose

def calibration_gripper_to_eef(pose: geometry_msgs.msg.Pose) -> moveit_msgs.msg.RobotTrajectory:
    """校正手抓位置到末端执行器位置
    """
    
    pose.position.x = pose.position.x - math.cos(15 / 180 * math.pi) * 0.22
    pose.position.z = pose.position.z + math.sin(15 / 180 * math.pi) * 0.22
    
    pose.orientation.x = -0.43052172074893863
    pose.orientation.y = -0.42373614847706753
    pose.orientation.z = 0.5630642655585555
    pose.orientation.w = 0.56396574123629

    return pose

def calibration_target_pose(pose: geometry_msgs.msg.Pose) -> geometry_msgs.msg.Pose:
    cali_pose = copy.deepcopy(pose)
    cali_pose.position.x = cali_pose.position.x + 0.08
    cali_pose.position.z = cali_pose.position.z + 0.12
    return cali_pose

def calibration_support_pose(pose: geometry_msgs.msg.Pose) -> geometry_msgs.msg.Pose:
    cali_pose = copy.deepcopy(pose)
    cali_pose.position.x = cali_pose.position.x + 0.29 - 0.065
    cali_pose.position.y = cali_pose.position.y - 0.4 + 0.075
    cali_pose.position.z = cali_pose.position.z - 0.37
    return cali_pose
