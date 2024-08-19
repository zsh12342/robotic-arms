"""
robot_arm_action.py
 - 这个模块提供了执行不同手臂动作的函数，可以通过传入不同的动作名称执行对应的动作。
 - 它包含了发布机器人手臂轨迹、执行作揖、打招呼、靠近传感器和送花等功能。

 - 打招呼 robot_arm_action(robot_instance, "say_hello")
 - 作揖   robot_arm_action(robot_instance, "bowing")
 - 送花   robot_arm_action(robot_instance, "sending_flowers")
"""
import rospy
import sensor_msgs.msg
import time

from kuavoRobotSDK import kuavo

from utils import rad_to_angle, l_to_r, load_traj

""" -------- moveit plan function ----------- """
def publish_arm_traj(publisher, traj) -> None:
    """发布左手轨迹
    """
    joint_state = sensor_msgs.msg.JointState()
    positions  = [0 for _ in range(14)]
    velocities = [0 for _ in range(14)]
    rate = rospy.Rate(3)
    
    for point in traj.joint_trajectory.points:
        if rospy.is_shutdown():
            rospy.logerr("用户终止程序")
            exit(0)
        positions[0:7] = rad_to_angle(point.positions)
        velocities[0:7] = point.velocities
        joint_state.position = positions
        joint_state.velocity = velocities
        
        publisher.publish(joint_state)
        rate.sleep()

def publish_lr_arm_traj(publisher, l_traj, r_traj) -> None:
    """发布左右手轨迹
    """
    joint_state = sensor_msgs.msg.JointState()
    positions  = [0 for _ in range(14)]
    velocities = [0 for _ in range(14)]
    rate = rospy.Rate(3)
    
    if len(l_traj.joint_trajectory.points) > len(r_traj.joint_trajectory.points):
        for i in range(len(r_traj.joint_trajectory.points)):
            if rospy.is_shutdown():
                rospy.logerr("用户终止程序")
                exit(0)
            positions[0:7] = rad_to_angle(l_traj.joint_trajectory.points[i].positions)
            positions[7:14] = rad_to_angle(r_traj.joint_trajectory.points[i].positions)
            velocities[0:7] = l_traj.joint_trajectory.points[i].velocities
            velocities[7:14] = r_traj.joint_trajectory.points[i].velocities
            joint_state.position = positions
            joint_state.velocity = velocities
            
            publisher.publish(joint_state)
            rate.sleep()
            
        for i in range(len(r_traj.joint_trajectory.points) , len(l_traj.joint_trajectory.points)):
            if rospy.is_shutdown():
                rospy.logerr("用户终止程序")
                exit(0)
            positions[0:7] = rad_to_angle(l_traj.joint_trajectory.points[i].positions)
            positions[7:14] = [0 for _ in range(7)]
            velocities[0:7] = l_traj.joint_trajectory.points[i].velocities
            velocities[7:14] = [0 for _ in range(7)]
            joint_state.position = positions
            joint_state.velocity = velocities
            
            publisher.publish(joint_state)
            rate.sleep()
        
    else:
        for i in range(len(l_traj.joint_trajectory.points)):
            if rospy.is_shutdown():
                rospy.logerr("用户终止程序")
                exit(0)
            positions[0:7] = rad_to_angle(l_traj.joint_trajectory.points[i].positions)
            positions[7:14] = rad_to_angle(r_traj.joint_trajectory.points[i].positions)
            velocities[0:7] = l_traj.joint_trajectory.points[i].velocities
            velocities[7:14] = r_traj.joint_trajectory.points[i].velocities
            joint_state.position = positions
            joint_state.velocity = velocities
            
            publisher.publish(joint_state)
            rate.sleep()
        for i in range(len(l_traj.joint_trajectory.points) , len(r_traj.joint_trajectory.points)):
            if rospy.is_shutdown():
                rospy.logerr("用户终止程序")
                exit(0)
            positions[0:7] = [0 for _ in range(7)]
            positions[7:14] = rad_to_angle(r_traj.joint_trajectory.points[i].positions)
            velocities[0:7] = [0 for _ in range(7)]
            velocities[7:14] = r_traj.joint_trajectory.points[i].velocities
            joint_state.position = positions
            joint_state.velocity = velocities
            
            publisher.publish(joint_state)
            rate.sleep()

""" -------- kuavo arm action function ----------- """
def set_robot_arm_bowing(kuavo_robot, publisher):
    """
       作揖   Bowing
    """
    # load action 
    l_traj = load_traj("./traj/Bowing/traj_l_bainian1.json")
    r_traj = l_to_r(l_traj)
    publish_lr_arm_traj(publisher, l_traj, r_traj)
    l_traj = load_traj("./traj/Bowing/traj_l_bainian2.json")
    r_traj = l_to_r(l_traj)
    publish_lr_arm_traj(publisher, l_traj, r_traj)

    # 等待2s
    time.sleep(2)
    
    # 手臂归中
    kuavo_robot.set_robot_arm_recenter()

def set_robot_arm_say_hello(kuavo_robot, publisher):
    """
       打招呼 say_hello
    """
    # 举起
    l_traj = load_traj("./traj/SayHello/traj_l_SayHello_1.json")
    publish_arm_traj(publisher, l_traj)

    # 抬高
    l_traj = load_traj("./traj/SayHello/traj_l_SayHello_2.json")
    publish_arm_traj(publisher, l_traj)

    # 左右摆手两次
    for i in range(2):
        l_traj = load_traj("./traj/SayHello/traj_l_SayHello_3.json")
        publish_arm_traj(publisher, l_traj)      

        time.sleep(2)

        l_traj = load_traj("./traj/SayHello/traj_l_SayHello_4.json")
        publish_arm_traj(publisher, l_traj)  

    # 等待1s
    time.sleep(1)
    
    # 手臂归中
    kuavo_robot.set_robot_arm_recenter()

def set_robot_arm_sending_flowers(kuavo_robot, publisher):
    """
       送花  sending_flowers
    """
    # load robot_arm_traj_position form json
    l_traj = load_traj("./traj/Sending_flowers/traj_l_Sending_flowers1.json")
    publish_arm_traj(publisher, l_traj)

    l_traj = load_traj("./traj/Sending_flowers/traj_l_Sending_flowers2.json")
    publish_arm_traj(publisher, l_traj)

    # 等待2s
    time.sleep(2)
    
    # 手臂归中
    kuavo_robot.set_robot_arm_recenter()

def robot_arm_action(kuavo_robot, action_name:str)->bool:
    """
        根据输入的动作名字读取对应的特定的动作发布给Kuavo手臂
    """
    robot_arm_publisher = rospy.Publisher(
        "/kuavo_arm_traj",
        sensor_msgs.msg.JointState,
        queue_size=1
    )

    # 执行动作
    if action_name == "bowing":
        # 执行作揖动作
        set_robot_arm_bowing(kuavo_robot, robot_arm_publisher)
        return True
    elif action_name == "say_hello":
        # 执行打招呼动作
        set_robot_arm_say_hello(kuavo_robot, robot_arm_publisher)
        return True
    elif action_name == "sending_flowers":
        # 执行送花动作
        set_robot_arm_sending_flowers(kuavo_robot, robot_arm_publisher)
        return True
    else:
        # 未知动作
        print(f"未知的动作: {action_name}"+"......please check you action again")
        return False

if __name__ == '__main__':  
    # 初始化节点
    rospy.init_node('kuavo_arm_action_node') 

    # 初始化机器人
    robot_instance = kuavo("3_7_kuavo")

    # 等待2s
    time.sleep(2)

    # 机器人进入到手臂规划模式
    robot_instance.set_robot_arm_ctl_mode(True)

    # 机器人执行 动作 打招呼
    robot_arm_action(robot_instance, "say_hello")

    # 等待2s 
    time.sleep(2)

    # 手臂归中
    robot_instance.set_robot_arm_recenter() 

    # 等待1s
    time.sleep(1)

    # 关闭手臂控制
    robot_instance.set_robot_arm_ctl_mode(False)