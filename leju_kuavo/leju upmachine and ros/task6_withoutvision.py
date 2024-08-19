#!/usr/bin/env python3
import rospy
import math
from dynamic_biped.msg import walkCommand, robotPhase, armTargetPoses
from dynamic_biped.srv import srvChangePhases, changeArmCtrlMode
from sensor_msgs.msg import JointState

class KuavoController:
    def __init__(self):
        rospy.init_node('kuavo_blind_movement', log_level=rospy.DEBUG)
        
        self.walk_pub = rospy.Publisher('/walkCommand', walkCommand, queue_size=10)
        self.arm_pub = rospy.Publisher('/kuavo_arm_traj', armTargetPoses, queue_size=10)
        
        rospy.wait_for_service('/setPhase')
        self.set_phase = rospy.ServiceProxy('/setPhase', srvChangePhases)
        rospy.wait_for_service('/change_arm_ctrl_mode')
        self.set_arm_mode = rospy.ServiceProxy('/change_arm_ctrl_mode', changeArmCtrlMode)
        
        rospy.Subscriber('/leju_robot_phase', robotPhase, self.phase_callback)
        
        self.current_phase = None
        self.walk_speed = 0.1  # 行走速度，单位：米/秒
        self.initial_distance = 0.5  # 初始行走距离
        self.final_distance = 1.5  # 最终行走距离
        
        rospy.loginfo("KuavoController initialized")

    def phase_callback(self, msg):
        rospy.loginfo(f"Phase callback triggered: {msg.mainPhase}")
        self.current_phase = msg.mainPhase

    def set_robot_phase(self, phase):
        try:
            response = self.set_phase(0, phase, '')
            rospy.loginfo(f"Set phase to {phase}, response: {response.stateRes}")
            return response.stateRes
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")
            return None
    
    def walk(self, distance):
        rospy.loginfo(f"Starting walk sequence for {distance} meters")
        phase_response = self.set_robot_phase('P_walk')
        if phase_response is None:
            rospy.logerr("Failed to set phase to P_walk")
            return
        
        rospy.sleep(1)  # 等待相位切换
        
        walk_time = distance / self.walk_speed
        start_time = rospy.Time.now()
        
        rate = rospy.Rate(10)  # 10Hz
        
        while (rospy.Time.now() - start_time).to_sec() < walk_time:
            walk_cmd = walkCommand()
            walk_cmd.mode = 1  # 步行模式
            walk_cmd.values = [self.walk_speed, 0, 0]  # [x方向速度, y方向速度, 转向角速度]
            self.walk_pub.publish(walk_cmd)
            rate.sleep()
        
        self.stop_walking()

    def stop_walking(self):
        rospy.loginfo("Stopping walk sequence")
        walk_cmd = walkCommand()
        walk_cmd.mode = 1
        walk_cmd.values = [0.0, 0.0, 0.0]
        self.walk_pub.publish(walk_cmd)
        
        self.set_robot_phase('P_stand')
        rospy.sleep(1)  # 等待相位切换

    def execute_arm_action(self):
        rospy.loginfo("执行手臂动作")
        self.set_arm_mode(True)
        rospy.sleep(1)

        arm_positions = [
            [20, 0, 0, -30, 0, 0, 0, 20, 0, 0, -30, 0, 0, 0],  # init_hand_pos
            [-5, 0, 0, -30, -90, 0, 0, 20, 0, 0, -30, 0, 0, 0],  # hand_pose0
            [-15, 0, 0, -90, -90, 0, 0, 20, 0, 0, -30, 0, 0, 0],  # hand_pose1
            [-30, 0, 0, 0, -90, 0, -20, 20, 0, 0, -30, 0, 0, 0],  # hand_pose2
            [-90, 0, 0, -10, -90, 0, 20, 20, 0, 0, -30, 0, 0, 0],  # hand_pose3
            [20, 0, 0, -30, 0, 0, 0, 20, 0, 0, -30, 0, 0, 0]  # 回到初始位置
        ]

        for position in arm_positions:
            arm_msg = JointState()
            arm_msg.position = position
            self.arm_pub.publish(arm_msg)
            rospy.sleep(2)

        self.set_arm_mode(False)

    def run(self):
        rospy.loginfo("开始运行控制器，执行盲走序列...")

        # 初始行走
        self.walk(self.initial_distance)
        rospy.loginfo(f"完成初始 {self.initial_distance} 米行走")

        # 执行预设动作
        self.execute_arm_action()
        rospy.loginfo("完成预设动作")

        # 等待5秒
        rospy.loginfo("等待5秒...")
        rospy.sleep(5)

        # 最终行走
        self.walk(self.final_distance)
        rospy.loginfo(f"完成最终 {self.final_distance} 米行走")

        rospy.loginfo("任务完成")

    def shutdown(self):
        rospy.loginfo("Shutting down KuavoController")
        self.stop_walking()

if __name__ == '__main__':
    controller = None
    try:
        controller = KuavoController()
        controller.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Program interrupted by user")
    except KeyboardInterrupt:
        rospy.loginfo("Program interrupted by user")
    finally:
        if controller:
            controller.shutdown()