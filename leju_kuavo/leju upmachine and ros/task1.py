#!/usr/bin/env python3
import rospy
import math
from std_msgs.msg import String
from dynamic_biped.msg import walkCommand, robotPhase, armTargetPoses
from dynamic_biped.srv import srvChangePhases, changeArmCtrlMode
from sensor_msgs.msg import JointState
import simple_pid

class KuavoController:
    def __init__(self):
        rospy.init_node('kuavo_depth_based_movement', log_level=rospy.DEBUG)
        
        self.walk_pub = rospy.Publisher('/walkCommand', walkCommand, queue_size=10)
        self.arm_pub = rospy.Publisher('/kuavo_arm_traj', JointState, queue_size=10)
        
        rospy.wait_for_service('/setPhase')
        self.set_phase = rospy.ServiceProxy('/setPhase', srvChangePhases)
        rospy.wait_for_service('/change_arm_ctrl_mode')
        self.set_arm_mode = rospy.ServiceProxy('/change_arm_ctrl_mode', changeArmCtrlMode)
        
        rospy.Subscriber('/leju_robot_phase', robotPhase, self.phase_callback)
        rospy.Subscriber('/valve_detection_node', String, self.button_callback)
        
        self.current_phase = None
        self.walk_speed = 0.1  # 行走速度，单位：米/秒
        self.current_depth = None
        self.target_distance = 0.8  # 目标距离，改为0.3米
        self.image_width = 640  # 假设图像宽度为640像素
        self.current_x = None
        self.max_turn_angle = 0.3  # 最大转向角度（弧度）
        self.last_detection_time = rospy.Time.now()
        self.detection_timeout = rospy.Duration(1.0)  # 1秒无检测就停止
        self.button_detected = False
        
        self.pid = simple_pid.PID(Kp=0.1, Ki=0.01, Kd=0.05, setpoint=self.image_width / 2)
        self.pid.output_limits = (-0.2, 0.2)  # 限制输出范围
        
        rospy.loginfo("KuavoController initialized")

    def phase_callback(self, msg):
        rospy.loginfo(f"Phase callback triggered: {msg.mainPhase}")
        self.current_phase = msg.mainPhase

    def button_callback(self, msg):
        rospy.loginfo("Valve detection callback triggered")
        try:
            rospy.loginfo(f"Received message: {msg.data}")
            parts = msg.data.split(', ')
            class_info = parts[0].split(': ')[1]
            confidence = float(parts[1].split(': ')[1])
            depth = float(parts[2].split(': ')[1].split(' ')[0])
            
            self.current_depth = depth
            self.last_detection_time = rospy.Time.now()
            self.button_detected = True
            
            if 'x' in class_info:
                x_center = int(class_info.split('x')[1].split('y')[0])
                self.current_x = x_center
            
            rospy.loginfo(f"Parsed: Class: {class_info}, Confidence: {confidence:.2f}, Depth: {self.current_depth:.2f}m, X: {self.current_x}")
        except Exception as e:
            rospy.logerr(f"Error parsing valve detection data: {e}")
        
    def set_robot_phase(self, phase):
        try:
            response = self.set_phase(0, phase, '')
            rospy.loginfo(f"Set phase to {phase}, response: {response.stateRes}")
            return response.stateRes
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")
            return None
    
    def calculate_turn_angle(self):
        if self.current_x is None:
            return 0.0
        
        turn_angle = self.pid(self.current_x)
        return turn_angle
            
    def walk(self):
        rospy.loginfo("Starting walk sequence")
        phase_response = self.set_robot_phase('P_walk')
        if phase_response is None:
            rospy.logerr("Failed to set phase to P_walk")
            return
        
        rospy.sleep(1)  # 等待相位切换
        
        rate = rospy.Rate(10)  # 10Hz
        
        while self.current_depth > self.target_distance:
            if rospy.Time.now() - self.last_detection_time > self.detection_timeout:
                rospy.loginfo("No recent detection, stopping walk")
                break
            
            turn_angle = self.calculate_turn_angle()
            
            walk_cmd = walkCommand()
            walk_cmd.mode = 1  # 步行模式
            
            # 根据偏离程度调整前进速度
            forward_speed = self.walk_speed * (1 - abs(turn_angle) / 0.2)
            walk_cmd.values = [forward_speed, 0, turn_angle]
            
            self.walk_pub.publish(walk_cmd)
            rospy.loginfo(f"Walking: current depth = {self.current_depth:.2f}m, turn angle = {math.degrees(turn_angle):.2f} degrees")
            
            rate.sleep()
        
        # self.fine_tune_position()
        self.stop_walking()

    # def fine_tune_position(self):
    #     rospy.loginfo("Fine-tuning position")
    #     rate = rospy.Rate(10)
        
    #     for _ in range(50):  # 尝试微调5秒
    #         if abs(self.current_x - self.image_width / 2) < 10:  # 如果已经很接近中心
    #             break
            
    #         turn_angle = self.calculate_turn_angle()
    #         walk_cmd = walkCommand()
    #         walk_cmd.mode = 1
    #         walk_cmd.values = [0.02, 0, turn_angle]  # 很慢的前进速度和转向
    #         self.walk_pub.publish(walk_cmd)
            
    #         rate.sleep()
        
    #     self.stop_walking()

    def stop_walking(self):
        rospy.loginfo("Stopping walk sequence")
        walk_cmd = walkCommand()
        walk_cmd.mode = 1
        walk_cmd.values = [0.0, 0.0, 0.0]
        self.walk_pub.publish(walk_cmd)
        
        self.set_robot_phase('P_stand')
        rospy.sleep(1)  # 等待相位切换
        
    def turn_valve(self):
        rospy.loginfo("执行转阀门动作")
        self.set_arm_mode(True)
        rospy.sleep(1)
        
        arm_positions = [
            [20, 0, 0, -30, 0, 0, 0, -70, 0, 0, 0, 0, 0, 0],
            [20, 0, 0, -30, 0, 0, 0, -85, 0, 0, -85, 0, 0, 0],
            [20, 0, 0, -30, 0, 0, 0, -85, 0, 0, -50, 0, 0, 0],
            [20, 0, 0, -30, 0, 0, 0, -85, 0, 0, -50, -80, 0, 0],
            [20, 0, 0, -30, 0, 0, 0, -35, 0, 0, -50, -80, 0, 35],
            [20, 0, 0, -30, 0, 0, 0, -85, 0, 0, -50, -80, 0, 30],
            [20, 0, 0, -30, 0, 0, 0, -35, 0, 0, -50, -80, 0, 35],
            [20, 0, 0, -30, 0, 0, 0, -85, 0, 0, -50, -80, 0, 30],
            [20, 0, 0, -30, 0, 0, 0, -35, 0, 0, -50, -80, 0, 35],
            [20, 0, 0, -30, 0, 0, 0, -85, 0, 0, -50, -80, 0, 30],
            [20, 0, 0, -30, 0, 0, 0, -35, 0, 0, -50, -80, 0, 35],
            [20, 0, 0, -30, 0, 0, 0, -85, 0, 0, -50, -80, 0, 30],
            [0, 0, 0, 0, 0, 0, -0, 0, 0, 0, 0, 0, 0, 0],
            [20, 0, 0, -30, 0, 0, 0, 20, 0, 0, -30, 0, 0, 0]
        ]
        
        for position in arm_positions:
            arm_msg = JointState()
            arm_msg.position = position
            self.arm_pub.publish(arm_msg)
            rospy.sleep(4)
        
        self.set_arm_mode(False)

    def run(self):
        rospy.loginfo("开始运行控制器，等待检测到阀门...")
        rate = rospy.Rate(1)  # 1Hz for status updates
        action_completed = False
        while not rospy.is_shutdown() and not action_completed:
            if self.button_detected and self.current_depth is not None and rospy.Time.now() - self.last_detection_time <= self.detection_timeout:
                if self.current_depth > self.target_distance:
                    rospy.loginfo(f"当前深度: {self.current_depth:.2f}m, 开始行走")
                    self.walk()
                else:
                    rospy.loginfo(f"当前深度: {self.current_depth:.2f}m, 已达到目标距离")
                    self.turn_valve()
                    action_completed = True
                    rospy.loginfo("动作完成,停止进一步操作")
            else:
                rospy.loginfo("等待检测阀门...")
                self.stop_walking()  # 确保机器人停止行走
            rate.sleep()
        
        if action_completed:
            rospy.loginfo("所有动作已完成,程序结束")

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