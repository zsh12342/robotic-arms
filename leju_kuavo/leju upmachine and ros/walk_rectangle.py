#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
from dynamic_biped.msg import walkCommand, robotPhase
from dynamic_biped.srv import srvChangePhases

class KuavoController:
    def __init__(self):
        rospy.init_node('kuavo_depth_based_movement', log_level=rospy.DEBUG)
        
        # 发布者
        self.walk_pub = rospy.Publisher('/walkCommand', walkCommand, queue_size=10)
        
        # 服务客户端
        rospy.wait_for_service('/setPhase')
        self.set_phase = rospy.ServiceProxy('/setPhase', srvChangePhases)
        
        # 订阅者
        rospy.Subscriber('/leju_robot_phase', robotPhase, self.phase_callback)
        rospy.Subscriber('/bottle_detection_node', String, self.bottle_callback)
        
        self.current_phase = None
        self.walk_time = 5.0
        self.walk_speed = 0.2  # 行走速度（米/秒）
        self.current_depth = None
        self.target_distance = 0.5  # 目标停止距离（米）
        
        rospy.loginfo("KuavoController initialized")

    def phase_callback(self, msg):
        rospy.loginfo(f"Phase callback triggered: {msg.mainPhase}")
        self.current_phase = msg.mainPhase

    def bottle_callback(self, msg):
        rospy.loginfo("Bottle detection callback triggered")
        try:
            # 假设消息格式为 "class,x,y,depth"
            rospy.loginfo(f"Received message: {msg.data}")
            _, _, _, depth = msg.data.split(',')
            self.current_depth = float(depth)
            rospy.loginfo(f"Received depth: {self.current_depth:.2f}m")
        except ValueError as e:
            rospy.logerr(f"Error parsing bottle detection data: {e}")
        
    def set_robot_phase(self, phase):
        try:
            response = self.set_phase(0, phase, '')
            rospy.loginfo(f"Set phase to {phase}, response: {response.stateRes}")
            return response.stateRes
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")
            return None
            
    def walk(self):
        rospy.loginfo("Starting walk sequence")
        phase_response = self.set_robot_phase('P_walk')
        if phase_response is None:
            rospy.logerr("Failed to set phase to P_walk")
            return
        
        rospy.sleep(1)  # 等待状态切换
        
        start_time = rospy.Time.now()
        rate = rospy.Rate(100)  # 100Hz
        
        while (rospy.Time.now() - start_time).to_sec() < self.walk_time and self.current_depth > self.target_distance:
            walk_cmd = walkCommand()
            walk_cmd.mode = 1  # 速度控制模式
            walk_cmd.values = [self.walk_speed, 0.0, 0.0]  # [x速度, y速度, 旋转速度]
            self.walk_pub.publish(walk_cmd)
            rospy.loginfo(f"Walking: current depth = {self.current_depth:.2f}m")
            rate.sleep()
        
        # 停止行走
        rospy.loginfo("Stopping walk sequence")
        walk_cmd = walkCommand()
        walk_cmd.mode = 1
        walk_cmd.values = [0.0, 0.0, 0.0]
        self.walk_pub.publish(walk_cmd)
        
        self.set_robot_phase('P_stand')
        rospy.sleep(1)  # 等待稳定
        
    def run(self):
        rospy.loginfo("开始基于深度的移动任务")
        rate = rospy.Rate(1)  # 1Hz for status updates
        while not rospy.is_shutdown():
            if self.current_depth is not None:
                if self.current_depth > self.target_distance:
                    rospy.loginfo(f"当前深度: {self.current_depth:.2f}m, 开始走动")
                    self.walk()
                else:
                    rospy.loginfo(f"当前深度: {self.current_depth:.2f}m, 已达到目标距离")
            else:
                rospy.loginfo("等待深度信息...")
            rate.sleep()

if __name__ == '__main__':
    try:
        controller = KuavoController()
        controller.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Program interrupted by user")
    except KeyboardInterrupt:
        rospy.loginfo("Program interrupted by user")
