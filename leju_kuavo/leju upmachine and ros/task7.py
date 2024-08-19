#!/usr/bin/env python3
import rospy
import math
from dynamic_biped.msg import walkCommand, robotPhase
from dynamic_biped.srv import srvChangePhases
from std_msgs.msg import String
from simple_pid import PID


class RotateFindAndWalkController:
    def __init__(self):
        rospy.init_node('rotate_find_and_walk_controller', log_level=rospy.DEBUG)

        self.walk_pub = rospy.Publisher('/walkCommand', walkCommand, queue_size=10)

        rospy.wait_for_service('/setPhase')
        self.set_phase = rospy.ServiceProxy('/setPhase', srvChangePhases)

        rospy.Subscriber('/leju_robot_phase', robotPhase, self.phase_callback)
        rospy.Subscriber('/bottle_detection_node', String, self.object_callback)

        self.current_phase = None
        self.rotation_speed = 0.5  # 旋转速度，单位：弧度/秒
        self.walk_speed = 0.1  # 行走速度，单位：米/秒
        self.total_rotation = 0  # 已旋转的角度
        self.target_rotation = 2 * math.pi  # 目标旋转角度（360度）
        self.bottle_detected = False
        self.bottle_direction = None
        self.bottle_depth = None

        self.image_width = 640  # 假设图像宽度为640像素
        self.pid = PID(Kp=0.001, Ki=0.0001, Kd=0.0005, setpoint=self.image_width / 2)
        self.pid.output_limits = (-self.rotation_speed, self.rotation_speed)

        rospy.loginfo("RotateFindAndWalkController initialized")

    def phase_callback(self, msg):
        self.current_phase = msg.mainPhase

    def object_callback(self, msg):
        if "bottle" in msg.data.lower():
            self.bottle_detected = True
            try:
                parts = msg.data.split(', ')
                conf = float(parts[1].split(': ')[1])
                self.bottle_depth = float(parts[2].split(': ')[1].split(' ')[0])

                # 假设消息中包含瓶子的x坐标信息
                x_pos = int(parts[3].split(': ')[1])
                self.bottle_direction = self.pid(x_pos)

                rospy.loginfo(
                    f"Bottle detected: Confidence={conf:.2f}, Depth={self.bottle_depth:.2f}m, Direction={self.bottle_direction:.2f}")
            except:
                rospy.logwarn("Unable to parse bottle information")
                self.bottle_depth = None
                self.bottle_direction = None
        else:
            self.bottle_detected = False
            self.bottle_depth = None
            self.bottle_direction = None

    def set_robot_phase(self, phase):
        try:
            response = self.set_phase(0, phase, '')
            rospy.loginfo(f"Set phase to {phase}, response: {response.stateRes}")
            return response.stateRes
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")
            return None

    def rotate(self):
        rospy.loginfo("Starting rotation sequence")
        phase_response = self.set_robot_phase('P_walk')
        if phase_response is None:
            rospy.logerr("Failed to set phase to P_walk")
            return

        rospy.sleep(1)  # 等待相位切换

        rate = rospy.Rate(10)  # 10Hz
        start_time = rospy.Time.now()

        while not self.bottle_detected and not rospy.is_shutdown():
            elapsed_time = (rospy.Time.now() - start_time).to_sec()
            self.total_rotation = self.rotation_speed * elapsed_time

            if self.total_rotation >= self.target_rotation:
                rospy.loginfo("Completed 360 degree rotation without finding bottle")
                break

            walk_cmd = walkCommand()
            walk_cmd.mode = 1  # 步行模式
            walk_cmd.values = [0, 0, self.rotation_speed]  # [x方向速度, y方向速度, 转向角速度]
            self.walk_pub.publish(walk_cmd)

            rate.sleep()

        self.stop_moving()

    def walk_to_bottle(self):
        if not self.bottle_detected:
            rospy.loginfo("No bottle detected, cannot walk")
            return

        rospy.loginfo("Walking towards the bottle")
        rate = rospy.Rate(10)  # 10Hz

        while not rospy.is_shutdown() and self.bottle_depth > 0.5:  # 假设我们想靠近到0.5米
            if self.bottle_direction is not None:
                walk_cmd = walkCommand()
                walk_cmd.mode = 1
                forward_speed = self.walk_speed * (1 - abs(self.bottle_direction) / self.rotation_speed)
                walk_cmd.values = [forward_speed, 0, self.bottle_direction]
                self.walk_pub.publish(walk_cmd)
                rospy.loginfo(f"Walking: speed={forward_speed:.2f}, turn={self.bottle_direction:.2f}")
            else:
                self.stop_moving()
                rospy.loginfo("Lost sight of the bottle, stopping")

            rate.sleep()

        self.stop_moving()
        rospy.loginfo("Reached the bottle or lost detection")

    def stop_moving(self):
        rospy.loginfo("Stopping movement")
        walk_cmd = walkCommand()
        walk_cmd.mode = 1
        walk_cmd.values = [0.0, 0.0, 0.0]
        self.walk_pub.publish(walk_cmd)

        self.set_robot_phase('P_stand')
        rospy.sleep(1)  # 等待相位切换

    def run(self):
        rospy.loginfo("Starting to rotate and find bottle...")
        self.rotate()
        if self.bottle_detected:
            rospy.loginfo("Bottle detected, starting to walk towards it")
            self.walk_to_bottle()
        else:
            rospy.loginfo("No bottle found after complete rotation")

    def shutdown(self):
        rospy.loginfo("Shutting down RotateFindAndWalkController")
        self.stop_moving()


if __name__ == '__main__':
    controller = None
    try:
        controller = RotateFindAndWalkController()
        controller.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Program interrupted by user")
    except KeyboardInterrupt:
        rospy.loginfo("Program interrupted by user")
    finally:
        if controller:
            controller.shutdown()