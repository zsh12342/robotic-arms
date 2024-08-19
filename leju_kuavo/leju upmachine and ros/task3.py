#!/usr/bin/env python3
import rospy
from dynamic_biped.msg import walkCommand, robotPhase
from dynamic_biped.srv import srvChangePhases


class SimpleWalkController:
    def __init__(self):
        rospy.init_node('simple_walk_controller', log_level=rospy.DEBUG)

        self.walk_pub = rospy.Publisher('/walkCommand', walkCommand, queue_size=10)

        rospy.wait_for_service('/setPhase')
        self.set_phase = rospy.ServiceProxy('/setPhase', srvChangePhases)

        rospy.Subscriber('/leju_robot_phase', robotPhase, self.phase_callback)

        self.current_phase = None
        self.walk_speed = 0.1  # 行走速度，单位：米/秒
        self.target_distance = 3.0  # 目标距离，单位：米

        rospy.loginfo("SimpleWalkController initialized")

    def phase_callback(self, msg):
        self.current_phase = msg.mainPhase

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

        rospy.sleep(1)  # 等待相位切换

        start_time = rospy.Time.now()
        rate = rospy.Rate(10)  # 10Hz

        while (rospy.Time.now() - start_time).to_sec() < (self.target_distance / self.walk_speed):
            walk_cmd = walkCommand()
            walk_cmd.mode = 1  # 步行模式
            walk_cmd.values = [self.walk_speed, 0, 0]  # [x方向速度, y方向速度, 转向角速度]
            self.walk_pub.publish(walk_cmd)
            rospy.loginfo(f"Walking: elapsed time = {(rospy.Time.now() - start_time).to_sec():.2f}s")

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

    def run(self):
        rospy.loginfo("Starting to walk 3 meters...")
        self.walk()
        rospy.loginfo("Walk completed")

    def shutdown(self):
        rospy.loginfo("Shutting down SimpleWalkController")
        self.stop_walking()


if __name__ == '__main__':
    controller = None
    try:
        controller = SimpleWalkController()
        controller.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Program interrupted by user")
    except KeyboardInterrupt:
        rospy.loginfo("Program interrupted by user")
    finally:
        if controller:
            controller.shutdown()