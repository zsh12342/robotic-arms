import rospy
import rosbag
import time
import subprocess
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

def get_cur_jointstate(topic_name="/joint_states"):
    joint_state = rospy.wait_for_message(topic_name, JointState)
    joint_state_position = list(joint_state.position)
    joint_state_position[0], joint_state_position[2] = joint_state_position[2], joint_state_position[0]
    return joint_state_position

class Replay:
    def __init__(self, pub_topic="/scaled_pos_joint_traj_controller/command"):
        self.joint = JointTrajectory()
        self.arm_pub = rospy.Publisher(pub_topic, JointTrajectory, queue_size=10)

    def reset_position(self, init_pos, optisions=None):
        cur_pos = get_cur_jointstate()
        delta_pos = [(init_pos[i] - cur_pos[i])/100  for i in range(6)]
        
        for i in range(100):
            position = list()
            cur_pos = get_cur_jointstate()

            for i in range(6):
                position.append(cur_pos[i] + delta_pos[i])

            self.joint.header.stamp = rospy.Time.now()
            self.joint.points.clear()
            goal = JointTrajectoryPoint(positions=position, time_from_start=rospy.Duration(0.3))
            self.joint.points.append(goal)
            print("self.joint.point:  ",  self.joint.points)
            self.arm_pub.publish(self.joint)
            time.sleep(0.02)

    def replay(self, bag_path):
            target_pose = None
            with rosbag.Bag(bag_path, 'r') as bag:
                 for topic, msg, t in bag.read_messages():
                      if topic == '/scaled_pos_joint_traj_controller/command':
                        self.joint.joint_names = msg.name
                        target_pose = list(msg.position)
                        self.reset_position(target_pose)
                        break
            
            with rosbag.Bag(bag_path, 'r') as bag:
                for topic, msg, t in bag.read_messages():
                    if topic == '/scaled_pos_joint_traj_controller/command':
                        msg.header.stamp = rospy.Time.now()
                        self.arm_pub.publish(msg)
                        time.sleep(0.05)

        
