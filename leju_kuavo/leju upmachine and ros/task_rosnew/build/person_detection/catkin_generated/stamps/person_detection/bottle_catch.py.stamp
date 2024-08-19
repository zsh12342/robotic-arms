#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
import pyrealsense2 as rs
from std_msgs.msg import String
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
from openvino.runtime import Core
import onnxruntime as ort
from moveit_commander import MoveGroupCommander
import tf2_ros
import tf2_geometry_msgs
from tf.transformations import quaternion_matrix, quaternion_from_matrix

CLASSES = ['bottle']

class Transformer(object):
    _instance = None
    def __new__(cls, *args, **kwargs):
        if not cls._instance:
            cls._instance = super().__new__(cls, *args, **kwargs)
        return cls._instance
    
    def __init__(self) -> None:
        pass
    
    def matrix_transform(self, matrix_1: list, matrix_2: list) -> list:
        return matrix_1 @ matrix_2
    
    def pose_to_matrix(self, pose: Pose) -> list:
        matrix = quaternion_matrix([
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w
        ])
        matrix[:-1, -1] = [
            pose.position.x,
            pose.position.y,
            pose.position.z
        ]
        return matrix
    
    def matrix_to_pose(self, matrix: list) -> Pose:
        pose = Pose()
        quaternion = quaternion_from_matrix(matrix)
        pose.orientation.x = quaternion[0]
        pose.orientation.y = quaternion[1]
        pose.orientation.z = quaternion[2]
        pose.orientation.w = quaternion[3]
        pose.position.x = matrix[0, -1]
        pose.position.y = matrix[1, -1]
        pose.position.z = matrix[2, -1]
        return pose

class ArmIk:
    def __init__(self, group_name="l_arm_group"):
        self.group = MoveGroupCommander(group_name)
        
        # ÉèÖÃ¹æ»®Æ÷
        self.group.set_planner_id("RRTstar")
        self.group.set_num_planning_attempts(3)
        self.group.set_planning_time(0.2)
        
        # ÉèÖÃËÙ¶ÈºÍ¼ÓËÙ¶ÈËõ·ÅÒò×Ó
        self.group.set_max_velocity_scaling_factor(1)
        self.group.set_max_acceleration_scaling_factor(1)
        
        # ÉèÖÃÈÝ²î
        self.group.set_goal_joint_tolerance(0.1)
        self.group.set_goal_orientation_tolerance(0.1)
        self.group.set_goal_position_tolerance(0.1)

    def computeIK(self, target_pose):
        plan = self.group.plan(target_pose)
        if plan.joint_trajectory.points:
            return plan.joint_trajectory.points[-1].positions
        else:
            return None

    def current_joint_values(self):
        return self.group.get_current_joint_values()

class YOLOv5:
    def __init__(self, onnx_model, imgsz=(640, 640), infer_tool='openvino'):
        # YOLOv5 ³õÊ¼»¯´úÂë£¬±£³Ö²»±ä
        pass

    def __call__(self, img, conf_threshold=0.4, iou_threshold=0.45):
        # YOLOv5 µ÷ÓÃ´úÂë£¬±£³Ö²»±ä
        pass

    def preprocess(self, img):
        # Ô¤´¦Àí´úÂë£¬±£³Ö²»±ä
        pass

    def postprocess(self, preds, img0, ratio, pad_w, pad_h, conf_threshold, iou_threshold):
        # ºó´¦Àí´úÂë£¬±£³Ö²»±ä
        pass

class BottleDetectionAndGrasping:
    def __init__(self):
        super().__init__()
        rospy.init_node('bottle_detection_and_grasping')
        
        # ³õÊ¼»¯RealSenseÏà»ú
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        self.pipeline.start(config)

        # ³õÊ¼»¯YOLOv5Ä£ÐÍ
        self.model = YOLOv5('/home/kuavo/yolov5s.onnx', (640, 640), 'openvino')

        # ROS·¢²¼Õß
        self.detection_pub = rospy.Publisher('/bottle_detection', String, queue_size=10)
        self.arm_trajectory_pub = rospy.Publisher('/arm_trajectory', JointState, queue_size=10)

        # ³õÊ¼»¯ArmIkÀà
        self.arm_ik = ArmIk("l_arm_group")
        self.planning_frame = "base_link"
        self.gripper_name = "l_hand_eff"

        # ³õÊ¼»¯Transformer
        self.transformer = Transformer()

        # ³õÊ¼»¯TF2
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

    def move_gripper(self, open_gripper=True):
        if open_gripper:
            gripper_position = [0.01, 0.01]  # ´ò¿ª×¥È¡Æ÷µÄÎ»ÖÃ
        else:
            gripper_position = [0, 0]  # ¹Ø±Õ×¥È¡Æ÷µÄÎ»ÖÃ

        self.set_grasp_gripper_pose(pre_position=gripper_position, position=gripper_position)
    
    def detect_and_grasp(self):
        while not rospy.is_shutdown():
            # »ñÈ¡Ïà»úÖ¡²¢½øÐÐÄ¿±ê¼ì²â
            frames = self.pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            depth_frame = frames.get_depth_frame()
            if not color_frame or not depth_frame:
                continue

            # ´¦ÀíÍ¼Ïñ
            color_image = np.asanyarray(color_frame.get_data())
            depth_image = np.asanyarray(depth_frame.get_data())

            # ¼ì²âË®Æ¿
            detections = self.model(color_image, conf_threshold=0.25, iou_threshold=0.45)

            if len(detections) > 0:
                detected_bottle = True
                # »ñÈ¡µÚÒ»¸ö¼ì²âµ½µÄË®Æ¿
                bottle = detections[0]
                x_center = int((bottle[0] + bottle[2]) / 2)
                y_center = int((bottle[1] + bottle[3]) / 2)

                # »ñÈ¡Éî¶ÈÐÅÏ¢
                depth = depth_frame.get_distance(x_center, y_center)

                # ·¢²¼¼ì²â½á¹û
                detection_msg = f"Bottle detected at ({x_center}, {y_center}) with depth {depth}m"
                self.detection_pub.publish(detection_msg)

                # ¼ÆËã3DÎ»ÖÃ£¨Ïà»ú×ø±êÏµ£©
                camera_intrinsics = depth_frame.profile.as_video_stream_profile().intrinsics
                bottle_3d = rs.rs2_deproject_pixel_to_point(camera_intrinsics, [x_center, y_center], depth)

                # ×ª»»µ½»úÆ÷ÈË»ù×ù±êÏµ
                bottle_position = self.transform_to_base(bottle_3d)
                if bottle_position is None:
                    rospy.logwarn("Failed to transform bottle position to base frame")
                    continue

                # ¼ÆËã×¥È¡×ËÌ¬
                grasp_pose = self.calculate_grasp_pose(bottle_position)

                # Ê¹ÓÃArmIk¼ÆËãÄæÔË¶¯Ñ§
                joint_values = self.arm_ik.computeIK(grasp_pose)

                if joint_values is not None:
                    # ·¢²¼¹Ø½Ú¹ì¼£
                    trajectory = JointState()
                    trajectory.position = joint_values
                    self.arm_trajectory_pub.publish(trajectory)
                else:
                    rospy.logwarn("Failed to find IK solution")

    def get_camera_to_base_transform(self):
        try:
            transform = self.tf_buffer.lookup_transform("base_link", "camera_link", rospy.Time(0))
            return transform
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logerr(f"Failed to get transform: {e}")
            return None

    def transform_to_base(self, point_in_camera):
        camera_to_base_transform = self.get_camera_to_base_transform()
        if camera_to_base_transform is None:
            return None

        point_stamped = tf2_geometry_msgs.PointStamped()
        point_stamped.header.frame_id = "camera_link"
        point_stamped.point.x = point_in_camera[0]
        point_stamped.point.y = point_in_camera[1]
        point_stamped.point.z = point_in_camera[2]

        try:
            point_base = tf2_geometry_msgs.do_transform_point(point_stamped, camera_to_base_transform)
            return [point_base.point.x, point_base.point.y, point_base.point.z]
        except Exception as e:
            rospy.logerr(f"Transform failed: {e}")
            return None

    def calculate_grasp_pose(self, bottle_position):
        grasp_pose = Pose()
        grasp_pose.position.x = bottle_position[0]
        grasp_pose.position.y = bottle_position[1]
        grasp_pose.position.z = bottle_position[2]
        grasp_pose.orientation.w = 1.0  # Ê¹ÓÃÄ¬ÈÏµÄËÄÔªÊý
        return grasp_pose

if __name__ == '__main__':
    try:
        detection_and_grasping = BottleDetectionAndGrasping()
        detection_and_grasping.detect_and_grasp()
    except rospy.ROSInterruptException:
        pass
