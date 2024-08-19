#!/usr/bin/env python3
# -*- coding: UTF-8 -*-

import sys
import rospy
import moveit_commander
import math
import numpy as np
import tf
from geometry_msgs.msg import PoseStamped
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import time
from kuavoRobotSDK import kuavo

# 视觉系统所需的库
import pyrealsense2 as rs
import cv2
from openvino.runtime import Core
import onnxruntime as ort

# 只检测 'bottle' 类别
CLASSES = ['bottle']

class OpenvinoInference:
    def __init__(self, onnx_path):
        self.onnx_path = onnx_path
        ie = Core()
        self.model_onnx = ie.read_model(model=self.onnx_path)
        self.compiled_model_onnx = ie.compile_model(model=self.model_onnx, device_name="CPU")
        self.output_layer_onnx = self.compiled_model_onnx.output(0)

    def predict(self, datas):
        predict_data = self.compiled_model_onnx([datas])[self.output_layer_onnx]
        return predict_data

class YOLOv5:
    def __init__(self, onnx_model, imgsz=(640, 640), infer_tool='openvino'):
        self.infer_tool = infer_tool
        if self.infer_tool == 'openvino':
            self.openvino = OpenvinoInference(onnx_model)
            self.ndtype = np.single
        else:
            self.ort_session = ort.InferenceSession(onnx_model,
                                                    providers=['CUDAExecutionProvider', 'CPUExecutionProvider']
                                                    if ort.get_device() == 'GPU' else ['CPUExecutionProvider'])
            self.ndtype = np.half if self.ort_session.get_inputs()[0].type == 'tensor(float16)' else np.single
        
        self.classes = CLASSES
        self.model_height, self.model_width = imgsz[0], imgsz[1]
        self.color_palette = np.random.uniform(0, 255, size=(len(self.classes), 3))

    def __call__(self, img, conf_threshold=0.4, iou_threshold=0.45):
        im, ratio, (pad_w, pad_h) = self.preprocess(img)
        
        if self.infer_tool == 'openvino':
            preds = self.openvino.predict(im)
        else:
            preds = self.ort_session.run(None, {self.ort_session.get_inputs()[0].name: im})[0]

        boxes = self.postprocess(preds,
                                img0=img,
                                ratio=ratio,
                                pad_w=pad_w,
                                pad_h=pad_h,
                                conf_threshold=conf_threshold,
                                iou_threshold=iou_threshold)

        return boxes

    def preprocess(self, img):
        shape = img.shape[:2]
        new_shape = (self.model_height, self.model_width)
        r = min(new_shape[0] / shape[0], new_shape[1] / shape[1])
        ratio = r, r
        new_unpad = int(round(shape[1] * r)), int(round(shape[0] * r))
        pad_w, pad_h = (new_shape[1] - new_unpad[0]) / 2, (new_shape[0] - new_unpad[1]) / 2
        if shape[::-1] != new_unpad:
            img = cv2.resize(img, new_unpad, interpolation=cv2.INTER_LINEAR)
        top, bottom = int(round(pad_h - 0.1)), int(round(pad_h + 0.1))
        left, right = int(round(pad_w - 0.1)), int(round(pad_w + 0.1))
        img = cv2.copyMakeBorder(img, top, bottom, left, right, cv2.BORDER_CONSTANT, value=(114, 114, 114))

        img = np.ascontiguousarray(np.einsum('HWC->CHW', img)[::-1], dtype=self.ndtype) / 255.0
        img_process = img[None] if len(img.shape) == 3 else img
        return img_process, ratio, (pad_w, pad_h)

    def postprocess(self, preds, img0, ratio, pad_w, pad_h, conf_threshold, iou_threshold):
        x = preds
        # 只保留 bottle 类别（ID 39）的预测结果
        x = x[(x[..., 4] > conf_threshold) & (np.argmax(x[..., 5:], axis=-1) == 39)]
        if len(x) > 0:
            x = np.c_[x[..., :4], x[..., 4], np.full((x.shape[0], 1), 0)]  # 将类别ID设为0
            x = x[cv2.dnn.NMSBoxes(x[:, :4], x[:, 4], conf_threshold, iou_threshold)]
        
        if len(x) > 0:
            x[..., [0, 1]] -= x[..., [2, 3]] / 2
            x[..., [2, 3]] += x[..., [0, 1]]
            x[..., :4] -= [pad_w, pad_h, pad_w, pad_h]
            x[..., :4] /= min(ratio)
            x[..., [0, 2]] = x[:, [0, 2]].clip(0, img0.shape[1])
            x[..., [1, 3]] = x[:, [1, 3]].clip(0, img0.shape[0])

            return x[..., :6]
        else:
            return []

class Planner:
    def __init__(self):
        self.move_group = moveit_commander.MoveGroupCommander("arm")

    def plan_to_target_pose(self, target_pose):
        self.move_group.set_pose_target(target_pose)
        plan = self.move_group.plan()
        return plan

    def plan_to_target_joints(self, joint_values):
        self.move_group.set_joint_value_target(joint_values)
        plan = self.move_group.plan()
        return plan

class Logger:
    def log(self, message):
        rospy.loginfo(message)

    def dump_traj(self, traj, file_name):
        # 实现轨迹保存逻辑
        pass

class Publisher:
    def __init__(self):
        self.joint_pub = rospy.Publisher('/joint_states', JointState, queue_size=10)

    def publish_joint_state(self, joint_state):
        self.joint_pub.publish(joint_state)

class Executor:
    def __init__(self):
        self.move_group = moveit_commander.MoveGroupCommander("arm")

    def execute_traj(self, traj, wait=True):
        self.move_group.execute(traj, wait=wait)

def angle_to_rad(angles):
    return [math.radians(angle) for angle in angles]

class WaterBottleGraspingDemo:
    def __init__(self):
        rospy.init_node("water_catch_vision_demo_node", anonymous=True)
        moveit_commander.roscpp_initialize(sys.argv)

        self.planner = Planner()
        self.logger = Logger()
        self.publisher = Publisher()
        self.executor = Executor()

        self.joint_state = JointState()
        self.robot_instance = kuavo("4_1_kuavo")
        self.Y_TO_MOVEIT_OFFSET = 0

        # 初始化视觉系统
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        self.pipeline.start(config)

        # 初始化YOLOv5模型
        self.yolo_model = YOLOv5('/home/kuavo/yolov5s.onnx', infer_tool='openvino')

        # 定义预设的关节角度点
        self.Point_zero = angle_to_rad([0, 0, 0, 0, 0, 0, 0])
        self.Point_1 = angle_to_rad([ 20, 50, 0,   0, 10,   0, 0])
        self.Point_2 = angle_to_rad([ 30, 90, 0, -50, 90, -30, 0])
        self.Point_3 = angle_to_rad([-15, 90, 0, -50, 45, -40, 0])
        self.Point_4 = angle_to_rad([-50, 50, 0, -30,  0, -50, 0])
        self.Point_5 = angle_to_rad([-50,  0, 0, -30,  0, -50, 0])

        self.trajectory_counter = 0
        self.MAX_TRAJECTORY_COUNT = 3

        rospy.Subscriber("/robot_arm_q_v_tau", JointState, self.joint_callback)

    def get_objects(self):
        frames = self.pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        depth_frame = frames.get_depth_frame()

        if not color_frame or not depth_frame:
            return None, None, None

        color_image = np.asanyarray(color_frame.get_data())
        detections = self.yolo_model(color_image)

        if len(detections) > 0:
            x1, y1, x2, y2, conf, cls_id = detections[0]
            x_center = int((x1 + x2) / 2)
            y_center = int((y1 + y2) / 2)
            distance = depth_frame.get_distance(x_center, y_center)
            return x_center, y_center, distance
        else:
            return None, None, None

    def execute_trajectory(self, x, y, z):
        target_pose = PoseStamped()
        target_pose.header.frame_id = "base_link"
        target_pose.pose.position.x = x
        target_pose.pose.position.y = y + self.Y_TO_MOVEIT_OFFSET
        target_pose.pose.position.z = z
        target_pose.pose.orientation.x = -0.0005388071066334781
        target_pose.pose.orientation.y = -0.7904212674887817
        target_pose.pose.orientation.z = 0.00032694187655405566
        target_pose.pose.orientation.w = 0.6125633213777487

        traj = self.planner.plan_to_target_pose(target_pose)
        if traj:
            self.logger.dump_traj(traj, file_name=f"vision_traj_{self.trajectory_counter}")
            self.executor.execute_traj(traj, wait=True)
            self.trajectory_counter += 1
            return True
        return False

    def joint_callback(self, data):
        self.joint_state = data

    def grasp_object(self):
        # 实现灵巧手抓取逻辑
        self.robot_instance.set_end_control([65, 65, 90, 90, 90, 90], [65, 65, 90, 90, 90, 90])
        rospy.sleep(2)

    def release_object(self):
        # 实现释放物体的逻辑
        self.robot_instance.set_end_control([100, 0, 0, 0, 0, 0], [100, 0, 0, 0, 0, 0])
        rospy.sleep(2)

    def run_demo(self):
        print(" -----moveit规划器 + pythonAPI启动完毕，按下Enter键 开始进行视觉规划 -------------")
        input()

        # 执行预设轨迹
        self.execute_predefined_trajectory()

        while not rospy.is_shutdown() and self.trajectory_counter < self.MAX_TRAJECTORY_COUNT:
            x_center, y_center, distance = self.get_objects()
            if x_center is not None and y_center is not None and distance is not None:
                print(f"检测到目标：位置({x_center}, {y_center})，距离：{distance:.2f}米")
                if self.execute_trajectory(x_center, y_center, distance):
                    rospy.sleep(3)
                else:
                    print("规划失败，重试...")
            else:
                print("未检测到目标")
            rospy.sleep(1)

        if self.trajectory_counter >= self.MAX_TRAJECTORY_COUNT:
            self.grasp_object()
            self.return_to_home()
            self.release_object()

    def execute_predefined_trajectory(self):
        trajectory_points = [self.Point_1, self.Point_2, self.Point_3, self.Point_4, self.Point_5]
        for i, point in enumerate(trajectory_points):
            traj = self.planner.plan_to_target_joints(point)
            if traj:
                self.logger.dump_traj(traj, file_name=f"predefined_traj_{i}")
                self.executor.execute_traj(traj, wait=True)
                rospy.sleep(3)
            else:
                print(f"预设轨迹点 {i} 规划失败")

    def return_to_home(self):
        trajectory_points = [self.Point_4, self.Point_3, self.Point_2, self.Point_1, self.Point_zero]
        for i, point in enumerate(trajectory_points):
            traj = self.planner.plan_to_target_joints(point)
            if traj:
                self.logger.dump_traj(traj, file_name=f"return_traj_{i}")
                self.executor.execute_traj(traj, wait=True)
                rospy.sleep(3)
            else:
                print(f"返回轨迹点 {i} 规划失败")

if __name__ == "__main__":
    demo = WaterBottleGraspingDemo()
    demo.run_demo()