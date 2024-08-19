#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from task5.msg import TargetInfo
import pyrealsense2 as rs
import numpy as np
import cv2
from your_yolov5_module import YOLOv5

class VisionNode:
    def __init__(self):
        rospy.init_node('vision_node')
        self.bridge = CvBridge()
        self.target_pub = rospy.Publisher('/target_info', TargetInfo, queue_size=10)
        self.model = YOLOv5('path_to_your_model.onnx', (640, 640), 'openvino')
        self.setup_camera()
        self.target_classes = ['button', 'valve', 'doorknob']

    def setup_camera(self):
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        self.pipeline.start(config)

    def get_frames(self):
        frames = self.pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        depth_frame = frames.get_depth_frame()
        color_image = np.asanyarray(color_frame.get_data())
        return color_image, depth_frame

    def detect_objects(self, image):
        boxes = self.model(image, conf_threshold=0.25, iou_threshold=0.45)
        return [box for box in boxes if self.model.classes[int(box[5])] in self.target_classes]

    def get_3d_position(self, depth_frame, box):
        x1, y1, x2, y2, _, cls = map(int, box)
        center_x = (x1 + x2) // 2
        center_y = (y1 + y2) // 2
        depth = depth_frame.get_distance(center_x, center_y)
        camera_intrinsics = depth_frame.profile.as_video_stream_profile().intrinsics
        point_3d = rs.rs2_deproject_pixel_to_point(camera_intrinsics, [center_x, center_y], depth)
        return point_3d, (center_x, center_y)

    def run(self):
        while not rospy.is_shutdown():
            color_image, depth_frame = self.get_frames()
            detected_objects = self.detect_objects(color_image)

            if detected_objects:
                target = detected_objects[0]  # ¼ÙÉèÎÒÃÇÖ»¹Ø×¢µÚÒ»¸ö¼ì²âµ½µÄÄ¿±ê
                point_3d, (center_x, center_y) = self.get_3d_position(depth_frame, target)
                target_class = self.model.classes[int(target[5])]

                # ¼ì²éÄ¿±êÊÇ·ñÔÚ»­ÃæÖÐÐÄ
                image_center_x, image_center_y = color_image.shape[1] // 2, color_image.shape[0] // 2
                is_centered = abs(center_x - image_center_x) < 20 and abs(center_y - image_center_y) < 20

                target_info = TargetInfo()
                target_info.x = point_3d[0]
                target_info.y = point_3d[1]
                target_info.z = point_3d[2]
                target_info.target_class = target_class
                target_info.is_centered = is_centered
                self.target_pub.publish(target_info)

            rospy.Rate(30).sleep()

if __name__ == "__main__":
    try:
        node = VisionNode()
        node.run()
    except rospy.ROSInterruptException:
        pass