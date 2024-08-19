import pyrealsense2 as rs
import numpy as np
import cv2
import argparse
import time
from openvino.runtime import Core
import onnxruntime as ort
import rospy
from std_msgs.msg import String
from collections import deque

# 定义检测类别，包含按钮、阀门和把手
CLASSES = ['button', 'valve', 'handle']

# 模型推理类，支持OpenVINO和ONNXRuntime
class ModelInference:
    def __init__(self, model_path, infer_tool='openvino'):
        self.infer_tool = infer_tool
        self.model_path = model_path
        
        # 根据指定的推理工具加载模型
        if infer_tool == 'openvino':
            ie = Core()
            # 读取和编译OpenVINO模型
            self.model = ie.read_model(model=self.model_path)
            self.compiled_model = ie.compile_model(model=self.model, device_name="CPU")
            self.output_layer = self.compiled_model.output(0)
            self.ndtype = np.single  # 指定浮点数数据类型
        else:
            # 加载ONNX模型
            self.ort_session = ort.InferenceSession(self.model_path)
            self.ndtype = np.half if self.ort_session.get_inputs()[0].type == 'tensor(float16)' else np.single

    def predict(self, data):
        # 进行模型推理，返回预测结果
        if self.infer_tool == 'openvino':
            return self.compiled_model([data])[self.output_layer]
        else:
            return self.ort_session.run(None, {self.ort_session.get_inputs()[0].name: data})[0]

# YOLOv5目标检测类
class YOLOv5:
    def __init__(self, model_inference, imgsz=(640, 640)):
        self.model_inference = model_inference
        self.imgsz = imgsz
        self.classes = CLASSES
        # 为每个类别生成不同的颜色用于标注
        self.color_palette = np.random.uniform(0, 255, size=(len(self.classes), 3))

    def preprocess(self, img):
        # 预处理输入图像，包括缩放和填充
        shape = img.shape[:2]
        new_shape = self.imgsz
        r = min(new_shape[0] / shape[0], new_shape[1] / shape[1])
        ratio = r, r
        new_unpad = int(round(shape[1] * r)), int(round(shape[0] * r))
        pad_w, pad_h = (new_shape[1] - new_unpad[0]) / 2, (new_shape[0] - new_unpad[1]) / 2
        if shape[::-1] != new_unpad:
            img = cv2.resize(img, new_unpad, interpolation=cv2.INTER_LINEAR)
        top, bottom = int(round(pad_h - 0.1)), int(round(pad_h + 0.1))
        left, right = int(round(pad_w - 0.1)), int(round(pad_w + 0.1))
        img = cv2.copyMakeBorder(img, top, bottom, left, right, cv2.BORDER_CONSTANT, value=(114, 114, 114))

        # 转换颜色通道顺序并归一化
        img = np.ascontiguousarray(np.einsum('HWC->CHW', img)[::-1], dtype=self.model_inference.ndtype) / 255.0
        img_process = img[None] if len(img.shape) == 3 else img
        return img_process, ratio, (pad_w, pad_h)

    def postprocess(self, preds, img0, ratio, pad_w, pad_h, conf_threshold, iou_threshold):
        # 后处理模型预测结果，仅保留目标类别为"valve"的检测结果
        x = preds[(preds[..., 4] > conf_threshold) & (np.argmax(preds[..., 5:], axis=-1) == 1)]
        if len(x) > 0:
            x = np.c_[x[..., :4], x[..., 4], np.full((x.shape[0], 1), 1)]
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

    def __call__(self, img, conf_threshold=0.4, iou_threshold=0.45):
        # 完整的图像处理流程：预处理、模型推理、后处理
        im, ratio, (pad_w, pad_h) = self.preprocess(img)
        preds = self.model_inference.predict(im)
        boxes = self.postprocess(preds, img, ratio, pad_w, pad_h, conf_threshold, iou_threshold)
        return boxes

    def draw_and_visualize(self, im, bboxes, vis=False, save=False):
        # 绘制检测结果，并选择是否可视化或保存
        for (*box, conf, cls_) in bboxes:
            cv2.rectangle(im, (int(box[0]), int(box[1])), (int(box[2]), int(box[3])),
                          self.color_palette[int(cls_)], 1, cv2.LINE_AA)
            cv2.putText(im, f'{self.classes[int(cls_)]}: {conf:.3f}', (int(box[0]), int(box[1] - 9)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, self.color_palette[int(cls_)], 2, cv2.LINE_AA)
        
        if vis:
            cv2.imshow('demo', im)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                return

        if save:
            cv2.imwrite('output.jpg', im)

# 深度估计类，用于估计物体距离
class DepthEstimator:
    def __init__(self, history_size=10):
        # 用于存储最近的深度数据以进行平滑
        self.history = deque(maxlen=history_size)

    def estimate_depth(self, depth_frame, bbox, min_depth=0.1, max_depth=2.0):
        x_min, y_min, x_max, y_max = map(int, bbox[:4])
        width, height = x_max - x_min, y_max - y_min
        
        sample_step = max(1, min(width, height) // 20)
        depths = []
        for y in range(y_min, y_max, sample_step):
            for x in range(x_min, x_max, sample_step):
                depth = depth_frame.get_distance(x, y)
                if min_depth < depth < max_depth:
                    depths.append(depth)
        
        if not depths:
            return None

        # 排除异常值
        q1, q3 = np.percentile(depths, [25, 75])
        iqr = q3 - q1
        lower_bound = q1 - 1.5 * iqr
        upper_bound = q3 + 1.5 * iqr
        filtered_depths = [d for d in depths if lower_bound <= d <= upper_bound]

        if not filtered_depths:
            return None

        current_depth = np.median(filtered_depths)
        self.history.append(current_depth)

        smoothed_depth = np.median(self.history)
        return smoothed_depth if smoothed_depth <= max_depth else None

def main():
    # 定义命令行参数
    parser = argparse.ArgumentParser()
    parser.add_argument('--model', type=str, default='/home/kuavo/task3.onnx', help='ONNX模型路径')
    parser.add_argument('--imgsz', type=tuple, default=(640, 640), help='输入图像尺寸')
    parser.add_argument('--conf_thres', type=float, default=0.25, help='置信度阈值')
    parser.add_argument('--iou_thres', type=float, default=0.45, help='NMS IoU阈值')
    parser.add_argument('--infer_tool', type=str, default='openvino', choices=("openvino", "onnxruntime"), help='推理工具选择')
    parser.add_argument('--ros_topic', type=str, default='/valve_detection_node', help='ROS话题名称')
    args = parser.parse_args()

    # 实例化模型推理类、目标检测类和深度估计类
    model_inference = ModelInference(args.model, args.infer_tool)
    model = YOLOv5(model_inference, args.imgsz)
    depth_estimator = DepthEstimator()

    # 配置并启动RealSense摄像头管道
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    pipeline.start(config)

    # 对齐深度数据到彩色图像
    align_to = rs.stream.color
    align = rs.align(align_to)

    # 初始化ROS节点
    rospy.init_node('valve_detection_node', anonymous=True)
    pub = rospy.Publisher(args.ros_topic, String, queue_size=10)

    try:
        while not rospy.is_shutdown():
            # 获取对齐后的帧数据
            frames = pipeline.wait_for_frames()
            aligned_frames = align.process(frames)
            depth_frame = aligned_frames.get_depth_frame()
            color_frame = aligned_frames.get_color_frame()

            if not depth_frame or not color_frame:
                continue

            color_image = np.asanyarray(color_frame.get_data())

            # 进行目标检测
            detections = model(color_image, conf_threshold=args.conf_thres, iou_threshold=args.iou_thres)

            for det in detections:
                bbox = det[:4]
                conf = det[4]
                cls_ = det[5]
                
                # 估计深度
                depth = depth_estimator.estimate_depth(depth_frame, bbox)
                
                if depth is not None:
                    detection_result = f"Class: valve, Confidence: {conf:.2f}, Depth: {depth:.3f} meters"
                    print(detection_result)
                    pub.publish(detection_result)
                else:
                    print("Detection is too far or depth estimation failed")

            # 可视化检测结果
            model.draw_and_visualize(color_image, detections, vis=True, save=False)

    finally:
        pipeline.stop()

if __name__ == '__main__':
    main()
