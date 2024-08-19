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

CLASSES = ['button', 'valve', 'handle']

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
        t1 = time.time()
        im, ratio, (pad_w, pad_h) = self.preprocess(img)
        print('Ô¤´¦ÀíÊ±¼ä£º{:.3f}s'.format(time.time() - t1))
        
        t2 = time.time()
        if self.infer_tool == 'openvino':
            preds = self.openvino.predict(im)
        else:
            preds = self.ort_session.run(None, {self.ort_session.get_inputs()[0].name: im})[0]
        print('ÍÆÀíÊ±¼ä£º{:.2f}s'.format(time.time() - t2))

        t3 = time.time()
        boxes = self.postprocess(preds,
                                img0=img,
                                ratio=ratio,
                                pad_w=pad_w,
                                pad_h=pad_h,
                                conf_threshold=conf_threshold,
                                iou_threshold=iou_threshold)
        print('ºó´¦ÀíÊ±¼ä£º{:.3f}s'.format(time.time() - t3))

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
       
        x = x[(x[..., 4] > conf_threshold) & (np.argmax(x[..., 5:], axis=-1) == 0)] 
        if len(x) > 0:
            x = np.c_[x[..., :4], x[..., 4], np.full((x.shape[0], 1), 0)] 
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

    def draw_and_visualize(self, im, bboxes, vis=False, save=False):
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

class DepthEstimator:
    def __init__(self, history_size=10):
        self.history = deque(maxlen=history_size)

    def estimate_depth(self, depth_frame, bbox, min_depth=0.1, max_depth=10.0):
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
        return smoothed_depth

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--model', type=str, default='/home/kuavo/task3.onnx', help='Path to ONNX model')
    parser.add_argument('--imgsz', type=tuple, default=(640, 640), help='Image input size')
    parser.add_argument('--conf_thres', type=float, default=0.25, help='Confidence threshold')
    parser.add_argument('--iou_thres', type=float, default=0.45, help='NMS IoU threshold')
    parser.add_argument('--infer_tool', type=str, default='openvino', choices=("openvino", "onnxruntime"), help='Ñ¡ÔñÍÆÀíÒýÇæ')
    parser.add_argument('--ros_topic', type=str, default='/button_detection_node', help='ROS topic to publish detection data')
    args = parser.parse_args()

    model = YOLOv5(args.model, args.imgsz, args.infer_tool)
    depth_estimator = DepthEstimator()

    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

    pipeline.start(config)

    # Align depth to color
    align_to = rs.stream.color
    align = rs.align(align_to)

    rospy.init_node('button_detection_node', anonymous=True)
    pub = rospy.Publisher(args.ros_topic, String, queue_size=10)

    try:
        while not rospy.is_shutdown():
            frames = pipeline.wait_for_frames()
            aligned_frames = align.process(frames)
            depth_frame = aligned_frames.get_depth_frame()
            color_frame = aligned_frames.get_color_frame()

            if not depth_frame or not color_frame:
                continue

            color_image = np.asanyarray(color_frame.get_data())

            detections = model(color_image, conf_threshold=args.conf_thres, iou_threshold=args.iou_thres)

            for det in detections:
                bbox = det[:4]
                conf = det[4]
                cls_ = det[5]
                
                depth = depth_estimator.estimate_depth(depth_frame, bbox)
                
                if depth is not None:
                    detection_result = f"Class: button, Confidence: {conf:.2f}, Depth: {depth:.3f} meters"
                    print(detection_result)
                    pub.publish(detection_result)
                else:
                    print("Unable to estimate depth for this detection")

            model.draw_and_visualize(color_image, detections, vis=True, save=False)

    finally:
        pipeline.stop()

if __name__ == '__main__':
    main()