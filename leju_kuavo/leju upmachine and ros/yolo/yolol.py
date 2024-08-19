import pyrealsense2 as rs
import numpy as np
import cv2
import onnxruntime

# ÅäÖÃÉî¶ÈºÍÑÕÉ«Á÷
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

# Æô¶¯Á÷
pipeline.start(config)

# ¼ÓÔØ ONNX Ä£ÐÍ
model_path = 'kuavo_ros_application/src/ros_vision/detection_yolo/kuavo_vision_object/scripts/models/yolov5s-seg.onnx'  # Ìæ»»ÎªÄãµÄ ONNX Ä£ÐÍÂ·¾¶
session = onnxruntime.InferenceSession(model_path)

# ×¼±¸ÊäÈë
def preprocess_image(image):
    image_resized = cv2.resize(image, (640, 640))  # ¸ü¸ÄÎªÄ£ÐÍµÄÊäÈë´óÐ¡
    image_tensor = image_resized.astype(np.float32) / 255.0
    image_tensor = np.transpose(image_tensor, (2, 0, 1))  # HWC to CHW
    image_tensor = np.expand_dims(image_tensor, axis=0)  # Ôö¼ÓÅú´ÎÎ¬¶È
    return image_tensor

# ½øÐÐÄ¿±ê¼ì²â
def detect_objects(image):
    image_tensor = preprocess_image(image)
    inputs = {session.get_inputs()[0].name: image_tensor}
    outputs = session.run(None, inputs)
    detections = outputs[0]
    
    # ºó´¦ÀíµÃµ½¼ì²â½á¹û
    boxes = detections[0][:, :4]  # ¼ÙÉèÄ£ÐÍÊä³öµÄ¸ñÊ½ÊÇ [batch, num_boxes, 6]
    scores = detections[0][:, 4]
    classes = detections[0][:, 5]
    
    return boxes, scores, classes

try:
    while True:
        # µÈ´ýÒ»×éÁ¬¹áµÄÖ¡£ºÉî¶ÈºÍÑÕÉ«
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        if not color_frame:
            continue

        # ½«Í¼Ïñ×ª»»ÎªnumpyÊý×é
        color_image = np.asanyarray(color_frame.get_data())

        # ½øÐÐÄ¿±ê¼ì²â
        boxes, scores, classes = detect_objects(color_image)

        # ÔÚ²ÊÉ«Í¼ÏñÉÏ»æÖÆ¼ì²â¿ò
        for i, box in enumerate(boxes):
            if scores[i] > 0.5:  # Éè¶¨Ò»¸öãÐÖµÀ´É¸Ñ¡¼ì²â½á¹û
                x_min, y_min, x_max, y_max = map(int, box)
                cv2.rectangle(color_image, (x_min, y_min), (x_max, y_max), (0, 255, 0), 2)
                cv2.putText(color_image, f"Score: {scores[i]:.2f}", (x_min, y_min - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        # ÏÔÊ¾Í¼Ïñ
        cv2.imshow('RealSense with Detection', color_image)

        # °´'q'¼üÍË³öÑ­»·
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    # Í£Ö¹Á÷
    pipeline.stop()
    cv2.destroyAllWindows()
