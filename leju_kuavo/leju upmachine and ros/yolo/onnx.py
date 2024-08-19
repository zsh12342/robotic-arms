import argparse
import time
import cv2
import numpy as np
from openvino.runtime import Core  # pip install openvino -i https://pypi.tuna.tsinghua.edu.cn/simple
import onnxruntime as ort  # pip install onnxruntime£¬Ä¬ÈÏ°²×°CPU


# COCOÄ¬ÈÏµÄ80Àà
CLASSES = ['person', 'bicycle', 'car', 'motorcycle', 'airplane', 'bus', 'train', 'truck', 'boat', 'traffic light',
           'fire hydrant', 'stop sign', 'parking meter', 'bench', 'bird', 'cat', 'dog', 'horse', 'sheep', 'cow',
           'elephant', 'bear', 'zebra', 'giraffe', 'backpack', 'umbrella', 'handbag', 'tie', 'suitcase', 'frisbee',
           'skis', 'snowboard', 'sports ball', 'kite', 'baseball bat', 'baseball glove', 'skateboard', 'surfboard',
           'tennis racket', 'bottle', 'wine glass', 'cup', 'fork', 'knife', 'spoon', 'bowl', 'banana', 'apple', 'sandwich',
           'orange', 'broccoli', 'carrot', 'hot dog', 'pizza', 'donut', 'cake', 'chair', 'couch', 'potted plant', 'bed',
           'dining table', 'toilet', 'tv', 'laptop', 'mouse', 'remote', 'keyboard', 'cell phone', 'microwave', 'oven',
           'toaster', 'sink', 'refrigerator', 'book', 'clock', 'vase', 'scissors', 'teddy bear', 'hair drier', 'toothbrush']


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
        x = x[x[..., 4] > conf_threshold]
        x = np.c_[x[..., :4], x[..., 4], np.argmax(x[..., 5:], axis=-1)]
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
            cv2.waitKey(0)
            cv2.destroyAllWindows()

        if save:
            cv2.imwrite('output.jpg', im)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--model', type=str, default='/home/kuavo/yolov5s.onnx', help='Path to ONNX model')
    parser.add_argument('--imgsz', type=tuple, default=(640, 640), help='Image input size')
    parser.add_argument('--conf_thres', type=float, default=0.25, help='Confidence threshold')
    parser.add_argument('--iou_thres', type=float, default=0.45, help='NMS IoU threshold')
    parser.add_argument('--infer_tool', type=str, default='openvino', choices=("openvino", "onnxruntime"), help='Ñ¡ÔñÍÆÀíÒýÇæ')
    args = parser.parse_args()

    model = YOLOv5(args.model, args.imgsz, args.infer_tool)

    cap = cv2.VideoCapture(4)  # Ê¹ÓÃÄ¬ÈÏÉãÏñÍ·
    
    while True:
        ret, frame = cap.read()
        if not ret:
            print("ÎÞ·¨»ñÈ¡ÊÓÆµÖ¡")
            break
        
        # ÍÆÀí
        boxes = model(frame, conf_threshold=args.conf_thres, iou_threshold=args.iou_thres)

        # ÔÚframeÉÏ»æÖÆ¼ì²â½á¹û
        if len(boxes) > 0:
            for (*box, conf, cls_) in boxes:
                cv2.rectangle(frame, (int(box[0]), int(box[1])), (int(box[2]), int(box[3])),
                              model.color_palette[int(cls_)], 2)
                label = f'{model.classes[int(cls_)]}: {conf:.2f}'
                cv2.putText(frame, label, (int(box[0]), int(box[1] - 10)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, model.color_palette[int(cls_)], 2)

        # ÏÔÊ¾ÊµÊ±ÊÓÆµÁ÷
        cv2.imshow('YOLOv5 Detection', frame)
        
        # °´ 'q' ¼üÍË³ö
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
        
        

   
