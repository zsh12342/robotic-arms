import sys
import os

# 添加当前目录到Python路径
current_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.append(current_dir)

import torch
from models.experimental import attempt_load

# 加载YOLOv5模型
try:
    # 尝试不使用 map_location 参数
    model = attempt_load('/home/kuavo/yolov5s.pt')
    model = model.to('cpu')  # 将模型移到CPU
    model.eval()
except TypeError:
    try:
        # 如果上面失败，尝试使用 device 参数
        model = attempt_load('/home/kuavo/yolov5s.pt', device='cpu')
        model.eval()
    except Exception as e:
        print(f"Error loading model: {e}")
        sys.exit(1)

# 准备一个示例输入
dummy_input = torch.randn(1, 3, 640, 640)

# 导出ONNX模型
try:
    torch.onnx.export(model, dummy_input, 'yolov5s.onnx', opset_version=11)
    print("Model successfully exported to yolov5s.onnx")
except Exception as e:
    print(f"Error exporting model to ONNX: {e}")
    sys.exit(1)