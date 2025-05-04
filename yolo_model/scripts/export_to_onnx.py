from ultralytics import YOLO
import onnx

# Load your custom-trained YOLOv8 model
model = YOLO("../models/best.pt")  # e.g., 'runs/detect/train/weights/best.pt'

# Export to ONNX
model.export(
    format="onnx",
    imgsz=(640, 640),       # Input shape (H, W)
    opset=17,               # Opset version (17 for YOLOv8)
    simplify=True,          # Simplify ONNX graph (critical!)
    dynamic=False,          # Set True for dynamic batch size
    nms=False,               # Include NMS in the model (optional)
)
