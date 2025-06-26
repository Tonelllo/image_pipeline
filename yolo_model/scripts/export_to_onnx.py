from ultralytics import YOLO
# import onnx

# Load your custom-trained YOLOv8 model
model = YOLO("../models/manipulationStation.pt")  # e.g., 'runs/detect/train/weights/best.pt'

class_names = model.names
for class_id, name in class_names.items():
    print(f"{class_id}: {name}")
# Export to ONNX
model.fuse()
model.export(
    format="onnx",
    imgsz=(640, 640),       # Input shape (H, W)
    opset=17,               # Opset version (17 for YOLOv8)
    # simplify=True,          # Simplify ONNX graph (critical!)
    # dynamic=False,          # Set True for dynamic batch size
    # nms=False,               # Include NMS in the model (optional)
)
