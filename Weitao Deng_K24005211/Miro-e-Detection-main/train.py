!pip install ultralytics roboflow opencv-python matplotlib

from roboflow import Roboflow
from ultralytics import YOLO
import matplotlib.pyplot as plt
import cv2 as cv
import os
import torch
print("Is CUDA available:", torch.cuda.is_available())
print("Current device:", torch.cuda.current_device() if torch.cuda.is_available() else "CPU")
print("Device name:", torch.cuda.get_device_name(0) if torch.cuda.is_available() else "No GPU")
print(torch.__version__)
print(torch.version.cuda)

rf = Roboflow(api_key="qe4ZLSMBGGD4XqNndXs2")
project = rf.workspace("puspendu-ai-vision-workspace").project("hand_gesture_detection-xdcpy")
version = project.version(1)
dataset = version.download("yolov8")

# Output path to confirm the real directory
print("✅ Dataset location:", dataset.location)

# Initialize the YOLO model
model = YOLO("yolov8l.pt")

# Absolute path to download using Roboflow
model.train(
    data=f"{dataset.location}/data.yaml",
    epochs=50,
    imgsz=640,
)