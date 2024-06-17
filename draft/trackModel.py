import os
os.environ['KMP_DUPLICATE_LIB_OK'] = 'True'

from ultralytics import YOLO

model = YOLO("yolov8n")

results = model.track(source=0, show=True) #tracker="bytetrack.yaml")