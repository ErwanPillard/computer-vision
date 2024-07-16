import argparse
import cv2
import numpy as np
from ultralytics import YOLO
import supervision as sv

arucoDict = cv2.aruco.DICT_4X4_1000
def main():
    cap = cv2.VideoCapture(0)

    model = YOLO("../Model/best.pt")
    #Type of annotator
    box_annotator = sv.BoundingBoxAnnotator()
    label_annotator = sv.LabelAnnotator()


    while True:
        ret, frame = cap.read()

        results = model(frame)[0]
        detections = sv.Detections.from_ultralytics(results)
        detections = detections[(detections.confidence > 0.5)]

        labels = [
            model.model.names[class_id]
            for class_id
            in detections.class_id
        ]

        annotated_frame = box_annotator.annotate(
            scene=frame, detections=detections)
        annotated_frame = label_annotator.annotate(
                annotated_frame,
                detections=detections,
                labels=labels
            )

        cv2.imshow("Yolov8", annotated_frame)

        if (cv2.waitKey(1) & 0xFF == ord('q')):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()