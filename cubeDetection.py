import argparse
import cv2
import numpy as np
from ultralytics import YOLO
import supervision as sv

# Camera calibration parameters
intrinsic_camera = np.array(((1281.57894, 0, 457.638346), (0, 1262.76271, 260.388263), (0, 0, 1)))
distortion = np.array((0.12431658, -0.55314019, 0, 0, 0))

arucoDict = cv2.aruco.DICT_4X4_1000

def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Yolov8 live")
    parser.add_argument(
        "--webcam-resolution",
        default=[1280, 720],
        nargs=2,
        type=int
    )
    args = parser.parse_args()
    return args

def get_object_center(frame, results):
    if len(results.boxes.xyxy) > 0:
        x1, y1, x2, y2 = results.boxes.xyxy[0][:4]
        center_x = int((x1 + x2) / 2)
        center_y = int((y1 + y2) / 2)
        center_coordinates = (center_x, center_y)
        cv2.circle(frame, center_coordinates, radius=5, color=(0, 255, 0), thickness=-1)
        return center_coordinates
    else:
        return None

def setup_camera():
    args = parse_arguments()
    frame_width, frame_height = args.webcam_resolution

    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, frame_width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, frame_height)

    return cap, args
def main(ZONE_POLYGON):
    cap, args = setup_camera()

    model = YOLO("best.pt")
    tracker = sv.ByteTrack()

    #Type of annotator
    box_annotator = sv.BoundingBoxAnnotator()
    label_annotator = sv.LabelAnnotator()

    zone = sv.PolygonZone(polygon=ZONE_POLYGON, frame_resolution_wh=tuple(args.webcam_resolution))
    zone_annotator = sv.PolygonZoneAnnotator(zone=zone, color=sv.Color.WHITE)

    while True:
        ret, frame = cap.read()

        results = model(frame)[0]
        detections = sv.Detections.from_ultralytics(results)
        mask = zone.trigger(detections=detections)
        detections = detections[(detections.confidence > 0.5) & mask]
        detections = tracker.update_with_detections(detections)

        labels = [
            f"#{tracker_id} {results.names[class_id]}"
            for class_id, tracker_id
            in zip(detections.class_id, detections.tracker_id)
        ]

        annotated_frame = box_annotator.annotate(
            scene=frame,
            detections=detections,
        )

        annotated_frame = label_annotator.annotate(
            annotated_frame,
            detections=detections,
            labels=labels
        )

        center_coordinates = get_object_center(frame, results)

        zone.trigger(detections=detections)
        frame = zone_annotator.annotate(scene=annotated_frame)

        cv2.imshow("Yolov8", frame)

        if (cv2.waitKey(1) & 0xFF == ord('q')):
            break



if __name__ == "__main__":
    main()
