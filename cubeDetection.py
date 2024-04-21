import argparse
import cv2
import numpy as np
from ultralytics import YOLO
import supervision as sv

# ArUco Marker ID: 1, Corner: [150. 638.]
# ArUco Marker ID: 3, Corner: [1065.  637.]
# ArUco Marker ID: 2, Corner: [195.  57.]
# ArUco Marker ID: 4, Corner: [1023.   51.]

# Camera calibration parameters
intrinsic_camera = np.array(((1281.57894, 0, 457.638346), (0, 1262.76271, 260.388263), (0, 0, 1)))
distortion = np.array((0.12431658, -0.55314019, 0, 0, 0))

arucoDict = cv2.aruco.DICT_4X4_1000

# Charger les données du fichier texte dans ZONE_POLYGON
ZONE_POLYGON = np.loadtxt('zone_polygon.txt', dtype=np.int32)


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

def main():
    #upgrade video quality
    args = parse_arguments()
    frame_width, frame_height = args.webcam_resolution

    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, frame_width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, frame_height)

    model = YOLO("best.pt")
    tracker = sv.ByteTrack()

    #type of annotator
    box_annotator = sv.BoundingBoxAnnotator()
    color_annotator = sv.ColorAnnotator()

    label_annotator = sv.LabelAnnotator()

    zone = sv.PolygonZone(polygon=ZONE_POLYGON, frame_resolution_wh=tuple(args.webcam_resolution))
    zone_annotator = sv.PolygonZoneAnnotator(zone=zone, color=sv.Color.BLUE)

    while True:
        ret, frame = cap.read()

        results = model(frame)[0]
        detections = sv.Detections.from_ultralytics(results)
        detections = detections[detections.confidence > 0.85]
        detections = tracker.update_with_detections(detections)

        labels = [
            f"#{tracker_id} {results.names[class_id]}"
            for class_id, tracker_id
            in zip(detections.class_id, detections.tracker_id)
        ]

        annotated_frame = color_annotator.annotate(
            scene=frame,
            detections=detections,
        )

        frame = label_annotator.annotate(
            annotated_frame,
            detections=detections,
            labels=labels
        )

        # labels = [
        #     model.model.names[class_id]
        #     for class_id
        #     in detections.class_id
        # ]
        #
        # annotated_image = box_annotator.annotate(
        #     scene=frame, detections=detections)
        # annotated_image = label_annotator.annotate(
        #     scene=annotated_image, detections=detections, labels=labels)

        zone.trigger(detections=detections)
        frame = zone_annotator.annotate(scene=frame)

        cv2.imshow('yolov8', frame)

        #get cam resolution
        #print(frame.shape)
        #break

        if (cv2.waitKey(1) & 0xFF == ord('q')):
            break



if __name__ == "__main__":
    main()
