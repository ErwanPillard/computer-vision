import argparse
import cv2
import numpy as np
from ultralytics import YOLO
import supervision as sv

# Camera calibration parameters
intrinsic_camera = np.array(((1281.57894, 0, 457.638346), (0, 1262.76271, 260.388263), (0, 0, 1)))
distortion = np.array((0.12431658, -0.55314019, 0, 0, 0))

arucoDict = cv2.aruco.DICT_4X4_1000



def get_cube_center(results):
    for i in range(len(results.boxes.xyxy)):
        if results.names[int(results.boxes.cls[i])] == "cube":
            x1, y1, x2, y2 = results.boxes.xyxy[i][:4]
            center_x = int((x1 + x2) / 2)
            center_y = int((y1 + y2) / 2)
            return (center_x, center_y)
    return None

def cube_detection_inZone(ZONE_POLYGON, frame):

    model = YOLO("../Model/best_2.pt")
    tracker = sv.ByteTrack()

    # Type of annotator
    box_annotator = sv.BoundingBoxAnnotator()
    label_annotator = sv.LabelAnnotator()

    # Resolution of the frame
    frame_resolution = (frame.shape[1], frame.shape[0])
    zone = sv.PolygonZone(polygon=ZONE_POLYGON, frame_resolution_wh=frame_resolution)
    zone_annotator = sv.PolygonZoneAnnotator(zone=zone, color=sv.Color.WHITE)

    results = model(frame, agnostic_nms=True)[0]
    detections = sv.Detections.from_ultralytics(results)
    mask = zone.trigger(detections=detections)
    detections = detections[(detections.confidence > 0.9) & mask]
    detections = tracker.update_with_detections(detections)

    labels = [
        f"#{tracker_id} {results.names[class_id]}"
        for class_id, tracker_id in zip(detections.class_id, detections.tracker_id)
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

    center_coordinates = get_cube_center(results)
    # Check if the center coordinates are inside the zone
    if cv2.pointPolygonTest(ZONE_POLYGON, center_coordinates, measureDist=False) >= 0:
        cv2.circle(annotated_frame, center_coordinates, radius=5, color=(0, 255, 0), thickness=-1)

    zone.trigger(detections=detections)
    annotated_frame = zone_annotator.annotate(scene=annotated_frame)


    # Afficher ou enregistrer le cadre annoté
    output_path = '../img/detected_cube.jpg'
    success = cv2.imwrite(output_path, annotated_frame)
    if success:
        print(f"Image enregistrée sous {output_path}")
    else:
        print("Erreur: Impossible d'enregistrer l'image")

    #cv2.imshow("Yolov8", annotated_frame)
    #cv2.waitKey(0)
    #cv2.destroyAllWindows()
    return center_coordinates

