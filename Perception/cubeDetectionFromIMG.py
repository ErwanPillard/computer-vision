import cv2
from ultralytics import YOLO
import supervision as sv

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
    # Load YOLO model
    model = YOLO("../Model/best.pt")
    tracker = sv.ByteTrack()

    # Define annotators
    box_annotator = sv.BoundingBoxAnnotator()
    label_annotator = sv.LabelAnnotator()

    # Get frame resolution
    frame_resolution = (frame.shape[1], frame.shape[0])

    # Define polygon zone
    zone = sv.PolygonZone(polygon=ZONE_POLYGON, frame_resolution_wh=frame_resolution)
    zone_annotator = sv.PolygonZoneAnnotator(zone=zone, color=sv.Color.WHITE)

    # Perform object detection
    results = model(frame, agnostic_nms=True)[0]
    detections = sv.Detections.from_ultralytics(results)

    # Filter detections by confidence and apply zone mask
    mask = zone.trigger(detections=detections)
    detections = detections[(detections.confidence > 0.9) & mask]
    detections = tracker.update_with_detections(detections)

    # Generate labels for tracked objects
    labels = [
        f"#{tracker_id} {results.names[class_id]}"
        for class_id, tracker_id in zip(detections.class_id, detections.tracker_id)
    ]

    # Annotate the frame with bounding boxes and labels
    annotated_frame = box_annotator.annotate(
        scene=frame,
        detections=detections,
    )

    annotated_frame = label_annotator.annotate(
        annotated_frame,
        detections=detections,
        labels=labels
    )

    # Get center coordinates of the cube
    center_coordinates = get_cube_center(results)

    # Check if the center coordinates are inside the zone polygon
    if cv2.pointPolygonTest(ZONE_POLYGON, center_coordinates, measureDist=False) >= 0:
        cv2.circle(annotated_frame, center_coordinates, radius=5, color=(0, 255, 0), thickness=-1)

    # Trigger zone annotation
    zone.trigger(detections=detections)
    annotated_frame = zone_annotator.annotate(scene=annotated_frame)

    # Save annotated frame to file
    output_path = '../img/detected_cube.jpg'
    success = cv2.imwrite(output_path, annotated_frame)
    if success:
        print(f"Image saved to {output_path}")
    else:
        print("Error: Failed to save image")

    # Uncomment to display the annotated frame
    # cv2.imshow("Yolov8", annotated_frame)
    # cv2.waitKey(0)
    # cv2.destroyAllWindows()

    return center_coordinates


def get_boxes(results):
    boxes = []
    for result in results.boxes.xyxy:
        x_min, y_min, x_max, y_max = result[:4]
        boxes.append((x_min, y_min, x_max, y_max))
    return boxes


def cube_detection(frame):
    # Load YOLO model
    model = YOLO("Model/best.pt")

    # Define annotators
    box_annotator = sv.BoundingBoxAnnotator()
    label_annotator = sv.LabelAnnotator()

    # Perform object detection
    results = model(frame, agnostic_nms=True)[0]
    detections = sv.Detections.from_ultralytics(results)
    detections = detections[(detections.confidence > 0.9)]

    # Generate labels for detected objects
    labels = [
        f"{results.names[class_id]}"
        for class_id in detections.class_id
    ]

    # Annotate the frame with bounding boxes and labels
    annotated_frame = box_annotator.annotate(
        scene=frame,
        detections=detections,
    )

    annotated_frame = label_annotator.annotate(
        annotated_frame,
        detections=detections,
        labels=labels
    )

    # Get center coordinates of the cube
    center_coordinates = get_cube_center(results)
    box_coordinates = get_boxes(results)

    return center_coordinates, box_coordinates, annotated_frame
