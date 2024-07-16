import cv2
from ultralytics import YOLO  # Assuming YOLO model is imported from ultralytics library
import supervision as sv  # Assuming supervision module is imported as sv

# Load pre-trained YOLOv8 model
model = YOLO("Model/best.pt")  # Modify the path as needed, e.g., "../Model/best.pt"

# Define annotators
box_annotator = sv.BoundingBoxAnnotator()
label_annotator = sv.LabelAnnotator()

# Open webcam (default index 0)
cap = cv2.VideoCapture(0)

# Check if webcam opened successfully
if not cap.isOpened():
    print("Error: Unable to open webcam.")
    exit()

while True:
    # Capture frame-by-frame
    ret, frame = cap.read()

    # Check if frame is read correctly
    if not ret:
        print("Error: Unable to read frame from webcam.")
        break

    # Perform prediction on the current frame
    results = model(frame, agnostic_nms=True)[0]  # Perform inference using YOLO model
    detections = sv.Detections.from_ultralytics(results)  # Convert detections to a standardized format

    # Filter detections based on confidence threshold (e.g., keep detections with confidence > 0.85)
    detections = detections[detections.confidence > 0.85]

    # Generate labels for the detections
    labels = [
        f"{results.names[class_id]}"  # Get class names corresponding to class IDs in detections
        for class_id in detections.class_id
    ]

    # Annotate the frame with bounding boxes
    annotated_frame = box_annotator.annotate(
        scene=frame,
        detections=detections,
    )

    # Annotate the frame with labels
    annotated_frame = label_annotator.annotate(
        annotated_frame,
        detections=detections,
        labels=labels
    )

    # Display the annotated frame
    cv2.imshow("Webcam - Detection", annotated_frame)

    # Exit the loop if 'q' is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release resources
cap.release()
cv2.destroyAllWindows()
