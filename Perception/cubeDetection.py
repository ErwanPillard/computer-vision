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


def get_cube_center(results):
    for i in range(len(results.boxes.xyxy)):
        if results.names[int(results.boxes.cls[i])] == "cube":
            x1, y1, x2, y2 = results.boxes.xyxy[i][:4]
            center_x = int((x1 + x2) / 2)
            center_y = int((y1 + y2) / 2)
            return (center_x, center_y)
    return None


def setup_camera():
    args = parse_arguments()
    frame_width, frame_height = args.webcam_resolution

    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, frame_width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, frame_height)

    return cap, args


def cube_detection_inZone(ZONE_POLYGON):
    cap, args = setup_camera()

    model = YOLO("../Model/best_2.pt")
    tracker = sv.ByteTrack()

    #Type of annotator
    box_annotator = sv.BoundingBoxAnnotator()
    label_annotator = sv.LabelAnnotator()

    zone = sv.PolygonZone(polygon=ZONE_POLYGON, frame_resolution_wh=tuple(args.webcam_resolution))
    zone_annotator = sv.PolygonZoneAnnotator(zone=zone, color=sv.Color.WHITE)

    while True:
        ret, frame = cap.read()

        results = model(frame, agnostic_nms=True)[0]
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

        # annotated_frame = box_annotator.annotate(
        #     scene=frame, detections=detections)

        center_coordinates = get_cube_center(results)
        # Check if the center coordinates are inside the zone
        if cv2.pointPolygonTest(ZONE_POLYGON, center_coordinates, measureDist=False) >= 0:
            cv2.circle(annotated_frame, center_coordinates, radius=5, color=(0, 255, 0), thickness=-1)

        zone.trigger(detections=detections)
        frame = zone_annotator.annotate(scene=annotated_frame)

        cv2.imshow("Yolov8", frame)

        if (cv2.waitKey(1) & 0xFF == ord('q')):
            break


def robot_object():
    cap, args = setup_camera()

    model = YOLO("../Model/best_2.pt")

    # Type of annotator
    box_annotator = sv.BoundingBoxAnnotator()

    while True:
        ret, frame = cap.read()
        #Detect cube
        results = model(frame, agnostic_nms=True)[0]
        detections = sv.Detections.from_ultralytics(results)
        detections = detections[(detections.confidence > 0.8)]
        frame = box_annotator.annotate(
            scene=frame,
            detections=detections,
        )
        center_coordinates = get_cube_center(results)

        # Detect Robot
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_1000)
        parameters = cv2.aruco.DetectorParameters_create()

        corners, ids, _ = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=parameters,
                                                  cameraMatrix=intrinsic_camera,
                                                  distCoeff=distortion)

        robot_detected = False
        robot_index = -1
        if len(corners) > 0:
            for i in range(len(ids)):
                if ids[i] == 3:  # Recherche uniquement le marqueur avec l'ID 3
                    robot_detected = True
                    robot_index = i
                    break

        if robot_detected:
            # Estimer la pose du marqueur du robot
            rvec_robot, tvec_robot, _ = cv2.aruco.estimatePoseSingleMarkers(corners[robot_index], 0.02,
                                                                            intrinsic_camera,
                                                                            distortion)

            # Obtenir les centres du marqueur du robot et de l'objet dans l'image
            robot_center = np.array(tuple(map(int, corners[robot_index].mean(axis=1)[0])))
            object_center = np.array(tuple(map(int, center_coordinates)))

            # Calculate the direction vector from the robot marker to the object marker
            direction_vector = object_center - robot_center

            norm_direction_vector = direction_vector.flatten() / np.linalg.norm(direction_vector)

            # Calculate the rotation matrix for the robot
            rotation_matrix, _ = cv2.Rodrigues(rvec_robot[0])

            # Calculate the robot's forward vector (face avant)
            robot_forward_vector = np.array([[1, 0, 0]])

            # Transform the robot's forward vector to the world coordinate system
            robot_forward_vector_world = np.dot(rotation_matrix, robot_forward_vector.T)

            # Normalize the robot's forward vector
            norm_robot_forward_vector = robot_forward_vector_world.flatten() / np.linalg.norm(
                robot_forward_vector_world)

            # Calculate the signed angle between the robot's forward vector and the direction vector
            signed_angle_rad = np.arctan2(norm_direction_vector[1], norm_direction_vector[0]) - np.arctan2(
                norm_robot_forward_vector[1], norm_robot_forward_vector[0])
            signed_angle_deg = np.degrees(np.arctan2(np.sin(signed_angle_rad), np.cos(signed_angle_rad)))

            # Calculate the distance between the robot and the object
            # distance = np.linalg.norm(direction_vector)

            print("Signed Angle: {} degrees".format(signed_angle_deg))

            # Dessiner une ligne entre les centres du marqueur du robot et de l'objet
            cv2.line(frame, robot_center, object_center, (0, 255, 0), 2)

            # Dessiner un cercle au centre du marqueur du robot
            cv2.circle(frame, robot_center, 5, (0, 0, 255), -1)

            # Dessiner un cercle au centre de l'objet
            cv2.circle(frame, object_center, 5, (255, 0, 0), -1)

            # Dessiner le marqueur détecté et l'axe du marqueur du robot sur l'image
            cv2.aruco.drawDetectedMarkers(frame, corners)
            cv2.aruco.drawAxis(frame, intrinsic_camera, distortion, rvec_robot, tvec_robot, 0.01)

        cv2.imshow("Yolov8", frame)

        if (cv2.waitKey(1) & 0xFF == ord('q')):
            break



if __name__ == '__main__':
    robot_object()
