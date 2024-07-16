import cv2
import numpy as np

def detect_robot_center(frame, aruco_dict_type=cv2.aruco.DICT_4X4_100, target_id=0):
    # Convert the image to grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Define the ArUco dictionary and detector parameters
    aruco_dict = cv2.aruco.Dictionary_get(aruco_dict_type)
    parameters = cv2.aruco.DetectorParameters_create()

    # Detect markers
    corners, ids, _ = cv2.aruco.detectMarkers(frame, aruco_dict, parameters=parameters)

    center_coordinates = None
    annotated_frame = frame.copy()

    if ids is not None:
        for i in range(len(ids)):
            if ids[i][0] == target_id:
                # Calculate the coordinates of the ArUco marker center
                c = corners[i][0]
                center_x = int((c[0][0] + c[1][0] + c[2][0] + c[3][0]) / 4)
                center_y = int((c[0][1] + c[1][1] + c[2][1] + c[3][1]) / 4)
                center_coordinates = (center_x, center_y)

                # Annotate the image
                #cv2.circle(annotated_frame, center_coordinates, 10, (0, 255, 0), -1)
                print("ROBOT DETECTED")
                break

    return center_coordinates, annotated_frame
