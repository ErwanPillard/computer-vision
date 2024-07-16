import cv2
import numpy as np
import requests
from Trajectory.matrice import get_roi  # Assuming get_roi is a function imported from Trajectory.matrice module

# Camera calibration parameters
intrinsic_camera = np.array([
    [1.05327935e+04, 0.00000000e+00, 9.49629087e+02],
    [0.00000000e+00, 6.30700850e+03, 5.43852752e+02],
    [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]
])

distortion = np.array([
    [-0.45353514, 186.65504933, 0.39744416, -0.35031288, 1.01085776]
])

def robotToTarget(start, goal, directions, map1):
    current_position = start

    for target in directions:
        move_robot_to_target(current_position, target, map1)
        current_position = target

    move_robot_to_target(current_position, goal, map1)


def move_robot_to_target(start, target, map1):
    reached_target = False

    # Video capture initialization
    cap = cv2.VideoCapture(1)  # Assuming camera index 1, change if necessary

    while not reached_target:
        ret, image = cap.read()  # Read frame from camera
        frame = get_roi(map1, image, 1500, 1000)  # Get region of interest from the map
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)  # Convert frame to grayscale

        # ArUco markers detection
        aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_1000)
        parameters = cv2.aruco.DetectorParameters_create()
        corners, ids, _ = cv2.aruco.detectMarkers(frame, aruco_dict, parameters=parameters,
                                                  cameraMatrix=intrinsic_camera,
                                                  distCoeffs=distortion)

        robot_detected = False
        robot_index = -1
        if len(corners) > 0:
            for i in range(len(ids)):
                if ids[i] == 0:  # Only look for marker with ID 0 (adjust if needed)
                    robot_detected = True
                    robot_index = i
                    break

        if robot_detected:
            # Estimate pose of the robot marker
            rvec_robot, tvec_robot, _ = cv2.aruco.estimatePoseSingleMarkers(corners[robot_index], 0.02,
                                                                            intrinsic_camera,
                                                                            distortion)

            # Get centers of robot marker and target object in the image
            robot_center = np.array(tuple(map(int, corners[robot_index].mean(axis=1)[0])))
            object_center = np.array(target)

            # Calculate direction vector from robot marker to target object
            direction_vector = object_center - robot_center
            norm_direction_vector = direction_vector.flatten() / np.linalg.norm(direction_vector)

            # Calculate rotation matrix for the robot
            rotation_matrix, _ = cv2.Rodrigues(rvec_robot[0])

            # Calculate robot's forward vector (front-facing)
            robot_forward_vector = np.array([[1, 0, 0]])

            # Transform robot's forward vector into world coordinate system
            robot_forward_vector_world = np.dot(rotation_matrix, robot_forward_vector.T)

            # Normalize robot's forward vector
            norm_robot_forward_vector = robot_forward_vector_world.flatten() / np.linalg.norm(robot_forward_vector_world)

            # Calculate signed angle between robot's forward vector and direction vector
            signed_angle_rad = np.arctan2(norm_direction_vector[1], norm_direction_vector[0]) - np.arctan2(
                norm_robot_forward_vector[1], norm_robot_forward_vector[0])
            signed_angle_deg = np.degrees(np.arctan2(np.sin(signed_angle_rad), np.cos(signed_angle_rad)))

            # Calculate distance between robot and object
            distance = np.linalg.norm(direction_vector)

            print("Signed Angle: {} degrees".format(signed_angle_deg))
            print("Distance to target: {} units".format(distance))

            # Draw line between robot marker center and object center
            cv2.line(frame, tuple(robot_center), tuple(object_center), (0, 255, 0), 2)

            # Draw circle at robot marker center
            cv2.circle(frame, tuple(robot_center), 5, (0, 0, 255), -1)

            # Draw circle at object center
            cv2.circle(frame, tuple(object_center), 5, (255, 0, 0), -1)

            # Draw detected marker and marker axis on the frame
            cv2.aruco.drawDetectedMarkers(frame, corners)
            cv2.aruco.drawAxis(frame, intrinsic_camera, distortion, rvec_robot, tvec_robot, 0.01)

            # Send data to control the robot (e.g., rotation and distance)
            send_data(signed_angle_deg, distance)

            # Update current position of the robot
            current_position = robot_center

            # Check if robot has reached the target
            if distance < 10:  # Tolerance threshold
                reached_target = True

        # Display updated frame
        cv2.imshow("Yolov8", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):  # Press 'q' to quit
            break

    cap.release()  # Release the capture
    cv2.destroyAllWindows()  # Close all OpenCV windows


def send_data(angle, distance):
    url = 'http://192.168.1.15/data'  # Replace with the IP address of your ESP8266 or server

    # Send distance and angle values to the server
    response = requests.get(url, params={'distance': distance, 'angle': angle, 'vitesse_r': 160, 'vitesse': 251})
    # print(response.text)  # Print response from the server (optional)
