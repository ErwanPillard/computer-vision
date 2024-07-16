
import cv2
import numpy as np
import requests
from Trajectory.matrice import get_roi

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

    # Récupération ce la video
    cap = cv2.VideoCapture(1)

    while not reached_target:

        ret, image = cap.read()
        frame = get_roi(map1, image, 1500, 1000)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_1000)
        parameters = cv2.aruco.DetectorParameters_create()

        corners, ids, _ = cv2.aruco.detectMarkers(frame, aruco_dict, parameters=parameters,
                                                  cameraMatrix=intrinsic_camera,
                                                  distCoeff=distortion)

        robot_detected = False
        robot_index = -1
        if len(corners) > 0:
            for i in range(len(ids)):
                if ids[i] == 0:  # Recherche uniquement le marqueur avec l'ID 5
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
            object_center = np.array(target)

            # Calculer le vecteur de direction du marqueur du robot au marqueur de l'objet
            direction_vector = object_center - robot_center

            norm_direction_vector = direction_vector.flatten() / np.linalg.norm(direction_vector)

            # Calculer la matrice de rotation pour le robot
            rotation_matrix, _ = cv2.Rodrigues(rvec_robot[0])

            # Calculer le vecteur avant du robot (face avant)
            robot_forward_vector = np.array([[1, 0, 0]])

            # Transformer le vecteur avant du robot dans le système de coordonnées du monde
            robot_forward_vector_world = np.dot(rotation_matrix, robot_forward_vector.T)

            # Normaliser le vecteur avant du robot
            norm_robot_forward_vector = robot_forward_vector_world.flatten() / np.linalg.norm(
                robot_forward_vector_world)

            # Calculer l'angle signé entre le vecteur avant du robot et le vecteur de direction
            signed_angle_rad = np.arctan2(norm_direction_vector[1], norm_direction_vector[0]) - np.arctan2(
                norm_robot_forward_vector[1], norm_robot_forward_vector[0])
            signed_angle_deg = np.degrees(np.arctan2(np.sin(signed_angle_rad), np.cos(signed_angle_rad)))

            # Calculer la distance entre le robot et l'objet
            distance = np.linalg.norm(direction_vector)

            print("Signed Angle: {} degrees".format(signed_angle_deg))
            print("Distance to target: {} units".format(distance))

            # Dessiner une ligne entre les centres du marqueur du robot et de l'objet
            cv2.line(frame, robot_center, object_center, (0, 255, 0), 2)

            # Dessiner un cercle au centre du marqueur du robot
            cv2.circle(frame, robot_center, 5, (0, 0, 255), -1)

            # Dessiner un cercle au centre de l'objet
            cv2.circle(frame, object_center, 5, (255, 0, 0), -1)

            # Dessiner le marqueur détecté et l'axe du marqueur du robot sur l'image
            cv2.aruco.drawDetectedMarkers(frame, corners)
            cv2.aruco.drawAxis(frame, intrinsic_camera, distortion, rvec_robot, tvec_robot, 0.01)

            # Envoyer les données pour contrôler le robot (exemple : rotation et distance)
            #send_data(signed_angle_deg, distance)

            # Mettre à jour la position actuelle du robot
            current_position = robot_center

            # Vérifier si le robot a atteint la cible
            if distance < 10:  # Seuil de tolérance
                reached_target = True

        # Afficher l'image mise à jour
        cv2.imshow("Yolov8", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

def send_data(angle, distance):

    #url = 'http://172.20.10.2/data'  # Replace <ESP8266-IP-ADDRESS> with the IP address of your ESP8266
    url = 'http://192.168.1.15/data'  # Replace <ESP8266-IP-ADDRESS> with the IP address of your ESP8266

    # Send the distance and angle values to the ESP8266 server
    response = requests.get(url, params={'distance': distance, 'angle': angle, 'vitesse_r': 160, 'vitesse': 251})
    #qprint(response.text)  # Print the response from the ESP8266 server
