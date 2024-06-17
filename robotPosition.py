import cv2
import numpy as np


def pose_estimation(frame, aruco_dict_type, matrix_coefficients, distortion_coefficients, robot_marker_id):
    # Convertir l'image en niveaux de gris
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    # Créer un dictionnaire ArUco
    aruco_dict = cv2.aruco.Dictionary_get(aruco_dict_type)

    # Créer un détecteur ArUco
    parameters = cv2.aruco.DetectorParameters_create()

    # Détecter les marqueurs ArUco
    corners, ids, _ = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=parameters, cameraMatrix=matrix_coefficients,
                                              distCoeff=distortion_coefficients)

    if ids is not None and robot_marker_id in ids:
        # Récupérer l'indice du marqueur ArUco du robotq
        robot_marker_index = np.where(ids == robot_marker_id)[0][0]

        # Estimer la pose du marqueur ArUco du robot
        rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(corners[robot_marker_index], 0.05, matrix_coefficients,
                                                            distortion_coefficients)

        # Retourner les vecteurs de rotation et de translation
        return rvec, tvec

    # Si le marqueur du robot n'est pas détecté, retourner None
    return None, None


def main():
    # Camera calibration parameters
    intrinsic_camera = np.array(((1281.57894, 0, 457.638346), (0, 1262.76271, 260.388263), (0, 0, 1)))
    distortion = np.array((0.12431658, -0.55314019, 0, 0, 0))

    # Ouvrir la webcam
    cap = cv2.VideoCapture(0)

    while True:
        # Lire un frame de la webcam
        ret, frame = cap.read()

        # Appeler la fonction de pose estimation
        rvec, tvec = pose_estimation(frame, cv2.aruco.DICT_4X4_1000, intrinsic_camera, distortion, 5)

        # Afficher les résultats
        print('Rotation Vector: ', rvec)
        print('Translation Vector: ', tvec)

        # Si le marqueur est détecté, dessiner les vecteurs à l'écran
        if rvec is not None and tvec is not None:
            # Créer un frame de sortie
            frame_out = frame.copy()

            # Dessiner les axes du marqueur
            cv2.aruco.drawAxis(frame_out, intrinsic_camera, distortion, rvec, tvec, 0.1)

            # Afficher le frame de sortie
            cv2.imshow('frame', frame_out)
        else:
            # Afficher le frame original
            cv2.imshow('frame', frame)

        # Vérifier si l'utilisateur a appuyé sur la touche 'q' pour quitter
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Libérer la webcam
    cap.release()

    # Fermer toutes les fenêtres
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
