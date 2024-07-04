import cv2
import numpy as np
from Perception.mapDetectionFromIMG import mapDetection
from Trajectory.matrice import matrice, get_roi, grid_to_cartesian
from Perception.cubeDetectionFromIMG import cube_detection
from Perception.robotDetection import detect_robot_center
from Trajectory.robotToCube import robotToTarget
from Astar import astar

image_width = 1500
image_height = 1000

def main(image):
    #Récupération ce la video
    #cap = cv2.VideoCapture(0)

    #cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    #cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)


    #ret, image = cap.read()

    # Redimensionner l'image pour une résolution plus gérable (par exemple, 1024x768)
    scale_percent = 80  # pourcentage de réduction
    width = int(image.shape[1] * scale_percent / 100)
    height = int(image.shape[0] * scale_percent / 100)
    dim = (width, height)

    # Redimensionner l'image
    resized_image = cv2.resize(image, dim, interpolation=cv2.INTER_AREA)

    # Detection de la map
    map = mapDetection(resized_image)

    #Sélection de la region of interest
    roi = get_roi(map, resized_image, image_width, image_height)

    # Detection Objet
    center_cube, box_coordinates, roi = cube_detection(roi)

    # Création matrices 30x20
    grid_rows, grid_cols = 20, 30
    cell_width = image_width // grid_cols
    cell_height = image_height // grid_rows

    # Detection Robot
    robot_center, roi = detect_robot_center(roi)

    #Recuération de la map modelisé en fonction des object/obstacles presents
    grid_location, grid_obstacle, cube_row, cube_col, robot_row, robot_col = matrice(box_coordinates, center_cube, robot_center, grid_rows, grid_cols, cell_width, cell_height)

    cv2.imshow('Gridded ROI', resized_image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

    path, directions = astar(grid_location, (robot_row, robot_col), (cube_row, cube_col))

    grid_location[cube_row, cube_col] = 1
    grid_location[robot_row, robot_col] = 2  # 2 pour le robot

    # Afficher la matrice de grille
    print("Matrice de grille (1 indique une cellule occupée, 0 indique une cellule libre) :")
    print(grid_location)

    print("Chemin le plus court:", path)  # y,x
    print("Points de changement de direction:", directions)  # y,x

    print(robot_center)
    print(center_cube)


    # Convertir les directions en coordonnées cartésiennes
    cartesian_directions = [grid_to_cartesian(point, cell_width, cell_height) for point in directions]
    print(cartesian_directions)

    cv2.imshow('Gridded ROI', roi)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

    robotToTarget(roi, robot_center, center_cube, cartesian_directions)


if __name__ == '__main__':
    image = cv2.imread('data/map/IMG_0701.jpg')

    main(image)
