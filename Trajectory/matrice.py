import cv2
import numpy as np
from Perception.mapDetectionFromIMG import mapDetection
from Perception.cubeDetectionFromIMG import cube_detection
from Perception.robotDetection import detect_robot_center
from Astar import astar
from Trajectory.robotToCube import robotToTarget


def grid_to_cartesian(grid_point, cell_width, cell_height):
    # Convertit les coordonnées de la grille (row, col) en coordonnées cartésiennes (x, y)
    y, x = grid_point
    cartesian_x = x * cell_width + cell_width // 2
    cartesian_y = y * cell_height + cell_height // 2
    return (cartesian_x, cartesian_y)


def get_roi(map, resized_image, image_width, image_height):
    # Recupère les coordonnées des coins de la map
    bottom_left = map[1]
    top_left = map[2]
    top_right = map[3]
    bottom_right = map[0]

    # Assurez-vous que les points sont dans le bon ordre
    pts1 = np.array([bottom_left, top_left, top_right, bottom_right], dtype='float32')
    # Dimensions de l'image cible
    pts2 = np.array([[0, image_height], [0, 0], [image_width, 0], [image_width, image_height]], dtype='float32')

    # Calculer la matrice de transformation
    matrix = cv2.getPerspectiveTransform(pts1, pts2)
    roi = cv2.warpPerspective(resized_image, matrix, (image_width, image_height))

    return roi

def matrice(box_coordinates, center_cube, robot_center, grid_rows, grid_cols, cell_width, cell_height):
    # Initialiser la matrice de grille
    grid_obstacle = np.zeros((grid_rows, grid_cols), dtype=int)

    # Convertir les coordonnées des boîtes en indices de grille et marquer les cellules
    for box in box_coordinates:
        x_min, y_min, x_max, y_max = box
        col_start = int(x_min // cell_width)
        col_end = int(x_max // cell_width)
        row_start = int(y_min // cell_height)
        row_end = int(y_max // cell_height)

        for row in range(row_start, row_end + 1):
            for col in range(col_start, col_end + 1):
                grid_obstacle[row, col] = 1  # Marquer la cellule comme occupée

    # Initialiser la matrice de grille pour les emplacements
    grid_location = np.zeros((grid_rows, grid_cols), dtype=int)

    # Convertir les coordonnées de center_cube en indices de grille
    cube_row = int(center_cube[1] // cell_height)
    cube_col = int(center_cube[0] // cell_width)

    robot_row = int(robot_center[1] // cell_height)
    robot_col = int(robot_center[0] // cell_width)

    #grid_location[4:15, 17:18] = 1  # Par exemple, marquer un bloc d'occupation

    return grid_location, grid_obstacle, cube_row, cube_col, robot_row, robot_col



def main(image):
    # Redimensionner l'image pour une résolution plus gérable (par exemple, 1024x768)
    scale_percent = 90  # pourcentage de réduction
    width = int(image.shape[1] * scale_percent / 100)
    height = int(image.shape[0] * scale_percent / 100)
    dim = (width, height)

    # Redimensionner l'image
    resized_image = cv2.resize(image, dim, interpolation=cv2.INTER_AREA)

    # Detection de la map
    map = mapDetection(resized_image)

    # Recupère les coordonnées des coins de la map
    bottom_left = map[1]
    top_left = map[2]
    top_right = map[3]
    bottom_right = map[0]

    # Assurez-vous que les points sont dans le bon ordre
    pts1 = np.array([bottom_left, top_left, top_right, bottom_right], dtype='float32')
    # Dimensions de l'image cible
    image_width = 1500
    image_height = 1000
    pts2 = np.array([[0, image_height], [0, 0], [image_width, 0], [image_width, image_height]], dtype='float32')

    # Calculer la matrice de transformation
    matrix = cv2.getPerspectiveTransform(pts1, pts2)
    roi = cv2.warpPerspective(resized_image, matrix, (image_width, image_height))

    #Detection Objet
    center_cube, box_coordinates, roi = cube_detection(roi)

    # Création matrices 30x20
    grid_rows, grid_cols = 20, 30
    cell_width = image_width // grid_cols
    cell_height = image_height // grid_rows

    # Initialiser la matrice de grille
    grid_obstacle = np.zeros((grid_rows, grid_cols), dtype=int)

    # Convertir les coordonnées des boîtes en indices de grille et marquer les cellules
    for box in box_coordinates:
        x_min, y_min, x_max, y_max = box
        col_start = int(x_min // cell_width)
        col_end = int(x_max // cell_width)
        row_start = int(y_min // cell_height)
        row_end = int(y_max // cell_height)

        for row in range(row_start, row_end + 1):
            for col in range(col_start, col_end + 1):
                grid_obstacle[row, col] = 1  # Marquer la cellule comme occupée

    # Initialiser la matrice de grille pour les emplacements
    grid_location = np.zeros((grid_rows, grid_cols), dtype=int)

    # Convertir les coordonnées de center_cube en indices de grille
    cube_row = int(center_cube[1] // cell_height)
    cube_col = int(center_cube[0] // cell_width)

    robot_center, roi = detect_robot_center(roi)

    robot_row = int(robot_center[1] // cell_height)
    robot_col = int(robot_center[0] // cell_width)

    grid_location[4:15, 17:18] = 1  # Par exemple, marquer un bloc d'occupation

    path, directions = astar(grid_location, (robot_row, robot_col), (cube_row, cube_col))

    grid_location[cube_row, cube_col] = 1
    grid_location[robot_row, robot_col] = 2 #2 pour le robot

    # Afficher la matrice de grille
    print("Matrice de grille (1 indique une cellule occupée, 0 indique une cellule libre) :")
    print(grid_location)

    print("Chemin le plus court:", path) #y,x
    print("Points de changement de direction:", directions) #y,x

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
    # Charger l'image

    #cap = cv2.VideoCapture(1)

    # Capture image par image
    #ret, frame = cap.read()

    image = cv2.imread('../data/map/IMG_0701.jpg')

    main(image)
