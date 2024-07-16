import cv2
import numpy as np

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

    grid_location[4:15, 17:18] = 1  # Par exemple, marquer un bloc d'occupation

    return grid_location, grid_obstacle, cube_row, cube_col, robot_row, robot_col