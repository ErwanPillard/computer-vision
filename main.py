import cv2
from Perception.mapDetectionFromIMG import mapDetection
from Trajectory.matrice import matrice, get_roi, grid_to_cartesian
from Perception.cubeDetectionFromIMG import cube_detection
from Perception.robotDetection import detect_robot_center
from Trajectory.robotToCube import robotToTarget
from Trajectory.Astar import astar

image_width = 1500
image_height = 1000

def main(image):

    # Resize the image for a more manageable resolution (e.g., 1024x768)
    # scale_percent = 80  # percentage of reduction
    # width = int(image.shape[1] * scale_percent / 100)
    # height = int(image.shape[0] * scale_percent / 100)
    # dim = (width, height)

    # Resize the image
    # resized_image = cv2.resize(image, dim, interpolation=cv2.INTER_AREA)

    # Map detection
    map = mapDetection(image)

    # Select the region of interest
    roi = get_roi(map, image, image_width, image_height)

    # Object detection
    center_cube, box_coordinates, roi = cube_detection(roi)

    # Create 30x20 grids/matrices
    grid_rows, grid_cols = 20, 30
    cell_width = image_width // grid_cols
    cell_height = image_height // grid_rows

    # Robot detection
    robot_center, roi = detect_robot_center(roi)

    cv2.imshow('Gridded ROI', roi)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

    # Retrieve the modeled map based on detected objects/obstacles
    grid_location, grid_obstacle, cube_row, cube_col, robot_row, robot_col = matrice(box_coordinates, center_cube, robot_center, grid_rows, grid_cols, cell_width, cell_height)

    path, directions = astar(grid_location, (robot_row, robot_col), (cube_row, cube_col))

    grid_location[cube_row, cube_col] = 1
    grid_location[robot_row, robot_col] = 2  # 2 for the robot

    # Display the grid matrix
    print("Grid matrix (1 indicates an occupied cell, 0 indicates a free cell):")
    print(grid_location)

    print("Shortest path:", path)  # y,x
    print("Direction change points:", directions)  # y,x

    print(robot_center)
    print(center_cube)

    # Convert directions to Cartesian coordinates
    cartesian_directions = [grid_to_cartesian(point, cell_width, cell_height) for point in directions]
    print(cartesian_directions)

    robotToTarget(robot_center, center_cube, cartesian_directions, map)


if __name__ == '__main__':
    # Uncomment and specify the path to your image if not using webcam
    # image = cv2.imread('data/map/IMG_0701.jpg')

    # Capture video stream
    cap = cv2.VideoCapture(1)

    i = 0
    while True:
        ret, image = cap.read()
        i = i + 1
        if i == 20:
            break

    main(image)
