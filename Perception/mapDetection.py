import cv2
import numpy as np

def detect_aruco_markers(frame, aruco_dict_type, matrix_coefficients, distortion_coefficients):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    aruco_dict = cv2.aruco.Dictionary_get(aruco_dict_type)
    parameters = cv2.aruco.DetectorParameters_create()

    corners, ids, _ = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=parameters,
                                              cameraMatrix=matrix_coefficients,
                                              distCoeff=distortion_coefficients)

    detected_markers = []

    if len(corners) > 0:
        for i in range(len(ids)):
            # Convert corner coordinates to tuple
            marker_corners = tuple(map(tuple, corners[i][0]))
            detected_markers.append((ids[i][0], marker_corners))

    return detected_markers


def mapDetection(frame):
    # Camera calibration parameters
    intrinsic_camera = np.array(((1281.57894, 0, 457.638346), (0, 1262.76271, 260.388263), (0, 0, 1)))
    distortion = np.array((0.12431658, -0.55314019, 0, 0, 0))
    # ArUco dictionary
    arucoDict = cv2.aruco.DICT_4X4_1000
    aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_1000)
    parameters = cv2.aruco.DetectorParameters_create()

    # Variables to store detected markers
    detected_markers = []

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    # Detect ArUco markers
    detected_markers = detect_aruco_markers(frame, arucoDict, intrinsic_camera, distortion)

    # Draw detected markers
    new_polygon = []
    if len(detected_markers) > 0:
        # Sort detected markers by their IDs
        detected_markers.sort(key=lambda x: x[0])
        for marker_id, marker_corners in detected_markers:
            # Convert corner coordinates to integers
            marker_corners_int = np.int32(marker_corners)
            # Draw marker ID and corner coordinates on the frame
            cv2.putText(frame, f"ID: {marker_id}", (marker_corners_int[0][0], marker_corners_int[0][1] - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            cv2.putText(frame, f"({marker_corners_int[0][0]}, {marker_corners_int[0][1]})",
                        (marker_corners_int[0][0], marker_corners_int[0][1] + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                        (0, 0, 0), 2)
            # Determine specific corners based on ArUco marker ID
            if marker_id == 1:
                # Bottom left corner
                corner = marker_corners_int[1]
                new_polygon.insert(1,corner)
            elif marker_id == 2:
                # Top left corner
                corner = marker_corners_int[2]
                new_polygon.insert(2,corner)
            elif marker_id == 3:
                # Bottom right corner
                corner = marker_corners_int[3]
                new_polygon.insert(3,corner)
            elif marker_id == 4:
                # Top right corner
                corner = marker_corners_int[0]
                new_polygon.insert(0,corner)
            else:
                continue  # Skip this marker if ID is not 1, 2, 3, or 4

    # Check if all corners have been found before continuing
    if len(new_polygon) == 4:
        # Save ZONE_POLYGON to a file
        ZONE_POLYGON = np.array(new_polygon)
        #np.savetxt('zone_polygon.txt', ZONE_POLYGON.astype(int), fmt='%d')

        # Draw the polygon on the frame
        cv2.polylines(frame, [np.int32(ZONE_POLYGON)], True, (0, 255, 0), 2)
        print(new_polygon)
        return ZONE_POLYGON


    #cv2.imshow('Detected Arucos', frame)

    # Save the frame and exit
    #cv2.imwrite('webcam_frame.jpg', frame)

