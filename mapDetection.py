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


def main():
    # Camera calibration parameters
    intrinsic_camera = np.array(((1281.57894, 0, 457.638346), (0, 1262.76271, 260.388263), (0, 0, 1)))
    distortion = np.array((0.12431658, -0.55314019, 0, 0, 0))

    # Initialize webcam
    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

    # ArUco dictionary
    arucoDict = cv2.aruco.DICT_4X4_1000
    aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_1000)
    parameters = cv2.aruco.DetectorParameters_create()

    # Variables to store detected markers
    detected_markers = []

    while cap.isOpened():
        ret, frame = cap.read()

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        # Detect ArUco markers
        detected_markers = detect_aruco_markers(frame, arucoDict, intrinsic_camera, distortion)

        # Draw detected markers
        # if len(detected_markers) > 0:
        #     for marker_id, marker_corners in detected_markers:
        #         # Convert corner coordinates to integers
        #         marker_corners_int = np.int32(marker_corners)
        #         # Draw marker ID and corner coordinates on the frame
        #         cv2.putText(frame, f"ID: {marker_id}", (marker_corners_int[0][0], marker_corners_int[0][1] - 10),
        #                     cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        #         cv2.putText(frame, f"({marker_corners_int[0][0]}, {marker_corners_int[0][1]})",
        #                     (marker_corners_int[0][0], marker_corners_int[0][1] + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
        #                     (0, 0, 0), 2)
        #         # Draw marker corners
        #         for corner in marker_corners_int:
        #             cv2.circle(frame, (corner[0], corner[1]), 3, (0, 0, 255), -1)

        # Draw detected markers
        new_polygon =[]
        if len(detected_markers) > 0:
            for marker_id, marker_corners in detected_markers:
                # Convert corner coordinates to NumPy array
                marker_corners_np = np.array(marker_corners)

                print(marker_corners_np)
                # Determine specific corners based on ArUco marker ID
                if marker_id == 1:
                    # Bottom left corner
                    corner = marker_corners_np[0]
                elif marker_id == 2:
                    # Top left corner
                    corner = marker_corners_np[1]
                elif marker_id == 3:
                    # Bottom right corner
                    corner = marker_corners_np[3]
                elif marker_id == 4:
                    # Top right corner
                    corner = marker_corners_np[2]
                else:
                    continue  # Skip this marker if ID is not 1, 2, 3, or 4

                new_polygon.append(corner)

                # Display the detected markers and specific corners
                cv2.polylines(frame, [np.int32(marker_corners)], True, (0, 255, 0), 2)
                cv2.circle(frame, (int(corner[0]), int(corner[1])), 5, (0, 0, 255), -1)

        # Reverse the order of the last two elements in new_polygon
        if len(new_polygon) >= 2:
            new_polygon[-2:] = new_polygon[-2:][::-1]

        ZONE_POLYGON = np.array(new_polygon)
        # Save ZONE_POLYGON to a file
        np.savetxt('zone_polygon.txt', ZONE_POLYGON.astype(int), fmt='%d')

        cv2.polylines(frame, [np.int32(ZONE_POLYGON)], True, (0, 255, 0), 2)

        print(new_polygon)

        cv2.imshow('Detected Arucos', frame)

        # Save the frame and exit
        cv2.imwrite('webcam_frame.jpg', frame)


        #if cv2.waitKey(1) & 0xFF == ord('q'):
        break

    # Release the webcam
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
