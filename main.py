from Perception import mapDetection
from Perception.cubeDetectionFromIMG import cube_detection_inZone
from Trajectory.robotToCube import robotToCube
import numpy as np


#from cubeDetection import cube_detection_inZone


def main():
    #ZONE_POLYGONE = mapDetection.main()
    # print("\nZone polygone: \n", ZONE_POLYGONE)
    # cube_detection_inZone(ZONE_POLYGONE)

    #Map detection
    ZONE_POLYGONE, frame = mapDetection.main()
    ZONE_POLYGON = np.array([
        [1035, 685],
        [215, 640],
        [254, 103],
        [1041, 136]
    ], dtype=int)

    print("\nZone polygone: \n", ZONE_POLYGONE)

    #Cube detection in zone

    #return x,y
    center_coordinates = cube_detection_inZone(ZONE_POLYGONE, frame)
    print("\nCube x: \n", center_coordinates[0], " y \n", center_coordinates[1])

    robotToCube(center_coordinates, frame)


if __name__ == '__main__':
    main()
