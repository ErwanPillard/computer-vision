import cubeDetection
import mapDetection

def main():
    ZONE_POLYGONE = mapDetection.main()
    cubeDetection.main(ZONE_POLYGONE)

if __name__ == '__main__':
    main()
