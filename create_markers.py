import cv2
import cv2.aruco as aruco

def createArucoMarkers():
    dictionary = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
    for i in range(50):
        marker = aruco.drawMarker(dictionary, i, 500, 1)
        markerName = "./marker_images/4x4marker_" + str(i) + ".jpg"
        cv2.imwrite(markerName, marker)

def main():
    createArucoMarkers()

if __name__ == "__main__":
    main()

