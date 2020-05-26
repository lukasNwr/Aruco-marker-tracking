# ArucoMarkerTacking

Tracking of ArUco markers using OpenCV library and Parrot AR Drone 2.0. API for drone is PS-Drone made by  J. Philipp de Graaff - http://www.playsheep.de/drone/

## Modules
**ps_drone.py** - Library file of PS-Drone for controling the Parrot AR Drone 2.0.</br>
**create_markers.py** - Creates marker images.</br>
**create_calibration_images.py** - Uses camera to save images of calibration chessboard for calibration.</br>
**camera_calibration.py** - Handles camera calibration.</br>
**aruco_track_lib.py** - Performs marker detection and pose estimation.</br>
**drone_trcking.py** - Uses aruco_track_lib.py and ps_drone.py for detecting marker and navigating drone.</br>

## Folders
**image_data** - Contains calibration data and calibration images.</br>
**marker_images** - Contains marker images from dictionary (DICT_4x4_50).</br>
