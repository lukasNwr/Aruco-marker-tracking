# ArucoMarkerTacking

Tracking of ArUco markers using OpenCV library and Parrot AR Drone 2.0. API for drone is PS-Drone made by  J. Philipp de Graaff - http://www.playsheep.de/drone/

## Modules
**ps_drone.py** - Library file of PS-Drone for controling the Parrot AR Drone 2.0.\
**create_markers.py** - Creates marker images.\
**create_calibration_images.py** - Uses camera to save images of calibration chessboard for calibration.\
**camera_calibration.py** - Handles camera calibration.\
**aruco_track_lib.py** - Performs marker detection and pose estimation.\
**drone_trcking.py** - Uses aruco_track_lib.py and ps_drone.py for detecting marker and navigating drone.

## Folders
**image_data** - Contains calibration data and calibration images.\
**marker_images** - Contains marker images from dictionary (DICT_4x4_50).
