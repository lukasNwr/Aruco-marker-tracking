import numpy as np
import cv2
import cv2.aruco as aruco
import sys, time, math
import ps_drone

class ArucoSingleTracker():
    def __init__(self,
                id_to_find,
                marker_size,
                camera_matrix,
                camera_distortion,
                drone,
                camera_size=[640,360],
                show_video=True,
                ):
        
        # Detection variables
        self.id_to_find     = id_to_find
        self.marker_size    = marker_size
        self._show_video    = show_video
        self._drone = drone

        # Camera calibration files
        self._camera_matrix = camera_matrix
        self._camera_distortion = camera_distortion
        
        # Run variables
        self.is_detected    = False
        self._kill          = False
        
        # Dictionary settigns
        self._aruco_dict  = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)

        # Parameters settigns
        self._parameters  = aruco.DetectorParameters_create()

        #Thresholding parameters
        self._parameters.adaptiveThreshWinSizeMin = 3
        self._parameters.adaptiveThreshWinSizeMax = 20
        self._parameters.adaptiveThreshWinSizeStep = 10

        # Countour filtering parameters
        self._parameters.minMarkerPerimeterRate = 0.03
        self._parameters.maxMarkerPerimeterRate = 5.0
        self._parameters.polygonalApproxAccuracyRate = 0.05

        # Corner Refinement parameters
        self._parameters.cornerRefinementMethod = CORNER_REFINE_APRILTAG

        # Axis rotation
        self._R_flip      = np.zeros((3,3), dtype=np.float32)
        self._R_flip[0,0] = 1.0
        self._R_flip[1,1] =-1.0
        self._R_flip[2,2] =-1.0

        # Starign drone camera stream
        self._drone.startVideo()

        # Font for the text in the image
        self.font = cv2.FONT_HERSHEY_PLAIN

    #--------------#
    # Euler angles #
    #--------------#

    def _rotationMatrixToEulerAngles(self,R):
        def isRotationMatrix(R):
            Rt = np.transpose(R)
            shouldBeIdentity = np.dot(Rt, R)
            I = np.identity(3, dtype=R.dtype)
            n = np.linalg.norm(I - shouldBeIdentity)
            return n < 1e-6        
        assert (isRotationMatrix(R))

        sy = math.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])

        singular = sy < 1e-6

        if not singular:
            x = math.atan2(R[2, 1], R[2, 2])
            y = math.atan2(-R[2, 0], sy)
            z = math.atan2(R[1, 0], R[0, 0])
        else:
            x = math.atan2(-R[1, 2], R[1, 1])
            y = math.atan2(-R[2, 0], sy)
            z = 0

        return np.array([x, y, z])

    # Stop function
    def stop(self):
        self._kill = True

    #--------------#
    # Track method #
    #--------------#

    def track(self, loop=True, show_video=None):
        
        self._kill = False
        if show_video is None: show_video = self._show_video
        
        marker_found = False
        x = 0
        y = 0
        z = 0

        # Get number of encoded frames from drone
        IMC = self._drone.VideoImageCount
        
        # Main loop
        while not self._kill:

            # Update number of frames 
            while self._drone.VideoImageCount == IMC: time.sleep(0.01) # Wait for next frame
            IMC = self._drone.VideoImageCount

            # Get video image from drone
            frame = self._drone.VideoImage

            # Convert image to grayscale
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) 

            # Detect aruco markers
            corners, ids, rejected = aruco.detectMarkers(image=gray, dictionary=self._aruco_dict, 
                            parameters=self._parameters,
                            cameraMatrix=self._camera_matrix, 
                            distCoeff=self._camera_distortion)
                            
            if not ids is None and self.id_to_find in ids[0]:

                marker_found = True

                # Perform aruco pose estimation
                ret = aruco.estimatePoseSingleMarkers(corners, self.marker_size, self._camera_matrix, self._camera_distortion)

                # Get rotation and postition of first marker 
                rvec, tvec = ret[0][0,0,:], ret[1][0,0,:]
                
                x = tvec[0]
                y = tvec[1]
                z = tvec[2]

                # Draw the marker
                aruco.drawDetectedMarkers(frame, corners)

                # Draw axis over marker
                aruco.drawAxis(frame, self._camera_matrix, self._camera_distortion, rvec, tvec, 10)

                # Calculate ratation matrix 
                rotationMatrix    = np.matrix(cv2.Rodrigues(rvec)[0])
                rotationMatrix_t    = rotationMatrix.T

                # Get the attitude in terms of euler 321 
                roll_marker, pitch_marker, yaw_marker = self._rotationMatrixToEulerAngles(self._R_flip * rotationMatrix_t)

                # Get Position and attitude of the camera respect to the marker
                pos_camera = -rotationMatrix_t * np.matrix(tvec).T

                # Show information about postions and rotations on video 
                if show_video:
                    # Print the tag position in camera frame
                    str_position = "MARKER Position x=%4.0f  y=%4.0f  z=%4.0f"%(tvec[0], tvec[1], tvec[2])
                    cv2.putText(frame, str_position, (0, 100), self.font, 1, (255, 0, 0), 2, cv2.LINE_AA)        
                    
                    # Print the marker's attitude respect to camera frame
                    str_attitude = "MARKER Attitude r=%4.0f  p=%4.0f  y=%4.0f"%(math.degrees(roll_marker),math.degrees(pitch_marker),
                                        math.degrees(yaw_marker))
                    cv2.putText(frame, str_attitude, (0, 150), self.font, 1, (255, 0, 0), 2, cv2.LINE_AA)

                    str_position = "CAMERA Position x=%4.0f  y=%4.0f  z=%4.0f"%(pos_camera[0], pos_camera[1], pos_camera[2])
                    cv2.putText(frame, str_position, (0, 200), self.font, 1, (255, 0, 0), 2, cv2.LINE_AA)

                    # Get the attitude of the camera respect to the frame
                    roll_camera, pitch_camera, yaw_camera = self._rotationMatrixToEulerAngles(self._R_flip * rotationMatrix)
                    str_attitude = "CAMERA Attitude r=%4.0f  p=%4.0f  y=%4.0f"%(math.degrees(roll_camera),math.degrees(pitch_camera),
                                        math.degrees(yaw_camera))
                    cv2.putText(frame, str_attitude, (0, 250), self.font, 1, (255, 0, 0), 2, cv2.LINE_AA)


            

            if show_video:
                #--- Display the frame
                cv2.imshow('frame', frame)
                cv2.waitKey(1)

                #--- use 'q' to quit
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    cv2.destroyAllWindows()
                    self._kill = True
                    break
            
            if not loop: return(marker_found, x, y, z)

