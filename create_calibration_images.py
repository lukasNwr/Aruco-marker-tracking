import cv2
import time
import sys
import argparse
import os
import ps_drone
import time, sys

#################
# Drone startup #
#################

drone = ps_drone.Drone()                                     # Start using drone
drone.startup()                                              # Connects to drone and starts subprocesses

drone.reset()                                                # Sets drone's status to good (LEDs turn green when red)
while (drone.getBattery()[0] == -1):      time.sleep(0.1)    # Waits until drone has done its reset
print "Battery: "+str(drone.getBattery()[0])+"%  "+str(drone.getBattery()[1])	# Gives a battery-statush
drone.useDemoMode(True)                                      # Just give me 15 basic dataset per second (is default anyway)

drone.setConfigAllID()                                       # Go to multiconfiguration-mode
drone.sdVideo()                                              # Choose lower resolution (hdVideo() for...well, guess it)
drone.groundCam()                                             # Choose front view
CDC = drone.ConfigDataCount
while CDC == drone.ConfigDataCount:       time.sleep(0.0001) # Wait until it is done (after resync is done)       
# Start drone video stream
drone.startVideo()

# Get number of encoded frames
IMC = drone.VideoImageCount

###############################
# Saving calibration pictures #
###############################

def save_calibration_pictures(name="calibration_picture", folder="."):

    picture = 0

    fileName    = "%s/%s_" %(folder, name)
    while True:

        # Update number of frames
        while drone.VideoImageCount == IMC: time.sleep(0.01)
        IMC = drone.VideoImageCount

        # Get image from drone
        frame = drone.VideoImage

        # Show image
        cv2.imshow('camera', frame)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        if key == ord(' '):
            print "Saving image ", picture
            cv2.imwrite("%s%d.jpg"%(fileName, picture), frame)
            picture += 1

    cap.release()
    cv2.destroyAllWindows()




def main():

    IMAGE_FOLDER = "./image_data"
    FILE_NAME = "calibration_picture"

    save_snaps(name=FILE_NAME, folder=IMAGE_FOLDER)

    print "Files saved"

if __name__ == "__main__":
    main()


