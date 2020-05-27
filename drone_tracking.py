from os import sys, path
sys.path.append(path.dirname(path.dirname(path.abspath(__file__))))

import time
import math
from aruco_track_lib import *
import ps_drone

##################
# Help functions #
##################

# Converting marker position to angles in radians

def marker_position_to_angle(x, y, z):
    angle_x = math.atan2(x,z)
    angle_y = math.atan2(y,z)
    return (angle_x, angle_y)

def check_descend_angle(angle_x, angle_y, angle_to_descend):
    return (math.sqrt(angle_x ** 2 + angle_y ** 2) <= angle_to_descend)

def getAltitudeChange(currentAltitude, hoverAltitude, hoverDeviation):
    if (currentAltitude - hoverAltitude) > hoverDeviation:
        return -0.1
    elif (altitude - hoverAltitude) < -hoverDeviation:
        return 0.1
    else: return 0.0

################
# Drone setup  #
################

# Drone inicialization
drone = ps_drone.Drone()
drone.startup()

drone.reset()

while (drone.getBattery()[0] == -1):
    time.sleep(0.1)
print "Battery: " + str(drone.getBattery()[0]) + "% " + str(drone.getBattery()[1])

if drone.getBattery()[0] < 15:
    print "Battery too low!"
    sys.exit()

drone.useDemoMode(False)
drone.getNDpackage(["demo","pressure_raw","altitude","magneto","wifi"])

drone.setConfigAllID()
drone.sdVideo()

drone.groundCam()
CDC = drone.ConfigDataCount
while CDC == drone.ConfigDataCount: time.sleep(0.0001)
NDC = drone.NavDataCount

#############
# Variables #
#############

marker_id = 0
marker_size = 13.4 # cm

rad_2_deg = 180/math.pi
deg_2_rad = 1.0/rad_2_deg

# Tracker module variables
cwd = path.dirname(path.abspath(__file__))
calib_path = cwd + "/image_data/"
camera_matrix = np.loadtxt(calib_path + 'cameraMatrix.txt', delimiter = ',')
camera_distortion = np.loadtxt(calib_path + 'cameraDistortion.txt', delimiter = ',')
aruco_tracker = ArucoSingleTracker(marker_id, marker_size, camera_matrix, camera_distortion, drone) 

# Navigation variables
angle_to_descend = 5 * deg_2_rad
timeCounter = time.time()
hoverAltitude = 1050 # mm
hoverDeviation = 50 # mm
altitudeChange = 0.0
directionX = 0.0
directionY = 0.0
maxSpeed = 0.012
screenSize = [640, 360]
counter = 0

# Bool variables
printAltitude = False
isAboveMarker = False
do_hover = False
do_marker_hover = False
stop = False
do_marker_land = False
do_marker_track = False


#############
# Main Loop #
#############

while not stop:
    string = ""

    # Get naviation data count for drone
    while drone.NavDataCount == NDC:
        time.sleep(0.001)
    NDC = drone.NavDataCount

    # Get altitude data (in mm)
    altitude = drone.NavData["altitude"][3]

    # Perform marker tracking
    if aruco_tracker.track(loop = False) == None:
        # EMERGENCY LANDING
        stop = True

    marker_found, marker_x, marker_y, marker_z = aruco_tracker.track(loop=False)

    #----------------#
    # Keyboard setup #
    #----------------#

    key = drone.getKey()

    if key == " ":
        print "Takeoff toggle"
        if drone.NavData["demo"][0][2] and not drone.NavData["demo"][0][3]:
            drone.takeoff()
        else:
            drone.land()
    elif key == "0":
        do_marker_hover = False
        do_marker_land = False
        do_marker_track = False
        do_hover = False
        drone.hover()
    elif key == "8":
        drone.moveForward()
    elif key == "2":
        drone.moveBackward()
    elif key == "4":
        drone.moveLeft()
    elif key == "6":
        drone.moveRight()
    elif key == "p":
        if printAltitude == False:
            printAltitude = True
        else: printAltitude = False
    elif key == "a":
        do_hover = True
    elif key == "t":
        do_marker_track = True
    elif key == "h":
        do_marker_track = True
        do_marker_hover = True
        do_marker_land = False
    elif key == "l":
        do_marker_track = True
        do_marker_land = True
        do_marker_hover = False
    elif key != "":
        stop = True

    # DEBUG - printing altitude
    if printAltitude:
        print "Altitude: "  + str(altitude)
        print "Altutude difference: " + str(hoverDeviation)
    
    #-----------------#
    # Marker tracking #
    #-----------------#

    if marker_found and do_marker_track:
        #print "x " + str(marker_x) + "y" + str(marker_y)

        # converting marker positions to angles 
        angle_x, angle_y = marker_position_to_angle(marker_x, marker_y, marker_z)

        # checking, if drone is in zone above marker
        if check_descend_angle(angle_x, angle_y, angle_to_descend):
            isAboveMarker = True
            #print "IS ABOVE"

        # do every frame
        if (counter % 1) == 0:
           
            #----------------#
            # PID Controller #
            #----------------#
            
            # Calculating directional values for drone
            if marker_x < 0:
                directionX = -(((screenSize[0]/2) - marker_x)/20800) #right
            elif marker_x > 0:
                directionX = (((screenSize[0]/2) - marker_x)/20800) #left
            if marker_y < 0:
                directionY = (((screenSize[1]/2) - marker_y)/11700) #forward
            elif marker_y > 0:
                directionY = -(((screenSize[1]/2) - marker_y)/11700) #backwards

            # Check for max/min speed
            if directionX > maxSpeed:
                directionX = maxSpeed
            elif directionX < -maxSpeed:
                directionX = -maxSpeed
            if directionY > maxSpeed:
                directionY = maxSpeed
            elif directionY < -maxSpeed:
                directionY = -maxSpeed
            
            # Calculating altitude values for drone
            altitudeChange = 0.0
            altitudeChange =  getAltitudeChange(altitude, hoverAltitude, hoverDeviation)

            #----------------#

            # Hover above marker
            if do_marker_hover:
                if not isAboveMarker:
                    drone.move(directionX, directionY, altitudeChange, 0)
                else:
                    if altitudeChange != 0.0:
                        drone.move(0, 0, altitudeChange, 0)
                    else: drone.stop()

            # Land on marker
            if do_marker_land:
                if not isAboveMarker:
                    drone.move(directionX, directionY, 0, 0)
                else:
                    drone.stop()
                    if altitude > 500:
                        drone.move(0, 0, -0.2, 0)
                    else:
                        drone.land()


    # If drone looses marker, but is set to hover above marker
    elif not marker_found and do_marker_hover:
        drone.stop()

    # If drone looses marker, but is set to land on marker
    elif not marker_found and do_marker_land:
        drone.stop()
        if altitude <= 500:
            drone.land()

    #------------------------#
    # END of marker tracking #
    #------------------------#

    # Get to hoverAltitude
    if (counter % 1) == 0 and do_hover:
        altitudeChange =  getAltitudeChange(altitude, hoverAltitude, hoverDeviation)

        if altitudeChange != 0.0:
            drone.move(0, 0, altitudeChange, 0)
        else:
            drone.stop()
    counter += 1

# Destroy all opencv windows
cv2.destroyAllWindows()
