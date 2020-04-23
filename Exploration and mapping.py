# The code of exploration and mapping
# Chengkun Liu H00263940
import qi
import argparse
import sys
import numpy
from PIL import Image
import time


def main(session):
    """
    This example uses the explore method.
    """
    # Get the services ALNavigation and ALMotion.
    navigation_service = session.service("ALNavigation")
    motion_service = session.service("ALMotion")

    # Wake up robot
    motion_service.wakeUp()
    # Set the safety distance of the robot 
    motion_service.setOrthogonalSecurityDistance(0.03)
    motion_service.setTangentialSecurityDistance(0.02)

    # Explore the environement, in a radius of 4 m.
    radius = 4
    navigation_service.explore(radius)
    # Saves the exploration on disk
    path = navigation_service.saveExploration()
    print "Exploration saved at path: \"" + path + "\""
    # Start localization to navigate in map
    navigation_service.startLocalization()
    # Come back to initial position
    navigation_service.navigateToInMap([0., 0., 0.])
    print "I reached: " + str(navigation_service.getRobotPositionInMap()[0])
    # Stop localization
    navigation_service.stopLocalization()
    # Retrieve and display the map built by the robot
    result_map = navigation_service.getMetricalMap()
    map_width = result_map[1]
    map_height = result_map[2]
    img = numpy.array(result_map[4]).reshape(map_width, map_height)
    img = (100 - img) * 2.55 # from 0..100 to 255..0
    img = numpy.array(img, numpy.uint8)
    Image.frombuffer('L',  (map_width, map_height), img, 'raw', 'L', 0, 1).show()
    
    return path

if __name__ == "__main__":
    # Connect to the robot
    parser = argparse.ArgumentParser()
    parser.add_argument("--ip", type=str, default="192.168.1.121",
                        help="Robot IP address. On robot or Local Naoqi: use '192.168.1.121'.")
    parser.add_argument("--port", type=int, default=9559,
                        help="Naoqi port number")

    args = parser.parse_args()
    session = qi.Session()
    try:
        session.connect("tcp://" + args.ip + ":" + str(args.port))
    except RuntimeError:
        print ("Can't connect to Naoqi at ip \"" + args.ip + "\" on port " + str(args.port) +".\n"
               "Please check your script arguments. Run with -h option for help.")
        sys.exit(1)
    path = main(session)