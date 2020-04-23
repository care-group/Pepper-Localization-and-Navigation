#Marker recognition, Localization and Navigation
#Chengkun Liu H00263940
import os
import sys
import time
from naoqi import ALProxy
from cv2 import aruco
import cv2
import numpy as np
import math
import glob

# Store the information of the img
def detect_markers(img):
        aruco_list = []

        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        aruco_dict = aruco.Dictionary_get(aruco.DICT_5X5_100)
        parameters = aruco.DetectorParameters_create()
        
        # get the id of the marker
        corners, ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters = parameters)
        print(corners)
        print("\n")
        print(ids)
        centrecoord=[]
        if ids!=None:
            L=len(ids)
 
            # Calculate the center point of the marker
            for i in range(0,L):
                    x=int(corners[i][0][0][0]+corners[i][0][1][0]+corners[i][0][3][0]+corners[i][0][2][0])/4
                    y=int(corners[i][0][0][1]+corners[i][0][1][1]+corners[i][0][3][1]+corners[i][0][2][1])/4
                    p=(x,y)
                    print(p)
                    centrecoord.append(p)
            for i in range(0,L):
                    p=(ids[i][0],centrecoord[i],corners[i])#,rvec[i],tvec[i]
                
                    aruco_list.append(p)
        
        return aruco_list

# Show the marker and frame it with a square
def show_aruco(img,aruco_list):
    for x in aruco_list:
            centrecoord,corners = x[1],x[2]
            point_color = (0, 255, 0) # BGR
            thickness = 1
            lineType = 4
            startpoint = (corners[0][0][0],corners[0][0][1])
            stoppoint = (corners[0][1][0],corners[0][1][1])
            cv2.line(img, startpoint, stoppoint, point_color, thickness, lineType)
            startpoint = (corners[0][1][0],corners[0][1][1])
            stoppoint = (corners[0][2][0],corners[0][2][1])
            cv2.line(img, startpoint, stoppoint, point_color, thickness, lineType)
            startpoint = (corners[0][2][0],corners[0][2][1])
            stoppoint = (corners[0][3][0],corners[0][3][1])
            cv2.line(img, startpoint, stoppoint, point_color, thickness, lineType)
            startpoint = (corners[0][3][0],corners[0][3][1])
            stoppoint = (corners[0][0][0],corners[0][0][1])
            cv2.line(img, startpoint, stoppoint, point_color, thickness, lineType)

    return img

# Calculate the robot position by the inverse perspective transformation algorithm
def perspective(length_2,length_4,pos_mark,changeY):
    
    # c is the actual side length and k is the weight coeffcient
    c = 17.
    k = 22613.
    a = k/length_4
    b = k/length_2

    theat = math.acos((b*b+c*c-a*a)/(2*b*c))

    x = b*math.sin(theat)
    y = b*math.cos(theat)-c/2

    theat_1 = math.pi/2-math.atan(x/y)
    angle = theat_1-math.atan(changeY/1.)
    position = [x,y,angle] + pos_mark[:]

    return position

# Calculate the more accurate robot position by the kalman filter
def kalman_filter(obs_vl):

    Q = 1.
    R = 1.
    num = len(obs_vl)

    X = [[1.,1.]]*(num+1)
    P = [[10.,10.]]*(num+1)
    Xp1 = [[0.,0.]]*num
    Pp1 = [[0.,0.]]*num
    Kg = [[0.,0.]]*num

    for i in range(0,num):
        Xp1[i]=X[i]
        Pp1[i][0] = P[i][0] + Q
        Pp1[i][1] = P[i][1] + Q
        Kg[i][0] = Pp1[i][0]/(P[i][0]+R)
        Kg[i][1] = Pp1[i][1]/(P[i][1]+R)
        X[i+1][0] = Xp1[i][0]+Kg[i][0]*(obs_vl[i][0]-Xp1[i][0])
        X[i+1][1] = Xp1[i][1]+Kg[i][1]*(obs_vl[i][1]-Xp1[i][1])
        P[i+1][0] = (1-Kg[i][0])*P[i][0]
        P[i+1][1] = (1-Kg[i][1])*P[i][1]
    location = np.mean(X[int(num*0.9):num-1], axis =0)

    return location

# navigate to the target position
def navigating(navigation_service,path,location,target_loc):
    
    navigation_service.startLocalization()
    navigation_service.loadExploration(path)

    time.sleep(10)
    # Relocalize the robot and start the localization process.

    navigation_service.relocalizeInMap(location)
    # Navigate to another place in the map
    navigation_service.navigateToInMap(target_loc)

    # Check where the robot arrived
    print "I reached: " + str(navigation_service.getRobotPositionInMap()[0])

    # Stop localization
    navigation_service.stopLocalization()

# Get the length of four sides    
def aruco_data(aruco_list):
    data_list = []
    for i in aruco_list:
        corners = i[2]
        length_1 = math.sqrt((corners[0][1][0]-corners[0][0][0])**2 + (corners[0][1][1]- corners[0][0][1])**2)
        length_2 = math.sqrt((corners[0][2][0]-corners[0][1][0])**2 + (corners[0][2][1]- corners[0][1][1])**2)
        length_3 = math.sqrt((corners[0][3][0]-corners[0][2][0])**2 + (corners[0][3][1]- corners[0][2][1])**2)
        length_4 = math.sqrt((corners[0][0][0]-corners[0][3][0])**2 + (corners[0][0][1]- corners[0][3][1])**2)
        center = i[1]
        p = (length_1,length_2,length_3,length_4,center)

        data_list.append(p)
    return data_list

"""
Function Name : MAIN CODE
"""
if __name__=="__main__":
        IP = "192.168.1.121"
        PORT = 9559

# Create a proxy to ALPhotoCapture
        try:
                vision = ALProxy("ALVideoDevice", IP, PORT)
                tracker = ALProxy("ALTracker", IP, PORT)
                motion = ALProxy("ALMotionProxy", IP, PORT)
                navigation_service =ALProxy("ALNavigation", IP, PORT)
        except Exception, e:
            print "Error when creating ALPhotoCapture proxy:"
       
        #Specify the head orientation of the initial robot
        pos = [1.,0.,0.5]
        
        # Stop the other tracker
        tracker.stopTracker()
        tracker.unregisterAllTargets()
        trac = tracker.getActiveTarget()
        print(trac)

        # Create an empty array
        aruco_list = []
        # Store the aruco marker photos' information  in the aruco_list 
        while aruco_list == []:
            tracker.lookAt(pos,0,0.5,False)
            time.sleep(1)
            img_data = vision.getImageRemote("cam1_0")
            if img_data == None:
                check = vision.subscribeCamera("cam1",0,3,13,15)
                img_data = vision.getImageRemote("cam1_0")
            imageArray = np.array(bytearray(img_data[6]))
            img = imageArray.reshape([img_data[1],img_data[0],img_data[2]])
            # Store the information of the marker in the aruco_list
            aruco_list = detect_markers(img)
            # Show the img in your computer
            img = show_aruco(img,aruco_list)
            # Store the four sides in data_list
            data_list = aruco_data(aruco_list)
                #img = drawAxis(img, aruco_list, i[0], cam, dist)
                #img = drawCube(img, aruco_list, i[0], cam, dist)
                #img = drawCylinder(img, aruco_list, i[0], cam, dist)
            print(data_list)
            #if the marker is not recognized, the robot will turn pi/4
            motion.moveTo(0.,0.,math.pi/4)
            #cv2.imshow("img", img)
        
        # Create an empty array 
        position_list = []
        # Initializing the orientation of the robot
        changeY = 0
        changeZ = 0.5
        tracker.lookAt(pos,0,0.5,False)
        # Store 10 sets of data
        while len(position_list)<10 :
            # Adjust the marker to the center of vision of the robot
            if aruco_list != []:
                changeY -= (aruco_list[0][1][0]-(1280/2))*0.0005
                changeZ -= (aruco_list[0][1][1]-(960/2))*0.0003
                pos = [1.,changeY,changeZ]
                print([aruco_list[0][1][0]-(1280/2),aruco_list[0][1][1]-(960/2)])
            tracker.lookAt(pos,0,0.5,False)
            time.sleep(2)
            img_data = vision.getImageRemote("cam1_0")
            print(pos)
            imageArray = np.array(bytearray(img_data[6]))
            img = imageArray.reshape([img_data[1],img_data[0],img_data[2]])
            aruco_list = detect_markers(img)
            data_list = aruco_data(aruco_list)
            print(data_list)
            if aruco_list[0][1][0]-(1280/2)<10 and aruco_list[0][1][1]-(960/2)<10:
                # Different Id marker have the different position. The user need write based on the physical truth.
                if aruco_list[0][0] == 1:
                    pos_mark = [100.,200.,0.]
                elif aruco_list[0][0] == 3:
                    pos_mark = [100.,200.,0.]
                elif aruco_list[0][0] == 6:
                    pos_mark = [100.,200.,0.]

                # Calculate the robot position by the inverse perspective transformation algorithm
                position = perspective(data_list[0][1],data_list[0][3],pos_mark,changeY)
                position_list.append(position)
        
        # Calculate the more accurate robot position by the kalman filter
        location = kalman_filter(position_list)
        print(location)
        
        # Import from the first file
        path = ""
        # Write the target position
        target_loc = [0.,0.,0.]
        # navigate to the target position
        navigating(navigation_service,path,location,target_loc)

        cv2.waitKey(0)
        cv2.destroyAllWindows()
