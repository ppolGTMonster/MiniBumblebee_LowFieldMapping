####################################################################################################################
#   Class Definition of: 
#       * MarkerReferenceBoard
#           One Marker on the ReferenceBoard
#           The object knows his position/rotation on the board
#       * ReferenceBoard
#           The ReferenceBoard itself
#           Parameter: List of detected Markers + calcualted Origin Coor Sys
#       * MarkerOnRobot
#           One Marker on the Robot
#           Parameter: List of detected Positions
#       * Robot_ArucoMarker
#           The sum of all 4 Aruco-Marker
#       * RobotCybernetics
#           Object to describe the Kinematiks of the real robot (distances between marker/joints etc)
#
#   Copyright (c) 2024 Pavel Povolni, Tuebingen, Germany
#   MIT LICENSED  
#   Have fun guys!
####################################################################################################################

import cv2
import cv2.aruco as aruco
import numpy as np
import math
import socket
from Functions_MatrixCalculation import meanRotationMatrix,buildTransformationMatrix

class MarkerReferenceBoard:
    id = 0                      # ID of the marker
    T_RefBoard = np.zeros(3)    # Translation on Referenceboard relative to Origin Base CS)
    R_RefBoard = np.zeros((3,3)) # Rotation  on Referenceboard relative to Origin Base CS)

    # Lookup Table for used Reference Board in this work
    mk0_T = np.array([340,40,0])  * 1e-3    # Translation
    mk1_T = np.array([340,130,0]) * 1e-3
    mk2_T = np.array([340,230,0]) * 1e-3
    mk3_T = np.array([240,230,0]) * 1e-3
    mk4_T = np.array([140,230,0]) * 1e-3
    mk5_T = np.array([40,230,0]) * 1e-3
    mk6_T = np.array([60,-48,60]) *1e-3

    mk0_rot_angle = 0           # Rotation
    mk1_rot_angle = 0
    mk2_rot_angle = +1*np.pi/4
    mk3_rot_angle = 0
    mk4_rot_angle = -1* np.pi/4
    mk5_rot_angle = 0
    mk6_rot_angle = +1*np.pi/4


    def __init__(self, set_id):
        #Object constructor (setting ID and therefore T & R are set)
        
        self.id = set_id

        if set_id == 0:
            T_set = self.mk0_T
            R_set = self.mk0_rot_angle
        elif set_id == 1:
            T_set = self.mk1_T
            R_set = self.mk1_rot_angle
        elif set_id == 2:
            T_set = self.mk2_T
            R_set = self.mk2_rot_angle
        elif set_id == 3:
            T_set = self.mk3_T
            R_set = self.mk3_rot_angle
        elif set_id == 4:
            T_set = self.mk4_T
            R_set = self.mk4_rot_angle
        elif set_id == 5:
            T_set = self.mk5_T
            R_set = self.mk5_rot_angle
        elif set_id == 6:
            T_set = self.mk6_T
            R_set = self.mk6_rot_angle

        else:
            T_set = self.mk0_T
            R_set = self.mk0_rot_angle
            print('error -nonid selected')

        self.T_RefBoard = T_set
        # Rotation is just a rotation around z-axis of BCS
        self.R_RefBoard = np.array([[math.cos(R_set) ,-math.sin(R_set)   ,0],
                                    [math.sin(R_set) ,math.cos(R_set)    ,0],
                                    [0               ,0                  ,1]])

class ReferenceBoard:
    Rmatrix = np.zeros([3,3])
    Tvec = np.zeros(3)

    R_avg = np.zeros([3,3])
    T_avg = np.zeros(3)

    T_avg_list=[]
    R_avg_list=[]

    # Set Marker Info
    Marker_length = 0.08
    ArucoDict = aruco.Dictionary_get(aruco.DICT_5X5_50)
    ArucoParams = aruco.DetectorParameters_create()

    mk = []  # List of all markers on the board
    #ID_list = [0,1,2,3,4,5,6] # This is the ideal setup
    #However in the last measurement ID0 and ID1 were partially hidden by the spring
    ID_list = [2,3,4,5,6]  # therefore set this 



    def __init__(self):
        # self.mk.append(MarkerReferenceBoard(0)) # ID 0 and 1 hidden by the spring
        # self.mk.append(MarkerReferenceBoard(1))
        self.mk.append(MarkerReferenceBoard(2))
        self.mk.append(MarkerReferenceBoard(3))
        self.mk.append(MarkerReferenceBoard(4))
        self.mk.append(MarkerReferenceBoard(5))
        self.mk.append(MarkerReferenceBoard(6))

    def getMarker(self,id_goal):
        #get Marker Object (definded by ID)
        item = self.ID_list.index(id_goal)
        mktemp = self.mk[item] 
        return mktemp

    def getT(self,id_goal):
        #get Translation of one marker (defined by ID)
        mktemp = self.getMarker(id_goal)       
        return mktemp.T_RefBoard

    def getR(self,id_goal):
        #get Rotation of one marker (defined by ID)
        mktemp = self.getMarker(id_goal)
        return mktemp.R_RefBoard
    
    def resetBoard(self):
        # Clear Aruco Board. Camera is standing still
        # -> Position is not changing (moving averaging could be done here too)
        # However clear the list, but keep the calculated former average as the first value 
        
        self.T_avg_list=[]
        self.R_avg_list=[]
        self.T_avg_list.append(self.T_avg.copy())
        self.R_avg_list.append(self.R_avg.copy())

        self.Rmatrix = np.zeros([3,3])
        self.Tvec = np.zeros(3)
        self.R_avg = np.zeros([3,3])
        self.T_avg = np.zeros(3)     

    def saveT_R_avg_CamSys(self):
        #
        self.T_avg_list.append(self.Tvec.copy())
        self.R_avg_list.append(self.Rmatrix.copy())

    def calculateT_R_avg_CamSys(self):
        #Calculate Average of all appended Translation/Rotations
        self.T_avg = sum(self.T_avg_list)/len(self.T_avg_list) # Arith Average
        self.R_avg = meanRotationMatrix(self.R_avg_list) # Averaging through Quaternions

    def drawAxis(self, image, cam_mtx, cam_dist):
        #draw coor sys at BCS (Average T & R)
        image = aruco.drawAxis(image, cam_mtx,cam_dist,self.R_avg, self.T_avg, self.Marker_length)
        return image
    
    def drawAxis_Debugging(self, aruco, image, cam_mtx, cam_dist):
        # not used anymore - but remembering the good old times ;)
        image = aruco.drawAxis(image, cam_mtx,cam_dist,self.Rmatrix, self.Tvec, self.Marker_length)
        return image

class MarkerOnRobot:
    
    def __init__(self, set_id):
        self.id = set_id # id of  the marker
        self.detected = 0 # 0 = not detected, 1=detected

        # Translation/Rotation in the Camera CS
        self.T_temp = np.zeros(3)
        self.R_temp = np.zeros([3,3])

        # averaged coordinates in the Camera CS
        self.T_avg_list=[]
        self.R_avg_list=[]
        self.T_avg = np.zeros(3)
        self.R_avg = np.zeros([3,3])

        # Transformation of averaged position into Base CS (from Refernce Board)
        self.T_2RefBoard = np.zeros(3)
        self.R_2RefBoard = np.zeros([3,3])


    def appendListAvg_R_T(self,id_det):
        # Add detected T_temp & R_temp into the list
        if id_det == self.id:
            T_temp_temp = self.T_temp.copy()
           
            R_temp_temp = self.R_temp.copy()
            
            self.T_avg_list.append(T_temp_temp)
            self.R_avg_list.append(R_temp_temp)
        

    def resetAll(self):
        # Reset everything & prepare the next measurement
        self.detected = 0 # 0 = not detected, 1 = detected
        # Translation/Rotation im Kamera-Koorsystem
        self.T_temp = np.zeros(3)
        self.R_temp = np.zeros([3,3])

        # averaged Position in Camera CS
        self.T_avg = np.zeros(3)
        self.R_avg = np.zeros([3,3])
        self.T_avg_list=[]
        self.R_avg_list=[]

        # averaged Position in Base CS
        self.T_2RefBoard = np.zeros(3)
        self.R_2RefBoard = np.zeros([3,3])

    def drawAxis(self, image, cam_mtx, cam_dist, length):
        #Draw Axis of this Marker on the image
        image = aruco.drawAxis(image, cam_mtx,cam_dist,self.R_avg, self.T_avg, length)
        return image
    
    def drawAxis_Debugging(self, aruco, image, cam_mtx, cam_dist, length):
        # see you buddy, not used anymore
        image = aruco.drawAxis(image, cam_mtx,cam_dist,self.R_temp, self.T_temp, length)
        return image

class Robot_ArucoMarker:
    ID_list = [0,1,2,3]     # All Markers, which are mounted to the robot
    Marker_length = 0.03    # Meter, sidelength of the marker  

    # Used Aruco Dictionary
    ArucoDict = aruco.Dictionary_get(aruco.DICT_4X4_50)
    ArucoParams = aruco.DetectorParameters_create()

    mk = []  # List of all marker on robot

    def __init__(self):
        # constructor: Initiliaze the 4 mounted marker to the list
        self.mk.append(MarkerOnRobot(0))
        self.mk.append(MarkerOnRobot(1))
        self.mk.append(MarkerOnRobot(2))
        self.mk.append(MarkerOnRobot(3))

    def getMarker(self,id_goal):
        # get one Marker Object (def by ID)
        item = self.ID_list.index(id_goal)
        mktemp = self.mk[item] 
        return mktemp

    def getT_camSys(self,id_det):
        #get T & R in Camera CS of one marker (def by ID)
        mktemp = self.getMarker(id_det)
        return mktemp.T_temp, mktemp.T_avg

    def getR_camSys(self,id_det):
        #get R & R_avg in Camera CS of one marker (def by ID)
        mktemp = self.getMarker(id_det)      
        return mktemp.R_temp, mktemp.R_avg
    
    def getT_R_RefSys(self,id_det):  
        #get T & R in Base CS of one marker (def by ID)
        mktemp = self.getMarker(id_det)       
        return mktemp.T_2RefBoard, mktemp.R_2RefBoard
 
    def resetMarkers(self):
        #Reset all marker (def by ID)
        for item in range(0,len(self.mk)):
           self.mk[item].resetAll()

    def setT_R_RefSys(self,id_det,T,R):
        # set T & R in Base CS to one marker (def by ID)

        mktemp = self.getMarker(id_det)
        mktemp.T_2RefBoard = T
        mktemp.R_2RefBoard = R

    def setT_R_CamSys(self,id_det,T,R):
        # set T & R in Camera CS to one marker (def by ID)
        mktemp = self.getMarker(id_det)
        mktemp.T_temp = T
        mktemp.R_temp = R
        

    def saveT_R_avg_CamSys(self,id_det):
        #  Add detected T & R into the list of one marker (def by ID)
        item = self.ID_list.index(id_det)
        mktemp = self.mk[item] 
        mktemp.appendListAvg_R_T(id_det)


    def calculateT_R_avg_CamSys(self):
        # Average the positions of all detected markers
        for item in range(0,len(self.mk)):
            mktemp = self.mk[item]            
            if mktemp.detected != 0:
                mktemp.T_avg = sum(mktemp.T_avg_list)/len(mktemp.T_avg_list) # Arith Mean
                mktemp.R_avg = meanRotationMatrix(mktemp.R_avg_list)        # Avearge through Quaternions

    def setDetected(self,id_det):
        # Set if marker was detected
        mktemp = self.getMarker(id_det)
        mktemp.detected=1 

    def getDetected(self,id_det):
        # Ask if marker was detected
        mktemp = self.getMarker(id_det)
        return mktemp.detected 
    
    def allDetected(self):
        # Ask if all 4 markers were detected 
        ret_val = 0
        for item in range(0,len(self.mk)):
            mktemp = self.mk[item]   
            ret_val += mktemp.detected

        if ret_val == 4:
            return True
        else:
            return False
        
    def drawAxis(self, image, cam_mtx, cam_dist):
        length = self.Marker_length/2
        for item in range(0,len(self.mk)):            
            mktemp = self.mk[item]
            image = mktemp.drawAxis(image, cam_mtx, cam_dist, length)
        
        return image
    
    def drawAxis_Debugging(self, aruco, image, cam_mtx, cam_dist):
        # CU later alligator
        length = self.Marker_length/2
        for item in range(0,len(self.mk)):
            mktemp = self.mk[item]
            image = mktemp.drawAxis_Debugging(aruco, image, cam_mtx, cam_dist, length)
        
        return image

class Robot_Cybernetics:
    # In this class all geometric properties are defined (esp distances between markers and joints)

    r_temp =  np.eye(3) # Unity-Matrix [100;010;001]

    ## Marker Offset
    # Marker id1/2/3 are in the same z-plane. id0 is moved a bit along z direction
    id0_offset_z =  -24.596     #mm
    # Center of Marker id1/2/3 is not on axis between the joint-centers but moved a bit along y-direction
    id123_offset_y = -5.493     #mm

    ## Kinematik of the Robot
    # distance for segement 0, where marker 0 is mounted
    l_01 = np.array([71.55,0,id0_offset_z]) *1e-3 #id0-MP -> RotAxis(alpha0)
    l_02 = np.array([0,0,id0_offset_z]) *1e-3     #id0-MP -> RotAxis(alpha1)
    
    # distance for segement 1, where marker 1 is mounted
    l_11 = np.array([-65.617,id123_offset_y,0]) *1e-3 #id1-MP -> RotAxis(alpha2)
    l_12 = np.array([58.733,id123_offset_y,0]) *1e-3 #id1-MP -> RotAxis(alpha1)

    # distance for segement 2, where marker 2 is mounted
    l_21 = np.array([-65.8,id123_offset_y,0]) *1e-3  #id2-MP -> RotAxis(alpha3)
    l_22 = np.array([58.822,id123_offset_y,0]) *1e-3 #id2-MP -> RotAxis(alpha2)

    # distance for segement 3, where marker 3 is mounted
    l_31 = np.array([-237.938,-1.848,-33.592]) *1e-3 # id3-MP -> Hall sensor (Screw front, bottom lower PCB)
    l_32 = np.array([42.746,id123_offset_y,0]) *1e-3 # id3-MP -> RotAxis(alpha3)

    ## Calculated Angles of the robot joints
    alpha0 = 0 # Base Angle (origin)
    alpha1 = 0 # Angle Shoulder
    alpha2 = 0 # Angle Ellbow
    alpha3 = 0 # Angle Wrist

    ## Point 5, calculated out of id3
    P5_affin = np.eye(4)
    P5_eulerangles = []  # order "zyx"
    P5_tvec = []
    # Quaternion used for transfer the rotation to matlab (no gimbal lock) 
    P5_quaternion=[]
    # Usage of all markers for y-axis. Quaternion for transfer to matlab 
    P5_quaternion_advanced = [] 

    # Get Functions
    def get_l01(self):
        return buildTransformationMatrix(self.r_temp,self.l_01)
    
    def get_l02(self):
        return buildTransformationMatrix(self.r_temp,self.l_02)
    
    def get_l11(self):
        return buildTransformationMatrix(self.r_temp,self.l_11)
    
    def get_l12(self):
        return buildTransformationMatrix(self.r_temp,self.l_12)
    
    def get_l21(self):
        return buildTransformationMatrix(self.r_temp,self.l_21)
    
    def get_l22(self):
        return buildTransformationMatrix(self.r_temp,self.l_22)
    
    def get_l31(self):
        return buildTransformationMatrix(self.r_temp,self.l_31)
    
    def get_l32(self):
        return buildTransformationMatrix(self.r_temp,self.l_32)
    