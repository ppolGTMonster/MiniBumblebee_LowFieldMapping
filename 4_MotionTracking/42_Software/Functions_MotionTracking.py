import cv2
import cv2.aruco as aruco
import numpy as np
from numpy.linalg import inv as npInv
from datetime import datetime
import os
from Functions_MatrixCalculation import meanRotationMatrix, buildTransformationMatrix,getEulerAngles,split_Affine_Matrix,getQuaternion,getRotMatfromEuler

def estimateRefBoard(RefBoard,Board_ids,Board_corners,cam_mtx,cam_dist):
    #Calculation of the Origin of RefBoard for one given frame & all detected Markers on the board

    weight_cum=0

    list_detected_Rotations = []
    list_detected_Tvec = []
    Rmatrix = np.zeros([3,3])

    for i in range(0,len(Board_ids)):

        id_det = Board_ids[i]

        if id_det in RefBoard.ID_list:
            Mk_rvec, Mk_tvec, _ = aruco.estimatePoseSingleMarkers(Board_corners[i],RefBoard.Marker_length,cam_mtx,cam_dist)

            Mk_Rmatrix = cv2.Rodrigues(Mk_rvec[0][0]) # Use Rodrigues Formula to convert Rot-Vector into Rot-Matrix

            # Convert into normal Translation Vector and Rotation
            Mk_tvec = Mk_tvec[0][0]
            Mk_Rmatrix = Mk_Rmatrix[0]

            # get T & R of this Marker on the Ref Board
            temp_mk_getR = RefBoard.getR(id_det)
            temp_mk_getT = RefBoard.getT(id_det)

            #Calculate T & R of the Origin of BCS in the Camera CS from this marker
            temp0 = Mk_Rmatrix@temp_mk_getR@temp_mk_getT
            tvec = Mk_tvec - temp0 
            Rmatrix = Mk_Rmatrix@temp_mk_getR

            # add the determinded translation & rotation to a list, so it can be averaged in the end
            list_detected_Rotations.append(Rmatrix.copy())
            list_detected_Tvec.append(tvec.copy())
            weight_cum += 1 # succesfful detection -> can be used in the Averaging
            
    if weight_cum != 0:  # Averaging
        RefBoard.Tvec = sum(list_detected_Tvec)/len(list_detected_Tvec) # Arith Avg
        RefBoard.Rmatrix = meanRotationMatrix(list_detected_Rotations)  # Using Quaternions
      
        RefBoard.saveT_R_avg_CamSys()
    else:
        RefBoard.Tvec=  np.zeros(3)
        RefBoard.Rmatrix = np.zeros((3,3))
  
def estimateRobotArucos(RobotMarker,RefBoard,Robot_ids,Robot_corners,cam_mtx,cam_dist):
    # estimate position of every robot marker and save it

    for i in range(0,len(Robot_ids)):
        id_det = Robot_ids[i]

        if id_det in RobotMarker.ID_list:
            # Use openCV Function to get position in Camera CS
            Robot_rvec,Robot_tvec,Robot_markerPoints = aruco.estimatePoseSingleMarkers(Robot_corners[i],RobotMarker.Marker_length,cam_mtx,cam_dist)  

            Robot_R,__ = cv2.Rodrigues(Robot_rvec) # Convert Rot-Vector to Rot-Matrix
            Robot_tvec=Robot_tvec[0][0]

            RobotMarker.setDetected(id_det) # Is the detected-flag =1 -> marker was at least once detected
            RobotMarker.setT_R_CamSys(id_det,Robot_tvec,Robot_R)

            RobotMarker.saveT_R_avg_CamSys(id_det) 

def calculateCoorTrafo(RobotMarker,RefBoard):
    # Coordinate Transformation of RobotMarkers from Camera CS into Base CS
    trafo_Board = buildTransformationMatrix(RefBoard.R_avg, RefBoard.T_avg)    

    for id_det in RobotMarker.ID_list:

        if RobotMarker.getDetected(id_det) != 0:
            __,T_avg = RobotMarker.getT_camSys(id_det)
            __,R_avg = RobotMarker.getR_camSys(id_det)

            trafo_Robot = buildTransformationMatrix(R_avg, T_avg)

            # Calculate affine mapping (see publication)
            # CCS = Camera Coordinate System. RCS = Reference Board Coordinate System. MCS = Marker Coordinate System
           
                     
            H_CCS_RCS = trafo_Board         # homg Matrix from RefBoard (RCS) into CCS 
            
            H_CCS_MCS = trafo_Robot         # homog Matrix from MCS into CCS
            H_RCS_CCS = npInv(H_CCS_RCS)    # homog Matrix from CCS into RCS

            H_RCS_MCS = H_RCS_CCS @ H_CCS_MCS       # homog Matrix from MCS into RCS
            Rot_Robot_to_Board = H_RCS_MCS[:3,:3]
            Trans_Robot_to_Board = H_RCS_MCS[:3,3]

            RobotMarker.setT_R_RefSys(id_det,Trans_Robot_to_Board,Rot_Robot_to_Board)

        # else: do nothing

def calculateRealAnglesOfRobot(RobotMarker,RefBoard,RC):
    # Calculate the Joint Angles based on the now known positions of the markers 
    # RC = Object RobotCyber
   
    # Bestimme die Winkel vom Roboter
    if RobotMarker.allDetected() == True: # Nur wenn alle erkannt worden sind -> Mach ich mir einfach an der Stelle. Besser: Robuster machen, falls eins nicht erkannt wird
      
        #get the estimated Translation/Rotation of RobotMarkers in Reference System 
        mk0_t, mk0_r = RobotMarker.getT_R_RefSys(0)
        mk1_t, mk1_r = RobotMarker.getT_R_RefSys(1)
        mk2_t, mk2_r = RobotMarker.getT_R_RefSys(2)
        mk3_t, mk3_r = RobotMarker.getT_R_RefSys(3)

        mk0_r_affin = buildTransformationMatrix(mk0_r,mk0_t)
        mk1_r_affin = buildTransformationMatrix(mk1_r,mk1_t)
        mk2_r_affin = buildTransformationMatrix(mk2_r,mk2_t)
        mk3_r_affin = buildTransformationMatrix(mk3_r,mk3_t)

        mk0_euler = getEulerAngles(mk0_r) # order zyx
        mk1_euler = getEulerAngles(mk1_r)
        mk2_euler = getEulerAngles(mk2_r)
        mk3_euler = getEulerAngles(mk3_r)

        mk0_euler_y = mk0_euler[1]
        mk1_euler_y = mk1_euler[1]
        mk2_euler_y = mk2_euler[1]
        mk3_euler_y = mk3_euler[1]

        ## Calculation alpha0 = Base Angle
        # Here's a calculation using the marker. Used as a backup, but in the end the additional sensor was used
        # Weighting: id3 has biggest deflection around y-axis, and is there the most precise
        # id2 has less and so on 
        
        weight_id3 = 3
        weight_id2 = 2
        weight_id1 = 1
        weight_id0 = 0.5
        weigth_sum = weight_id0 + weight_id1 + weight_id2 + weight_id3

        # Arith Average
        alpha0 = (weight_id3*mk3_euler_y +
                  weight_id2*mk2_euler_y +
                  weight_id1*mk1_euler_y +
                  weight_id0*mk0_euler_y) /weigth_sum
        
        
        ## Calculation of the other joint angles
        #alpha3
        rot_alpha3 = calcRot_withCoorTrafo_asEuler(mk3_r_affin,mk2_r_affin,RC.get_l32(),RC.get_l21())
        alpha3 = rot_alpha3[0]

        #alpha2
        rot_alpha2 = calcRot_withCoorTrafo_asEuler(mk2_r_affin,mk1_r_affin,RC.get_l22(),RC.get_l11())
        alpha2 = rot_alpha2[0]

        #alpha1
        rot_alpha1 = calcRot_withCoorTrafo_asEuler(mk1_r_affin,mk0_r_affin,RC.get_l12(),RC.get_l01())
        alpha1 = rot_alpha1[0]

        ## Set calculated Values
        RC.alpha0 = alpha0
        RC.alpha1 = alpha1
        RC.alpha2 = alpha2
        RC.alpha3 = alpha3
        
    else:
        RC.alpha0 = np.nan
        RC.alpha1 = np.nan
        RC.alpha2 = np.nan
        RC.alpha3 = np.nan
        
def calcRot_withCoorTrafo_asEuler(id_ip1,id_i, l_ip1_2, l_i_1):
    # Calculation of Angle of one joint
    # for explanation of formule see publication

    # affine mapping
    # RCS = Reference CoorSys | RAS = RotAxis CoorSys | idi = id(i) CoorSys | idip1 = id(i+1) CoorSys
    # Convention: e.g. H_s1_s2 : H = homogenous Matrix, s1 = "Basis"-CoorSys (Origin), s2 = "Object"-CoorSys (target)
    H_idip1_RA = l_ip1_2
    H_idi_RA = l_i_1
    H_RCS_idi = id_i
    H_RCS_idip1 = id_ip1

    H_idi_RCS = npInv(H_RCS_idi)
    H_idip1_RCS = npInv(H_RCS_idip1)
    H_RA_idi = npInv(H_idi_RA)

    #mapping between marker id(i+1) and id(i) -> Angle part of the rotational matrix
    H_idp1_id = H_idip1_RA @ H_RA_idi @ H_idi_RCS @ H_RCS_idip1 @ H_idip1_RA @ H_RA_idi

    rot_alpha = getEulerAngles(H_idp1_id) 
    return rot_alpha

def drawAxisAndMarker(image0,RefBoard,RobotMarker,cam_mtx,cam_dist,Board_corners,Board_ids,Robot_corners,Robot_ids, id_slct_robot):
    #Draw alldetected Markers and axes into the image
    
    # Reference Board
    image_withRefBoard = RefBoard.drawAxis(image0, cam_mtx, cam_dist)
    image_withRefBoard = aruco.drawDetectedMarkers(image_withRefBoard,Board_corners,Board_ids,(255,255,0)) # Farbe Cyan der Marker

    # Roboter Marker
    #image_withRefBoard= image0   
    image_withRobot = RobotMarker.drawAxis(image_withRefBoard, cam_mtx, cam_dist)
    image_withRobot = aruco.drawDetectedMarkers(image_withRobot, Robot_corners,Robot_ids)
   
    return image_withRobot

def drawAxisAndMarker_Debugging(aruco, image0,RefBoard,RobotMarker,cam_mtx,cam_dist,Board_corners,Board_ids,Robot_corners,Robot_ids):
    # Referenzboard
    image_withRefBoard = RefBoard.drawAxis_Debugging(aruco, image0, cam_mtx, cam_dist)
    image_withRefBoard = aruco.drawDetectedMarkers(image_withRefBoard,Board_corners,Board_ids,(255,255,0)) # Farbe Cyan der Marker

    # Robotermarker   
    
    image_withRobot = RobotMarker.drawAxis_Debugging(aruco, image_withRefBoard, cam_mtx, cam_dist)
    image_withRobot = aruco.drawDetectedMarkers(image_withRobot, Robot_corners,Robot_ids)
    
    return image_withRobot
     
def safeImage(image,index):
    # Safe the image into the log folder

    # path to subfolder
    output_folder = "3_Images_MotionTracking"

    # Check if the folder exist. If not -> create it
    if not os.path.exists(output_folder):
        os.makedirs(output_folder)

    # Generate the Filename with current time
    current_time = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    output_filename = f"{index}_MotionTracking_Image_{current_time}.png"

    # Save the image as png-file in the definded subfolder
    output_path = os.path.join(output_folder, output_filename)
    cv2.imwrite(output_path, image)
